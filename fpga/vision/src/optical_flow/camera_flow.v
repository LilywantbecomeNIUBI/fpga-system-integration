// Separate optical-flow board profile. Reuses the proven camera and dual UDP path.
module camera_flow #(parameter STAGE=2)(
    input clk50m,reset_n,rgmii_rxc,input [3:0] rgmii_rxd,input rgmii_rxctl,
    output camera_sclk,inout camera_sdat,input camera_vsync,camera_href,camera_pclk,input [7:0] camera_data,
    output camera_rst_n,camera_pwdn,rgmii_gtxc,output [3:0] rgmii_txd,output rgmii_txen,
    output eth_mdc,inout eth_mdio,output eth_rst_n,output [7:0] led,output [31:0] network_debug,
    output m1_drv_en,m2_drv_en,lidar_moto_pwm
);
    wire clk50,clk125,unused_xclk,pll_lock,sys_rst_n,phy_init,cam_done,cam_id_ok,cam_error;
    wire [15:0] phy_status;
    ethernet_pll pll(.lock(pll_lock),.clkout0(clk50),.clkout1(clk125),.clkout2(unused_xclk),.clkin(clk50m),.reset(~reset_n));
    reset_sync sys_reset(.clk(clk50),.arst_n(reset_n && pll_lock),.rst_n(sys_rst_n));
    phy_reg_config phy(.Clk(clk50),.Rst_n(sys_rst_n),.Phy_rst_n(eth_rst_n),.Rd_Data(phy_status),.Phy_init_done(phy_init),.mdc(eth_mdc),.mdio(eth_mdio));
    ov5640_config #(.IMAGE_WIDTH(1280),.IMAGE_HEIGHT(720),.IMAGE_FLIP_EN(1),.IMAGE_MIRROR_EN(1),.OUTPUT_JPEG(0),.FRAME_RATE(30)) config_camera(
        .clk(clk50),.rst_n(sys_rst_n),.camera_rst_n(camera_rst_n),.camera_pwdn(camera_pwdn),.sclk(camera_sclk),.sdat(camera_sdat),
        .id_ok(cam_id_ok),.done(cam_done),.error(cam_error));
    wire cap_rst_n,eth_rst_stream;
    wire common_reset_n=sys_rst_n && phy_init && cam_done;
    reset_sync cap_reset(.clk(camera_pclk),.arst_n(common_reset_n),.rst_n(cap_rst_n));
    reset_sync perception_reset(.clk(clk125),.arst_n(common_reset_n),.rst_n(eth_rst_stream));
    wire pv,fs,fe,fg,format_error;wire [15:0] pixel,x,y,fid;
    vision_dvp_capture capture(.clk(camera_pclk),.rst_n(cap_rst_n),.vsync(camera_vsync),.href(camera_href),.data(camera_data),
        .pixel_valid(pv),.pixel(pixel),.x(x),.y(y),.frame_id(fid),.frame_start(fs),.frame_end(fe),.frame_good(fg),.format_error(format_error));
    wire ov,os,oe,og,red;wire [15:0] op,ox,oy,ofid;wire [7:0] luma;
    vision_pixel_pipeline pixels(.clk(camera_pclk),.rst_n(cap_rst_n),.pixel_valid(pv),.pixel(pixel),.x(x),.y(y),.frame_id(fid),
        .frame_start(fs),.frame_end(fe),.frame_good(fg),.out_valid(ov),.out_pixel(op),.out_x(ox),.out_y(oy),.out_frame_id(ofid),
        .luma(luma),.red(red),.out_start(os),.out_end(oe),.out_good(og));
    wire busy,flow_done,sv,sg,lost,pair_begin,status_valid;wire [15:0] result_id,fx,fy,status_id;
    wire signed [15:0] fu,fv;wire [31:0] cycles,drops;wire [2:0] state,hazard;
    wire stop;wire signed [1:0] steer;wire [7:0] speed;wire [15:0] nl,nc,nr,rl,rc,rr;
    wire flow_pixel=ov && !ox[0] && !oy[0];
    flow_pipeline #(.PYRAMID(STAGE==2),.OUTPUT_DRAIN_CYCLES(STAGE>0 ? 4 : 0)) flow(.clk(camera_pclk),.rst_n(cap_rst_n),.invalidate(1'b0),.frame_start(os),.frame_end(oe),.frame_good(og),
        .pixel_valid(flow_pixel),.frame_id(ofid),.x(ox>>1),.y(oy>>1),.luma(luma),.busy(busy),.frame_done(flow_done),
        .sample_valid(sv),.sample_good(sg),.history_lost(lost),.pair_begin(pair_begin),.result_id(result_id),
        .flow_x(fx),.flow_y(fy),.flow_u(fu),.flow_v(fv),.cycles(cycles),.dropped_frames(drops));
    generate if(STAGE>0)begin:avoidance
        flow_avoidance safety(.clk(camera_pclk),.rst_n(cap_rst_n),.frame_begin(pair_begin),.frame_done(flow_done),.invalidate(lost),
            .sample_valid(sv),.sample_good(sg),.frame_id(result_id),.x(fx),.y(fy),.u(fu),.v(fv),
            .status_valid(status_valid),.status_id(status_id),.state(state),.hazard(hazard),.stop_request(stop),.steer(steer),.speed_limit(speed),
            .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr));
    end else begin:raw_only
        assign status_valid=flow_done || lost;assign status_id=result_id;assign state=0;assign hazard=0;
        assign stop=1;assign steer=0;assign speed=0;assign nl=0;assign nc=0;assign nr=0;assign rl=0;assign rc=0;assign rr=0;
    end endgenerate
    wire obs_valid,observation_valid;wire [255:0] obs,observation;
    flow_event_source #(.STAGE(STAGE)) event_source(.clk(camera_pclk),.rst_n(cap_rst_n),.frame_begin(pair_begin),.sample_valid(sv),.sample_good(sg),
        .status_valid(status_valid),.frame_id(status_valid ? status_id : result_id),.x(fx),.y(fy),.u(fu),.v(fv),.state(state),.hazard(hazard),.stop_request(stop),.steer(steer),.speed_limit(speed),
        .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr),.cycles(cycles),.dropped_frames(drops),
        .event_valid(obs_valid),.event_payload(obs));
    vision_mailbox observation_mailbox(.src_clk(camera_pclk),.src_rst_n(cap_rst_n),.src_valid(obs_valid),.src_data(obs),
        .dst_clk(clk125),.dst_rst_n(eth_rst_stream),.dst_valid(observation_valid),.dst_data(observation));
    wire event_valid,event_ready,endpoint_ready,control_stop;wire [255:0] event_payload;
    wire signed [7:0] control_steer;wire [7:0] control_speed;wire [2:0] control_state;
    flow_event_bridge #(.STAGE(STAGE)) bridge(.clk(clk125),.rst_n(eth_rst_stream),.stream_ready(endpoint_ready),
        .observation_valid(observation_valid),.observation_payload(observation),.event_ready(event_ready),.event_valid(event_valid),.event_payload(event_payload),
        .stop_request(control_stop),.steer(control_steer),.speed_limit(control_speed),.control_state(control_state));
    wire start,done,read,first;wire [7:0] payload,gmii;wire [15:0] length,packet_fid;wire tx_valid;
    vision_frame_stream stream(.wr_clk(camera_pclk),.wr_rst_n(cap_rst_n),.frame_start(os),.frame_end(oe),.frame_good(og),.frame_id(ofid),
        .pixel(op),.pixel_valid(ov),.rd_clk(clk125),.rd_rst_n(eth_rst_stream),.payload_read(read),.tx_done(done),.payload_data(payload),
        .payload_length(length),.packet_frame_id(packet_fid),.packet_first(first),.tx_start(start),.dropped_frames());
    generate if(1)begin:runtime_network
        wire [7:0] rx_data;wire rx_valid,rx_error,rx_rst_n,network_rst_n,cache2,enabled2,result_valid,result_done;
        wire [255:0] result_payload,semantic_status;
        rgmii_input rx(.rxc(rgmii_rxc),.rxd(rgmii_rxd),.rxctl(rgmii_rxctl),.data(rx_data),.valid(rx_valid),.error(rx_error));
        reset_sync network_reset(.clk(clk125),.arst_n(sys_rst_n && phy_init),.rst_n(network_rst_n));
        reset_sync receive_reset(.clk(rgmii_rxc),.arst_n(sys_rst_n && phy_init),.rst_n(rx_rst_n));
        assign endpoint_ready=cache2 && enabled2;
        // Preserve the validated VDET transport configuration; semantics remain independent of LK.
        network_runtime_dual #(.DEFAULT_IP(0),.VISION_ENABLE(1),.EVENT_ENABLE(1)) net(
            .clk(clk125),.rst_n(network_rst_n),.rx_clk(rgmii_rxc),.rx_rst_n(rx_rst_n),.rx_data(rx_data),.rx_valid(rx_valid),.rx_error(rx_error),
            .video_start(start),.video_first(first),.video_length(length),.video_data(payload),.video_read(read),.video_done(done),
            .tx_data(gmii),.tx_valid(tx_valid),.target_ip(),.target_port(),.cache_hit(),.target_mac(),.enabled(),.arp_requests(),.rx_packets(),.rx_debug(network_debug),
            .target2_ip(),.target2_port(),.enabled2(enabled2),.cache2_hit(cache2),.target2_mac(),
            .vision_valid(result_valid),.vision_payload(result_payload),.vision_status(semantic_status),.vision_status_valid(result_done),
            .event_valid(event_valid),.event_payload(event_payload),.event_ready(event_ready));
        vision_result_core #(.MAX_AGE_FRAMES(14)) semantics(.clk(clk125),.rst_n(network_rst_n),.stream_ready(endpoint_ready),
            .frame_tick(start && first),.frame_id(packet_fid),.result_valid(result_valid),.result_payload(result_payload),
            .target_valid(),.target_direction(),.target_class(),.target_confidence(),.target_x(),.target_y(),.target_width(),.target_height(),.target_age(),
            .status_payload(semantic_status),.result_done(result_done),.accepted_pulse(),.accepted_payload());
    end endgenerate
    rgmii_output output_path(.clk125(clk125),.data(gmii),.enable(tx_valid),.rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen));
    assign led={control_stop,control_state,phy_init,cam_done,cam_id_ok,pll_lock};
    // The separate FOC/controller project is not connected to this observation profile.
    assign m1_drv_en=0;assign m2_drv_en=0;assign lidar_moto_pwm=0;
endmodule
