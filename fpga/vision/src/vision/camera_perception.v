// V2/V3 board datapath: raw DVP -> quarter-scale RGB + feature grid ->
// immutable frame slots / historical feature tracker -> existing dual UDP MAC.
module camera_perception(
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
    wire fv;wire [15:0] fa,feature;
    vision_feature_grid grid(.clk(camera_pclk),.rst_n(cap_rst_n),.frame_start(os),.pixel_valid(ov),.x(ox),.y(oy),.luma(luma),.red(red),
        .feature_valid(fv),.feature_address(fa),.feature(feature));
    wire start,done,read,first;wire [7:0] payload,gmii;wire [15:0] length,packet_fid;wire tx_valid;
    vision_frame_stream stream(.wr_clk(camera_pclk),.wr_rst_n(cap_rst_n),.frame_start(os),.frame_end(oe),.frame_good(og),.frame_id(ofid),
        .pixel(op),.pixel_valid(ov),.rd_clk(clk125),.rd_rst_n(eth_rst_stream),.payload_read(read),.tx_done(done),.payload_data(payload),
        .payload_length(length),.packet_frame_id(packet_fid),.packet_first(first),.tx_start(start),.dropped_frames());
    wire semantic_update,semantic_target;wire [255:0] semantic_payload;
    wire seed_valid;wire [256:0] seed;
    vision_mailbox #(.WIDTH(257)) seed_mailbox(.src_clk(clk125),.src_rst_n(eth_rst_stream),.src_valid(semantic_update),
        .src_data({semantic_target,semantic_payload}),.dst_clk(camera_pclk),.dst_rst_n(cap_rst_n),.dst_valid(seed_valid),.dst_data(seed));
    wire perception_valid,observation_valid;wire [255:0] perception_payload,observation_payload;
    vision_tracker tracker(.clk(camera_pclk),.rst_n(cap_rst_n),.frame_start(os),.frame_end(oe),.frame_good(og),.frame_id(ofid),
        .feature_valid(fv),.feature_address(fa),.feature(feature),.seed_valid(seed_valid),.seed_data(seed),.status_valid(perception_valid),.status_payload(perception_payload));
    vision_mailbox observation_mailbox(.src_clk(camera_pclk),.src_rst_n(cap_rst_n),.src_valid(perception_valid),.src_data(perception_payload),
        .dst_clk(clk125),.dst_rst_n(eth_rst_stream),.dst_valid(observation_valid),.dst_data(observation_payload));
    wire event_valid,event_ready;wire [255:0] event_payload;wire [3:0] vision_led;
    wire endpoint_ready;
    vision_v23_bridge bridge(.clk(clk125),.rst_n(eth_rst_stream),.stream_ready(endpoint_ready),.semantic_update(semantic_update),
        .semantic_valid(semantic_target),.semantic_payload(semantic_payload),.observation_valid(observation_valid),.observation_payload(observation_payload),
        .event_ready(event_ready),.event_valid(event_valid),.event_payload(event_payload),.led(vision_led));
    generate if(1)begin:runtime_network
        wire [7:0] rx_data;wire rx_valid,rx_error,rx_rst_n,network_rst_n,cache2,enabled2,result_valid,result_done;
        wire [255:0] result_payload,status;
        rgmii_input rx(.rxc(rgmii_rxc),.rxd(rgmii_rxd),.rxctl(rgmii_rxctl),.data(rx_data),.valid(rx_valid),.error(rx_error));
        reset_sync network_reset(.clk(clk125),.arst_n(sys_rst_n && phy_init),.rst_n(network_rst_n));
        reset_sync receive_reset(.clk(rgmii_rxc),.arst_n(sys_rst_n && phy_init),.rst_n(rx_rst_n));
        assign endpoint_ready=cache2 && enabled2;
        network_runtime_dual #(.DEFAULT_IP(0),.VISION_ENABLE(1),.EVENT_ENABLE(1)) net(
            .clk(clk125),.rst_n(network_rst_n),.rx_clk(rgmii_rxc),.rx_rst_n(rx_rst_n),.rx_data(rx_data),.rx_valid(rx_valid),.rx_error(rx_error),
            .video_start(start),.video_first(first),.video_length(length),.video_data(payload),.video_read(read),.video_done(done),
            .tx_data(gmii),.tx_valid(tx_valid),.target_ip(),.target_port(),.cache_hit(),.target_mac(),.enabled(),.arp_requests(),.rx_packets(),.rx_debug(network_debug),
            .target2_ip(),.target2_port(),.enabled2(enabled2),.cache2_hit(cache2),.target2_mac(),
            .vision_valid(result_valid),.vision_payload(result_payload),.vision_status(status),.vision_status_valid(result_done),
            .event_valid(event_valid),.event_payload(event_payload),.event_ready(event_ready));
        vision_result_core #(.MAX_AGE_FRAMES(14)) semantics(.clk(clk125),.rst_n(network_rst_n),.stream_ready(endpoint_ready),
            .frame_tick(start && first),.frame_id(packet_fid),.result_valid(result_valid),.result_payload(result_payload),
            .target_valid(semantic_target),.target_direction(),.target_class(),.target_confidence(),.target_x(),.target_y(),.target_width(),.target_height(),.target_age(),
            .status_payload(status),.result_done(result_done),.accepted_pulse(semantic_update),.accepted_payload(semantic_payload));
    end endgenerate
    rgmii_output output_path(.clk125(clk125),.data(gmii),.enable(tx_valid),.rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen));
    assign led={vision_led,phy_init,cam_done,cam_id_ok,pll_lock};
    assign m1_drv_en=0;assign m2_drv_en=0;assign lidar_moto_pwm=0;
endmodule
