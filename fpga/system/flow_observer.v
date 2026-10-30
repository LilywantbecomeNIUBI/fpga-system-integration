// Chassis-independent observer. Input is the existing 320x180 pixel stream.
// Separate history and solver instance from ROI tracking; shared flow_lk_patch RTL.
module flow_observer #(parameter STAGE=2)(
    input camera_pclk,cap_rst_n,ov,os,oe,og,
    input [15:0] ox,oy,ofid,input [7:0] luma,
    output event_valid,output [255:0] event_payload
);
    wire busy,flow_done,sv,sg,lost,pair_begin,status_valid;wire [15:0] result_id,fx,fy,status_id;
    wire signed [15:0] fu,fv;wire [31:0] cycles,drops;wire [2:0] state,hazard;
    wire stop;wire signed [1:0] steer;wire [7:0] speed;wire [15:0] nl,nc,nr,rl,rc,rr;
    wire flow_pixel=ov && !ox[0] && !oy[0];
    flow_pipeline #(.PYRAMID(STAGE==2)) flow(.clk(camera_pclk),.rst_n(cap_rst_n),.frame_start(os),.frame_end(oe),.frame_good(og),
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
    wire obs_valid;wire [255:0] obs;
    flow_event_source #(.STAGE(STAGE)) event_source(.clk(camera_pclk),.rst_n(cap_rst_n),.frame_begin(pair_begin),.sample_valid(sv),.sample_good(sg),
        .status_valid(status_valid),.frame_id(result_id),.x(fx),.y(fy),.u(fu),.v(fv),.state(state),.hazard(hazard),.stop_request(stop),.steer(steer),.speed_limit(speed),
        .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr),.cycles(cycles),.dropped_frames(drops),
        .event_valid(obs_valid),.event_payload(obs));
    assign event_valid=obs_valid;assign event_payload=obs;
endmodule
