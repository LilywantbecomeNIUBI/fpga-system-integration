// Chassis-independent observer. Input is the existing 320x180 pixel stream.
// Separate history and solver instance from ROI tracking; shared flow_lk_patch RTL.
module flow_observer #(parameter STAGE=2,CLK_HZ=50000000)(
    input clk,rst_n,invalidate,ov,os,oe,og,
    input [15:0] ox,oy,ofid,input [7:0] luma,
    output event_valid,output [255:0] event_payload
);
    wire busy,flow_done,sv,sg,lost,pair_begin,status_valid;wire [15:0] result_id,fx,fy,status_id;
    wire signed [15:0] fu,fv;wire [31:0] cycles,drops;wire [2:0] state,hazard;
    wire stop;wire signed [1:0] steer;wire [7:0] speed;wire [15:0] nl,nc,nr,rl,rc,rr;
    wire flow_pixel=ov && !ox[0] && !oy[0];
    wire [143:0] debug_status;
    flow_pipeline #(.PYRAMID(STAGE==2),.OUTPUT_DRAIN_CYCLES(STAGE>0 ? 4 : 0)) flow(.clk(clk),.rst_n(rst_n),.invalidate(invalidate),.frame_start(os),.frame_end(oe),.frame_good(og),
        .pixel_valid(flow_pixel),.frame_id(ofid),.x(ox>>1),.y(oy>>1),.luma(luma),.busy(busy),.frame_done(flow_done),
        .sample_valid(sv),.sample_good(sg),.history_lost(lost),.pair_begin(pair_begin),.result_id(result_id),
        .flow_x(fx),.flow_y(fy),.flow_u(fu),.flow_v(fv),.cycles(cycles),.dropped_frames(drops),.debug_status(debug_status));
    generate if(STAGE>0)begin:avoidance
        flow_avoidance #(.TIMEOUT_CYCLES(CLK_HZ/10)) safety(.clk(clk),.rst_n(rst_n),.frame_begin(pair_begin),.frame_done(flow_done),.invalidate(lost || invalidate),
            .sample_valid(sv),.sample_good(sg),.frame_id(result_id),.x(fx),.y(fy),.u(fu),.v(fv),
            .status_valid(status_valid),.status_id(status_id),.state(state),.hazard(hazard),.stop_request(stop),.steer(steer),.speed_limit(speed),
            .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr));
    end else begin:raw_only
        assign status_valid=flow_done || lost;assign status_id=result_id;assign state=0;assign hazard=0;
        assign stop=1;assign steer=0;assign speed=0;assign nl=0;assign nc=0;assign nr=0;assign rl=0;assign rc=0;assign rr=0;
    end endgenerate
    wire obs_valid;wire [255:0] obs;
    flow_event_source #(.STAGE(STAGE)) event_source(.clk(clk),.rst_n(rst_n),.frame_begin(pair_begin),.sample_valid(sv),.sample_good(sg),
        .status_valid(status_valid),.frame_id(status_valid ? status_id : result_id),.x(fx),.y(fy),.u(fu),.v(fv),.state(state),.hazard(hazard),.stop_request(stop),.steer(steer),.speed_limit(speed),
        .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr),.cycles(cycles),.dropped_frames(drops),
        .event_valid(obs_valid),.event_payload(obs));
    // Approximately 10Hz diagnostic packets share the existing immutable CDC.
    // Ordinary observations win; diagnostics never alter solver control.
    reg [22:0] debug_timer;reg debug_pending;
    localparam [7:0] DEBUG_STAGE=STAGE;
    wire debug_send=debug_pending && !obs_valid;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin debug_timer<=0;debug_pending<=0;end
        else begin
            if(debug_timer==CLK_HZ/10-1)begin debug_timer<=0;debug_pending<=1;end
            else debug_timer<=debug_timer+1'b1;
            if(debug_send)debug_pending<=0;
        end
    end
    assign event_valid=obs_valid || debug_send;
    assign event_payload=debug_send ? {32'h4f464447,8'd1,DEBUG_STAGE,debug_status,cycles,drops} : obs;
endmodule
