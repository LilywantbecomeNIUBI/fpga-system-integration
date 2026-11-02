module flow_v2_core(
    input clk,rst_n,frame_start,frame_end,frame_good,pixel_valid,
    input [15:0] frame_id,x,y,input [7:0] luma,
    output busy,frame_done,sample_valid,sample_good,history_lost,
    output [15:0] result_id,flow_x,flow_y,output signed [15:0] flow_u,flow_v,
    output [31:0] cycles,dropped_frames,
    output status_valid,output [15:0] status_id,output [2:0] state,hazard,
    output stop_request,output signed [1:0] steer,output [7:0] speed_limit,
    output [15:0] left_valid,center_valid,right_valid,left_risk,center_risk,right_risk
);
    flow_avoidance_top #(.PYRAMID(1)) core(.clk(clk),.rst_n(rst_n),.frame_start(frame_start),.frame_end(frame_end),
        .frame_good(frame_good),.pixel_valid(pixel_valid),.frame_id(frame_id),.x(x),.y(y),.luma(luma),
        .busy(busy),.frame_done(frame_done),.sample_valid(sample_valid),.sample_good(sample_good),.history_lost(history_lost),
        .result_id(result_id),.flow_x(flow_x),.flow_y(flow_y),.flow_u(flow_u),.flow_v(flow_v),.cycles(cycles),.dropped_frames(dropped_frames),
        .status_valid(status_valid),.status_id(status_id),.state(state),.hazard(hazard),.stop_request(stop_request),.steer(steer),.speed_limit(speed_limit),
        .left_valid(left_valid),.center_valid(center_valid),.right_valid(right_valid),.left_risk(left_risk),.center_risk(center_risk),.right_risk(right_risk));
endmodule
