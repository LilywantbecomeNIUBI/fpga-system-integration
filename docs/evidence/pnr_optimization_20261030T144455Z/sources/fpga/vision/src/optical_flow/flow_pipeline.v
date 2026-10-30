// Stream input is the 160x90 active grayscale image, with explicit frame verdict.
module flow_pipeline #(parameter WIDTH=160,HEIGHT=90,STEP=8,MARGIN=8,PYRAMID=0,OUTPUT_DRAIN_CYCLES=0)(
    input clk,rst_n,frame_start,frame_end,frame_good,pixel_valid,
    input [15:0] frame_id,x,y,input [7:0] luma,
    output busy,frame_done,sample_valid,sample_good,history_lost,pair_begin,
    output [15:0] result_id,flow_x,flow_y,output signed [15:0] flow_u,flow_v,
    output [31:0] cycles,dropped_frames
);
    wire pair_start,read_enable,read_current;wire [15:0] pair_id,read_address;wire [7:0] read_data;
    wire grid_busy;
    // Optional downstream result latency is part of the ownership lease. A
    // completed grid must not release its banks/start a new pair while a
    // consumer is still draining tagged results. Default preserves raw flow.
    generate if(OUTPUT_DRAIN_CYCLES>0)begin:output_drain
        localparam BITS=OUTPUT_DRAIN_CYCLES<2 ? 1 : $clog2(OUTPUT_DRAIN_CYCLES+1);
        reg [BITS-1:0] remaining;
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)remaining<=0;
            else if(frame_done)remaining<=OUTPUT_DRAIN_CYCLES;
            else if(remaining!=0)remaining<=remaining-1'b1;
        end
        assign busy=grid_busy || frame_done || remaining!=0;
    end else begin:no_output_drain
        assign busy=grid_busy;
    end endgenerate
    assign pair_begin=pair_start;
    wire read_coarse;
    flow_frame_store #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.PYRAMID(PYRAMID)) store(
        .clk(clk),.rst_n(rst_n),.frame_start(frame_start),.frame_end(frame_end),.frame_good(frame_good),
        .pixel_valid(pixel_valid),.frame_id(frame_id),.x(x),.y(y),.luma(luma),.processing(busy),
        .read_enable(read_enable),.read_current(read_current),.read_coarse(read_coarse),.read_address(read_address),.read_data(read_data),
        .pair_start(pair_start),.pair_id(pair_id),.rejected(history_lost),.dropped_frames(dropped_frames));
    generate if(PYRAMID)begin:two_scale
    flow_grid_pyramid #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.STEP(STEP),.MARGIN(MARGIN)) grid(
        .clk(clk),.rst_n(rst_n),.start(pair_start),.invalidate(history_lost),.frame_id(pair_id),.busy(grid_busy),
        .frame_done(frame_done),.sample_valid(sample_valid),.sample_good(sample_good),.result_id(result_id),
        .x(flow_x),.y(flow_y),.u(flow_u),.v(flow_v),.cycles(cycles),.read_coarse(read_coarse),
        .read_enable(read_enable),.read_current(read_current),.read_address(read_address),.read_data(read_data));
    end else begin:single_scale
    assign read_coarse=0;
    flow_grid #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.STEP(STEP),.MARGIN(MARGIN)) grid(
        .clk(clk),.rst_n(rst_n),.start(pair_start),.invalidate(history_lost),.frame_id(pair_id),.busy(grid_busy),
        .frame_done(frame_done),.sample_valid(sample_valid),.sample_good(sample_good),.result_id(result_id),
        .x(flow_x),.y(flow_y),.u(flow_u),.v(flow_v),.cycles(cycles),
        .read_enable(read_enable),.read_current(read_current),.read_address(read_address),.read_data(read_data));
    end endgenerate
endmodule
