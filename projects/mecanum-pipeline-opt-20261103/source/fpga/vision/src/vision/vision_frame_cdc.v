// Ordered sparse 320x180 samples + frame verdicts. All frame-store ownership
// lives at rd_clk. No individually synchronized pixel/coordinate/flag buses.
// Overflow is a level/ack handshake: block writer, drain reader, then resume
// only at a new frame start. Camera reset uses the same invalidation protocol.
module vision_frame_cdc #(parameter FIFO_AB=8,TIMEOUT_CYCLES=5000000)(
    input wr_clk,wr_rst_n,capture_ready,frame_start,frame_end,frame_good,pixel_valid,
    input [15:0] frame_id,x,y,input [7:0] luma,input red,
    input rd_clk,rd_rst_n,
    output reg out_start,out_end,out_good,out_valid,output reg [15:0] out_id,out_x,out_y,
    output reg [7:0] out_luma,output reg out_red,output reg invalidate,
    output reg [31:0] overflow_count,abort_count
);
    // {start,end,good,pixel,fid,x[8:0],y[7:0],gray,red} = 46 bits.
    wire [45:0] wd={frame_start,frame_end,frame_good,pixel_valid,frame_id,x[8:0],y[7:0],luma,red};
    wire full,empty,rv;wire [45:0] data;
    reg fault_req,ack0,ack1,accepting;
    reg fault0,fault1,fault_ack;
    wire event_in=frame_start || frame_end || pixel_valid;
    wire we=capture_ready && !fault_req && !ack1 && (accepting || frame_start) && event_in;
    reg reading;
    wire flush=fault1 || fault_ack;
    wire re=!empty && (flush || !reading);
    vision_async_fifo #(.WIDTH(46),.AB(FIFO_AB)) fifo(
        .wr_clk(wr_clk),.wr_rst_n(wr_rst_n),.wr_en(we),.wr_data(wd),.full(full),
        .rd_clk(rd_clk),.rd_rst_n(rd_rst_n),.rd_en(re),.rd_data(data),.rd_valid(rv),.empty(empty));
    always @(posedge wr_clk or negedge wr_rst_n)begin
        if(!wr_rst_n)begin fault_req<=0;ack0<=0;ack1<=0;accepting<=0;overflow_count<=0;end
        else begin
            ack0<=fault_ack;ack1<=ack0;
            if(ack1)begin fault_req<=0;accepting<=0;end
            else if(!capture_ready)begin fault_req<=1;accepting<=0;end
            else if(we && full)begin fault_req<=1;accepting<=0;overflow_count<=overflow_count+1'b1;end
            else if(!fault_req && frame_start)accepting<=1;
        end
    end
    // Serialization keeps simultaneous start/pixel/end events ordered and
    // leaves four processing clocks for feature/pyramid writes before sealing.
    localparam IDLE=0,WAIT_DATA=1,PIXEL=2,END_DRAIN=3;
    reg [1:0] state;reg [45:0] held;reg [2:0] drain;
    reg active,timed_out;reg [22:0] age;
    reg [15:0] active_id,seen;reg [8:0] expected_x;reg [7:0] expected_y;
    wire take_pixel=(state==WAIT_DATA && rv && !data[45] && active) || (state==PIXEL && active);
    wire [45:0] sample=state==PIXEL ? held : data;
    wire expired=!timed_out && age==TIMEOUT_CYCLES-1;
    always @*reading=state!=IDLE;
    always @(posedge rd_clk or negedge rd_rst_n)begin
        if(!rd_rst_n)begin
            fault0<=0;fault1<=0;fault_ack<=0;state<=IDLE;held<=0;drain<=0;
            active<=0;timed_out<=0;age<=0;invalidate<=0;abort_count<=0;
            active_id<=0;seen<=0;expected_x<=0;expected_y<=0;
            out_start<=0;out_end<=0;out_good<=0;out_valid<=0;
            out_id<=0;out_x<=0;out_y<=0;out_luma<=0;out_red<=0;
        end else begin
            fault0<=fault_req;fault1<=fault0;
            out_start<=0;out_end<=0;out_good<=0;out_valid<=0;invalidate<=0;
            if(rv && !flush)begin age<=0;timed_out<=0;end
            else if(!timed_out)age<=age+1'b1;
            if(fault1 && !fault_ack)begin
                fault_ack<=1;invalidate<=1;abort_count<=abort_count+1'b1;
            end else if(fault_ack && !fault1 && empty && !rv)fault_ack<=0;
            if(flush || expired)begin
                state<=IDLE;active<=0;
                if(expired)begin timed_out<=1;invalidate<=1;abort_count<=abort_count+1'b1;end
            end else case(state)
                IDLE:if(re)state<=WAIT_DATA;
                WAIT_DATA:if(rv)begin
                    held<=data;out_id<=data[41:26];
                    if(data[45])begin
                        if(active)begin
                            invalidate<=1;abort_count<=abort_count+1'b1;active<=0;state<=IDLE;
                        end else begin
                            out_start<=1;active<=1;state<=PIXEL;active_id<=data[41:26];
                            seen<=0;expected_x<=0;expected_y<=0;
                        end
                    end else if(active)begin
                        out_valid<=data[42];out_x<={7'd0,data[25:17]};out_y<={8'd0,data[16:9]};
                        out_luma<=data[8:1];out_red<=data[0];
                        if(data[44])begin drain<=4;state<=END_DRAIN;end else state<=IDLE;
                    end else state<=IDLE;
                end
                PIXEL:begin
                    out_valid<=held[42];out_x<={7'd0,held[25:17]};out_y<={8'd0,held[16:9]};
                    out_luma<=held[8:1];out_red<=held[0];
                    if(held[44])begin drain<=4;state<=END_DRAIN;end else state<=IDLE;
                end
                END_DRAIN:if(drain!=0)drain<=drain-1'b1;
                    else begin out_end<=1;out_good<=held[43] && seen==57600;active<=0;state<=IDLE;end
            endcase
            // A valid end alone cannot certify duplicated/reordered/truncated
            // samples or a torn frame ID. Abort before either store can seal it.
            if(!flush && !expired && take_pixel && sample[42])begin
                if(seen>=57600 || sample[41:26]!=active_id || sample[25:17]!=expected_x || sample[16:9]!=expected_y)begin
                    out_valid<=0;active<=0;state<=IDLE;invalidate<=1;abort_count<=abort_count+1'b1;
                end else begin
                    seen<=seen+1'b1;
                    if(expected_x==319)begin expected_x<=0;expected_y<=expected_y+1'b1;end
                    else expected_x<=expected_x+1'b1;
                end
            end
        end
    end
endmodule
