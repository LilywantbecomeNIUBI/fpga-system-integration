// V1 semantic result state, entirely in the Ethernet TX clock domain.
// VDET fields use network byte order; image JPEG frame IDs remain unchanged.
module vision_result_core #(
    parameter IMAGE_WIDTH=1280, IMAGE_HEIGHT=720, TARGET_CLASS=39,
    parameter MIN_CONFIDENCE=500, MAX_AGE_FRAMES=30,
    parameter CLK_HZ=125000000, TIMEOUT_MS=500
)(
    input clk, rst_n, stream_ready,
    input frame_tick, input [15:0] frame_id,
    input result_valid, input [255:0] result_payload,
    output reg target_valid, output reg [1:0] target_direction,
    output reg [15:0] target_class, target_confidence,
    output reg [15:0] target_x, target_y, target_width, target_height,
    output [15:0] target_age, output [255:0] status_payload,output reg result_done,
    output reg accepted_pulse,output [255:0] accepted_payload
);
    localparam [31:0] TIMEOUT_CYCLES=(CLK_HZ/1000)*TIMEOUT_MS;
    reg frame_seen, sequence_seen;
    reg [15:0] current_frame, last_frame;
    reg [31:0] idle_cycles, frame_idle_cycles, accepted, rejected;
    reg idle_due,frame_due,frame_expired;
    // reason: 0 reset/query, 1 accepted, 2 no matching target, 3 malformed,
    // 4 stale/future/unavailable, 5 duplicate/out of order, 6 timeout, 7 stream lost.
    reg [7:0] reason;
    reg [255:0] candidate;
    assign accepted_payload=candidate;
    reg [2:0] state;
    reg query_sample,bad_sample,ready_sample,fresh_sample,ordered_sample,match_sample;
    reg [15:0] age_sample,order_sample,last_age_sample;
    reg [1:0] direction_sample;
    wire [31:0] magic=candidate[255:224];
    wire [7:0] version=candidate[223:216], flags=candidate[215:208];
    wire [15:0] fid=candidate[207:192];
    wire [15:0] image_w=candidate[191:176], image_h=candidate[175:160];
    wire [15:0] class_id=candidate[159:144], confidence=candidate[143:128];
    wire [15:0] x1=candidate[127:112], y1=candidate[111:96];
    wire [15:0] x2=candidate[95:80], y2=candidate[79:64];
    wire query=magic==32'h56515259 && candidate[223:0]==0;
    wire malformed=magic!=32'h56444554 || version!=1 || flags>1 ||
        image_w!=IMAGE_WIDTH || image_h!=IMAGE_HEIGHT || candidate[47:0]!=0 ||
        (flags[0] ? (confidence>1000 || x1>=x2 || y1>=y2 || x2>IMAGE_WIDTH || y2>IMAGE_HEIGHT)
                  : candidate[159:48]!=0);
    wire [15:0] last_age=current_frame-last_frame;
    reg [15:0] last_age_r;
    // An acceptance epoch replaces the wide last-frame equality tag. The
    // result FSM accepts at most once per four cycles; this pipeline is one
    // cycle deep, so a one-bit epoch cannot wrap while a tag is in flight.
    reg acceptance_epoch,age_epoch;
    wire [16:0] sum_x={1'b0,x1}+{1'b0,x2}, sum_y={1'b0,y1}+{1'b0,y2};
    wire [18:0] triple_sum_x={2'd0,sum_x}+{1'd0,sum_x,1'b0};
    wire [15:0] box_width=x2-x1, box_height=y2-y1;
    wire target_match=flags[0] && class_id==TARGET_CLASS && confidence>=MIN_CONFIDENCE;
    assign target_age=frame_seen && sequence_seen ? current_frame-last_frame : 16'hffff;
    assign status_payload={32'h56535441,8'd1,4'd0,stream_ready,sequence_seen,frame_seen,target_valid,
        reason,6'd0,target_direction,last_frame,current_frame,target_class,target_confidence,
        target_x,target_y,target_age,16'd0,accepted,rejected};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            target_valid<=0;target_direction<=0;target_class<=0;target_confidence<=0;
            target_x<=0;target_y<=0;target_width<=0;target_height<=0;
            frame_seen<=0;sequence_seen<=0;current_frame<=0;last_frame<=0;
            idle_cycles<=0;frame_idle_cycles<=0;accepted<=0;rejected<=0;reason<=0;
            idle_due<=0;frame_due<=0;frame_expired<=0;
            candidate<=0;state<=0;result_done<=0;accepted_pulse<=0;query_sample<=0;bad_sample<=0;ready_sample<=0;
            fresh_sample<=0;ordered_sample<=0;match_sample<=0;age_sample<=0;order_sample<=0;
            last_age_sample<=0;direction_sample<=0;last_age_r<=0;acceptance_epoch<=0;age_epoch<=0;
        end else begin
            result_done<=0;accepted_pulse<=0;
            last_age_r<=last_age;age_epoch<=acceptance_epoch;
            idle_due<=idle_cycles>=TIMEOUT_CYCLES-2;
            frame_due<=frame_idle_cycles>=TIMEOUT_CYCLES-2;
            frame_expired<=frame_idle_cycles>=TIMEOUT_CYCLES-1;
            if(!frame_expired)frame_idle_cycles<=frame_idle_cycles+1'b1;
            if(frame_tick)begin current_frame<=frame_id;frame_seen<=1;frame_idle_cycles<=0;frame_due<=0;frame_expired<=0;end
            if(!idle_due)idle_cycles<=idle_cycles+1'b1;
            if(target_valid && (idle_due || (!frame_tick && frame_due)))begin
                target_valid<=0;target_direction<=0;reason<=6;
            end
            // The tag prevents an old sequence's pipelined expiry from clearing
            // a result accepted in the meantime. Wall-clock deadlines below
            // retain their original exact cycle; frame-age expiry gains a beat.
            if(sequence_seen && age_epoch==acceptance_epoch && !last_age_r[15] && last_age_r>MAX_AGE_FRAMES)begin
                sequence_seen<=0;target_valid<=0;target_direction<=0;reason<=4;
            end
            if(!stream_ready)begin
                target_valid<=0;target_direction<=0;sequence_seen<=0;reason<=7;
            end
            case(state)
                0:if(result_valid)begin candidate<=result_payload;state<=1;end
                1:begin
                    query_sample<=query;bad_sample<=malformed;
                    ready_sample<=stream_ready && frame_seen && !frame_expired;
                    age_sample<=current_frame-fid;order_sample<=fid-last_frame;last_age_sample<=last_age;
                    match_sample<=target_match;
                    direction_sample<=triple_sum_x < IMAGE_WIDTH*2 ? 1 : triple_sum_x >= IMAGE_WIDTH*4 ? 3 : 2;
                    state<=2;
                end
                2:begin
                    fresh_sample<=!age_sample[15] && age_sample<=MAX_AGE_FRAMES;
                    ordered_sample<=!sequence_seen || (!last_age_sample[15] && last_age_sample>MAX_AGE_FRAMES) ||
                        (order_sample!=0 && !order_sample[15]);
                    state<=3;
                end
                3:begin
                    result_done<=1;state<=0;
                    if(!query_sample)begin
                    if(bad_sample)begin rejected<=rejected+1'b1;reason<=3;end
                    else if(!stream_ready || !ready_sample || !fresh_sample)begin rejected<=rejected+1'b1;reason<=4;end
                    else if(!ordered_sample)begin rejected<=rejected+1'b1;reason<=5;end
                    else begin
                    accepted<=accepted+1'b1;sequence_seen<=1;last_frame<=fid;acceptance_epoch<=!acceptance_epoch;idle_cycles<=0;idle_due<=0;accepted_pulse<=1;
                    target_valid<=match_sample;reason<=match_sample ? 1 : 2;
                    target_class<=class_id;target_confidence<=confidence;
                    target_x<=flags[0] ? sum_x[16:1] : 0;target_y<=flags[0] ? sum_y[16:1] : 0;
                    target_width<=flags[0] ? box_width : 16'd0;target_height<=flags[0] ? box_height : 16'd0;
                    // Compare doubled center without division or rounding at the boundaries.
                    target_direction<=match_sample ? direction_sample : 0;
                    end
                    end
                end
            endcase
        end
    end
endmodule
