// Independent read-only 48-byte diagnostic frame, CRC16 little endian.
// C3 3C 02 axis seq16 mode=A2 levels 9xuint32 reserved16 crc16.
// Alternating axes at 10 frames/s total; no control or fault connections.
module encoder_diagnostic_source #(parameter integer CLK_HZ=50000000, ENCODER_B_MASK=0, ENCODER_RECOVERY_MASK=0)(
    input wire clk,reset_n,
    input wire [287:0] left_counters,right_counters,
    input wire [3:0] left_levels,right_levels,
    input wire [255:0] left_trace,right_trace,
    input wire [31:0] left_trace_count,right_trace_count,
    output wire [383:0] frame,output wire valid
);
    integer divider;
    reg axis;
    reg [15:0] sequence_id;
    reg [367:0] body;
    reg capture;
    integer trace_divider;
    reg trace_axis;
    reg [15:0] trace_sequence;
    function [7:0] decoder_mode(input lane);
        if(lane ? (ENCODER_RECOVERY_MASK&2)!=0 : (ENCODER_RECOVERY_MASK&1)!=0)decoder_mode=8'hd4;
        else if(lane ? (ENCODER_B_MASK&2)!=0 : (ENCODER_B_MASK&1)!=0)decoder_mode=8'hb2;
        else decoder_mode=8'ha2;
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin
            divider<=0;axis<=0;sequence_id<=0;body<=0;capture<=0;
            trace_divider<=0;trace_axis<=0;trace_sequence<=0;
        end
        else begin
            capture<=0;
            if(trace_divider==CLK_HZ/100-1) trace_divider<=0;
            else trace_divider<=trace_divider+1;
            if(divider==CLK_HZ/10-1)begin
                divider<=0;capture<=1;axis<=~axis;sequence_id<=sequence_id+16'd1;
                body<={16'b0,axis ? right_counters : left_counters,
                    4'b0,axis ? right_levels : left_levels,
                    decoder_mode(axis),sequence_id,
                    axis ? 8'd2 : 8'd1,8'h02,8'h3c,8'hc3};
            end else begin
                divider<=divider+1;
                if(trace_divider==CLK_HZ/100-1)begin
                    capture<=1;trace_axis<=~trace_axis;trace_sequence<=trace_sequence+16'd1;
                    body<={16'b0,trace_axis ? right_trace_count : left_trace_count,
                        trace_axis ? right_trace : left_trace,
                        4'b0,trace_axis ? right_levels : left_levels,
                        decoder_mode(trace_axis),trace_sequence,
                        trace_axis ? 8'd2 : 8'd1,8'h03,8'h3c,8'hc3};
                end
            end
        end
    end
    frame_crc crc(.clk(clk),.reset_n(reset_n),.capture(capture),.body_in(body),.frame(frame),.valid(valid));
endmodule
