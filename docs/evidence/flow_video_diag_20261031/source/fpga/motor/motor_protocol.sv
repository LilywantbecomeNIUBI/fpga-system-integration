// Fixed 16-byte command frame with CRC16-CCITT-FALSE; partial frames expire.
module motor_protocol #(parameter integer CLK_HZ=50000000)(
    input wire clk, reset_n, byte_valid, framing_error,
    input wire [7:0] byte_data,
    output reg command_valid,
    output reg [7:0] opcode, sequence_id, flags,
    output reg signed [15:0] left_target, right_target,
    output reg [15:0] kp, ki,
    output reg [31:0] bad_frames
);
    function automatic [15:0] crc_byte(input [15:0] old, input [7:0] value);
        reg [15:0] c; integer j;
        begin c=old^{value,8'b0}; for(j=0;j<8;j=j+1) c=c[15] ? (c<<1)^16'h1021 : c<<1; crc_byte=c; end
    endfunction
    reg [7:0] payload[0:13];
    reg [4:0] index;
    reg [15:0] crc;
    reg [7:0] crc_low;
    integer idle_count;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            index<=0; crc<=16'hffff; crc_low<=0; idle_count<=0; command_valid<=0;
            opcode<=0; sequence_id<=0; flags<=0; left_target<=0; right_target<=0;
            kp<=0; ki<=0; bad_frames<=0;
        end else begin
            command_valid<=0;
            if (index!=0) idle_count<=idle_count+1;
            if (framing_error || (index!=0 && idle_count>=CLK_HZ/50)) begin
                index<=0; idle_count<=0; bad_frames<=bad_frames+1;
            end else if (byte_valid) begin
                idle_count<=0;
                if (index==0) begin
                    if (byte_data==8'ha5) begin index<=1; crc<=crc_byte(16'hffff,byte_data); end
                end else if (index==1 && byte_data!=8'h5a) begin
                    if (byte_data==8'ha5) crc<=crc_byte(16'hffff,byte_data);
                    else index<=0;
                end else if (index<14) begin
                    payload[index]<=byte_data; crc<=crc_byte(crc,byte_data); index<=index+5'd1;
                end else if (index==14) begin crc_low<=byte_data; index<=15; end
                else begin
                    index<=0;
                    if ({byte_data,crc_low}==crc && payload[2]==1) begin
                        opcode<=payload[3]; sequence_id<=payload[4]; flags<=payload[5];
                        left_target<={payload[7],payload[6]}; right_target<={payload[9],payload[8]};
                        kp<={payload[11],payload[10]}; ki<={payload[13],payload[12]}; command_valid<=1;
                    end else bad_frames<=bad_frames+1;
                end
            end
        end
    end
endmodule
