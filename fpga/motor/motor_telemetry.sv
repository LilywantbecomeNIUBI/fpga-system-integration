module motor_telemetry #(parameter integer CLK_HZ=50000000, BAUD=115200)(
    input wire clk, reset_n, armed,
    input wire [7:0] flags,last_sequence,
    input wire [15:0] faults,
    input wire signed [31:0] left_position,right_position,left_velocity,right_velocity,
    input wire signed [15:0] left_applied,right_applied,
    input wire [31:0] left_invalid,right_invalid,bad_frames,rejected_commands,
    output wire tx
);
    function automatic [15:0] crc_byte(input [15:0] old,input [7:0] value);
        reg [15:0] c; integer j;
        begin c=old^{value,8'b0}; for(j=0;j<8;j=j+1) c=c[15]?(c<<1)^16'h1021:c<<1; crc_byte=c; end
    endfunction
    reg [367:0] snapshot;
    reg [15:0] crc,packet_count;
    reg [5:0] index;
    reg [2:0] state;
    integer divider;
    reg [7:0] data;
    reg start;
    wire busy;
    uart_tx_byte #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) uart(.clk(clk),.reset_n(reset_n),.start(start),.data(data),.tx(tx),.busy(busy));
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            snapshot<=0; crc<=16'hffff; packet_count<=0; index<=0; state<=0;
            divider<=0; data<=0; start<=0;
        end else begin
            start<=0;
            if(divider<CLK_HZ/20) divider<=divider+1;
            case(state)
                0: if(divider>=CLK_HZ/20) begin
                    divider<=0; packet_count<=packet_count+16'd1;
                    snapshot<={packet_count,rejected_commands,bad_frames,right_invalid,left_invalid,
                        right_applied,left_applied,right_velocity,left_velocity,right_position,left_position,
                        faults,flags,{7'b0,armed},last_sequence,8'h01,8'h3c,8'hc3};
                    index<=0; crc<=16'hffff; state<=1;
                end
                1: if(!busy) begin
                    if(index<46) begin data<=snapshot[index*8+:8]; crc<=crc_byte(crc,snapshot[index*8+:8]); end
                    else if(index==46) data<=crc[7:0]; else data<=crc[15:8];
                    start<=1; state<=2;
                end
                2: if(busy) state<=3;
                3: if(!busy) begin
                    if(index==47) state<=0;
                    else begin index<=index+6'd1; state<=1; end
                end
                default: state<=0;
            endcase
        end
    end
endmodule
