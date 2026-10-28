module motor_status_source #(parameter integer CLK_HZ=50000000, BAUD=115200)(
    input wire clk, reset_n, armed,
    input wire [7:0] flags,last_sequence,
    input wire [15:0] faults,
    input wire signed [31:0] left_position,right_position,left_velocity,right_velocity,
    input wire signed [15:0] left_applied,right_applied,
    input wire [31:0] left_invalid,right_invalid,bad_frames,rejected_commands,
    output wire [383:0] frame,output wire valid
);
    reg [367:0] body;
    reg [15:0] packet_count;
    integer divider;
    reg capture;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin body<=0;packet_count<=0;divider<=0;capture<=0;end
        else begin
            capture<=0;
            if(divider==CLK_HZ/20-1)begin
                divider<=0;packet_count<=packet_count+16'd1;capture<=1;
                body<={packet_count,rejected_commands,bad_frames,right_invalid,left_invalid,
                    right_applied,left_applied,right_velocity,left_velocity,right_position,left_position,
                    faults,flags,{7'b0,armed},last_sequence,8'h01,8'h3c,8'hc3};
            end else divider<=divider+1;
        end
    end
    frame_crc crc(.clk(clk),.reset_n(reset_n),.capture(capture),.body_in(body),.frame(frame),.valid(valid));
endmodule

module motor_telemetry #(parameter integer CLK_HZ=50000000, BAUD=115200)(
    input wire clk, reset_n, armed,
    input wire [7:0] flags,last_sequence,
    input wire [15:0] faults,
    input wire signed [31:0] left_position,right_position,left_velocity,right_velocity,
    input wire signed [15:0] left_applied,right_applied,
    input wire [31:0] left_invalid,right_invalid,bad_frames,rejected_commands,
    output wire tx
);
    wire [383:0] frame;wire valid;
    motor_status_source #(.CLK_HZ(CLK_HZ)) source(.clk(clk),.reset_n(reset_n),.armed(armed),.flags(flags),.last_sequence(last_sequence),.faults(faults),.left_position(left_position),.right_position(right_position),.left_velocity(left_velocity),.right_velocity(right_velocity),.left_applied(left_applied),.right_applied(right_applied),.left_invalid(left_invalid),.right_invalid(right_invalid),.bad_frames(bad_frames),.rejected_commands(rejected_commands),.frame(frame),.valid(valid));
    frame_uart_mux #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) sender(.clk(clk),.reset_n(reset_n),.motor_valid(valid),.motor_frame(frame),.imu_valid(1'b0),.imu_frame(384'b0),.tx(tx),.imu_drops());
endmodule
