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

module motor_telemetry #(parameter integer CLK_HZ=50000000, BAUD=115200, ENCODER_DIAGNOSTICS=0, ENCODER_B_MASK=0, ENCODER_RECOVERY_MASK=0,IMU_ENABLED=0)(
    input wire clk, reset_n, armed,
    input wire [7:0] flags,last_sequence,
    input wire [15:0] faults,
    input wire signed [31:0] left_position,right_position,left_velocity,right_velocity,
    input wire signed [15:0] left_applied,right_applied,
    input wire [31:0] left_invalid,right_invalid,bad_frames,rejected_commands,
    output wire tx,
    input wire [287:0] left_encoder_diagnostics,right_encoder_diagnostics,
    input wire [3:0] left_encoder_levels,right_encoder_levels,
    input wire [255:0] left_encoder_trace,right_encoder_trace,
    input wire [31:0] left_encoder_trace_count,right_encoder_trace_count,
    input wire imu_valid,input wire [383:0] imu_frame,output wire [15:0] imu_drops
);
    wire [383:0] frame;wire valid;
    motor_status_source #(.CLK_HZ(CLK_HZ)) source(.clk(clk),.reset_n(reset_n),.armed(armed),.flags(flags),.last_sequence(last_sequence),.faults(faults),.left_position(left_position),.right_position(right_position),.left_velocity(left_velocity),.right_velocity(right_velocity),.left_applied(left_applied),.right_applied(right_applied),.left_invalid(left_invalid),.right_invalid(right_invalid),.bad_frames(bad_frames),.rejected_commands(rejected_commands),.frame(frame),.valid(valid));
    wire [383:0] diagnostic_frame;wire diagnostic_valid;
    generate if(ENCODER_DIAGNOSTICS) begin : diagnostic_enabled
    encoder_diagnostic_source #(.CLK_HZ(CLK_HZ),.ENCODER_B_MASK(ENCODER_B_MASK),.ENCODER_RECOVERY_MASK(ENCODER_RECOVERY_MASK)) diagnostics(
        .clk(clk),.reset_n(reset_n),.left_counters(left_encoder_diagnostics),.right_counters(right_encoder_diagnostics),
        .left_levels(left_encoder_levels),.right_levels(right_encoder_levels),
        .left_trace(left_encoder_trace),.right_trace(right_encoder_trace),
        .left_trace_count(left_encoder_trace_count),.right_trace_count(right_encoder_trace_count),
        .frame(diagnostic_frame),.valid(diagnostic_valid));
    end else begin : diagnostic_disabled
        assign diagnostic_frame=0;assign diagnostic_valid=0;
    end endgenerate
    frame_uart_mux #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.AUX_ENABLED(IMU_ENABLED)) sender(.clk(clk),.reset_n(reset_n),.motor_valid(valid),.motor_frame(frame),.imu_valid(diagnostic_valid),.imu_frame(diagnostic_frame),.aux_valid(imu_valid),.aux_frame(imu_frame),.aux_drops(imu_drops),.tx(tx),.imu_drops());
endmodule
