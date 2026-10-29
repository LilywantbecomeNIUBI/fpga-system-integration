module brushed_motor_top #(
    parameter integer CLK_HZ=50000000, BAUD=115200
)(
    input wire clk50m,reset_n,estop_n,uart_rx,
    input wire left_enc_a,left_enc_b,right_enc_a,right_enc_b,
    output wire left_in1,left_in2,right_in1,right_in2,uart_tx,
    output wire legacy_m1_disable,legacy_m2_disable,
    output wire [5:0] legacy_m1_pwm_disabled,legacy_m2_pwm_disabled,
    output wire [1:0] legacy_cal_disabled,
    output wire [7:0] led
);
    // Async assertion, two-clock release. Input CDC endpoints are in the SDC.
    (* syn_preserve=1 *) reg reset_meta, reset_sync;
    always @(posedge clk50m or negedge reset_n) begin
        if(!reset_n) begin reset_meta<=0; reset_sync<=0; end
        else begin reset_meta<=1; reset_sync<=reset_meta; end
    end
    wire [7:0] byte_data,opcode,sequence_id,command_flags,flags,last_sequence;
    wire byte_valid,framing_error,command_valid,armed;
    wire signed [15:0] left_target,right_target,left_applied,right_applied;
    wire [15:0] kp,ki,faults;
    wire signed [31:0] left_position,right_position,left_velocity,right_velocity;
    wire [31:0] left_invalid,right_invalid,bad_frames,rejected_commands;
    uart_rx_byte #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) rx(
        .clk(clk50m),.reset_n(reset_sync),.rx(uart_rx),.data(byte_data),.valid(byte_valid),.framing_error(framing_error));
    motor_protocol #(.CLK_HZ(CLK_HZ)) protocol(
        .clk(clk50m),.reset_n(reset_sync),.byte_valid(byte_valid),.framing_error(framing_error),.byte_data(byte_data),
        .command_valid(command_valid),.opcode(opcode),.sequence_id(sequence_id),.flags(command_flags),
        .left_target(left_target),.right_target(right_target),.kp(kp),.ki(ki),.bad_frames(bad_frames));
    motor_core #(.CLK_HZ(CLK_HZ)) core(
        .clk(clk50m),.reset_n(reset_sync),.estop_n(estop_n),
        .left_enc_a(left_enc_a),.left_enc_b(left_enc_b),.right_enc_a(right_enc_a),.right_enc_b(right_enc_b),
        .command_valid(command_valid),.opcode(opcode),.sequence_id(sequence_id),.command_flags(command_flags),
        .left_target(left_target),.right_target(right_target),.command_kp(kp),.command_ki(ki),
        .left_in1(left_in1),.left_in2(left_in2),.right_in1(right_in1),.right_in2(right_in2),
        .armed(armed),.faults(faults),.flags(flags),.last_sequence(last_sequence),
        .left_position(left_position),.right_position(right_position),.left_velocity(left_velocity),.right_velocity(right_velocity),
        .left_applied(left_applied),.right_applied(right_applied),.left_invalid(left_invalid),.right_invalid(right_invalid),
        .rejected_commands(rejected_commands));
    motor_telemetry #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) telemetry(
        .clk(clk50m),.reset_n(reset_sync),.armed(armed),.flags(flags),.last_sequence(last_sequence),.faults(faults),
        .left_position(left_position),.right_position(right_position),.left_velocity(left_velocity),.right_velocity(right_velocity),
        .left_applied(left_applied),.right_applied(right_applied),.left_invalid(left_invalid),.right_invalid(right_invalid),
        .bad_frames(bad_frames),.rejected_commands(rejected_commands),.tx(uart_tx));
    assign legacy_m1_disable=0, legacy_m2_disable=0;
    assign legacy_m1_pwm_disabled=0, legacy_m2_pwm_disabled=0, legacy_cal_disabled=0;
    assign led={faults[4:0],flags[0],armed,reset_sync};
endmodule
