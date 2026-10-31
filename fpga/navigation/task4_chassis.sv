// Chassis adapter: UART wheel commands + wheel PI + feedback. No sensor drivers or perception policy.
module task4_chassis #(
    parameter integer CLK_HZ=50000000, BAUD=115200, PWM_LIMIT=350
)(
    input wire clk50m,reset_n,uart_rx,
    input wire auxiliary_valid,input wire [383:0] auxiliary_frame,
    output wire [15:0] auxiliary_drops,
    input wire left_enc_a,left_enc_b,right_enc_a,right_enc_b,
    output wire left_in1,left_in2,right_in1,right_in2,uart_tx,
    input wire nav_running,nav_permit,input wire signed [15:0] nav_left,nav_right,
    output wire nav_command_valid,output wire [7:0] nav_opcode,nav_sequence,
    output wire [15:0] nav_arg0,nav_arg1,nav_arg2,nav_arg3,
    output wire signed [31:0] nav_left_position,nav_right_position,output wire nav_armed,
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
    wire [287:0] left_encoder_diagnostics,right_encoder_diagnostics;
    wire [3:0] left_encoder_levels,right_encoder_levels;
    wire [255:0] left_encoder_trace,right_encoder_trace;
    wire [31:0] left_encoder_trace_count,right_encoder_trace_count;
    uart_rx_byte #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) rx(
        .clk(clk50m),.reset_n(reset_sync),.rx(uart_rx),.data(byte_data),.valid(byte_valid),.framing_error(framing_error));
    motor_protocol #(.CLK_HZ(CLK_HZ)) protocol(
        .clk(clk50m),.reset_n(reset_sync),.byte_valid(byte_valid),.framing_error(framing_error),.byte_data(byte_data),
        .command_valid(command_valid),.opcode(opcode),.sequence_id(sequence_id),.flags(command_flags),
        .left_target(left_target),.right_target(right_target),.kp(kp),.ki(ki),.bad_frames(bad_frames));
    assign nav_command_valid=command_valid,nav_opcode=opcode,nav_sequence=sequence_id;
    assign nav_arg0=left_target,nav_arg1=right_target,nav_arg2=kp,nav_arg3=ki;
    assign nav_left_position=left_position,nav_right_position=right_position,nav_armed=armed;
    wire cv,owner,inhibit;wire [7:0] cop,cid,cflags;wire signed [15:0] cl,cr;wire [15:0] ckp,cki;
    wire raw_l1,raw_l2,raw_r1,raw_r2;
    nav_command_mux #(.CLK_HZ(CLK_HZ)) intent_mux(.clk(clk50m),.rst_n(reset_sync),
     .command_valid(command_valid),.opcode(opcode),.sequence_id(sequence_id),.command_flags(command_flags),
     .left_target(left_target),.right_target(right_target),.kp(kp),.ki(ki),.armed(armed),.core_sequence(last_sequence),
     .nav_running(nav_running),.nav_permit(nav_permit),.nav_left(nav_left),.nav_right(nav_right),
     .core_valid(cv),.core_opcode(cop),.core_id(cid),.core_flags(cflags),.core_left(cl),.core_right(cr),.core_kp(ckp),.core_ki(cki),
     .auto_owner(owner),.inhibit(inhibit));
    // Independent supervisor gate. PI state still sees zero intent; gate releases
    // only for a healthy auto intent or after the motor core has disarmed.
    assign left_in1=raw_l1 && !inhibit,left_in2=raw_l2 && !inhibit;
    assign right_in1=raw_r1 && !inhibit,right_in2=raw_r2 && !inhibit;
    motor_core #(.CLK_HZ(CLK_HZ),.PWM_LIMIT(PWM_LIMIT),.LEFT_ENCODER_B(1),.LEFT_ENCODER_RECOVERY(1)) core(
        .clk(clk50m),.reset_n(reset_sync),
        .left_enc_a(left_enc_a),.left_enc_b(left_enc_b),.right_enc_a(right_enc_a),.right_enc_b(right_enc_b),
        .command_valid(cv),.opcode(cop),.sequence_id(cid),.command_flags(cflags),
        .left_target(cl),.right_target(cr),.command_kp(ckp),.command_ki(cki),
        .left_in1(raw_l1),.left_in2(raw_l2),.right_in1(raw_r1),.right_in2(raw_r2),
        .armed(armed),.faults(faults),.flags(flags),.last_sequence(last_sequence),
        .left_position(left_position),.right_position(right_position),.left_velocity(left_velocity),.right_velocity(right_velocity),
        .left_applied(left_applied),.right_applied(right_applied),.left_invalid(left_invalid),.right_invalid(right_invalid),
        .rejected_commands(rejected_commands),
        .left_encoder_diagnostics(left_encoder_diagnostics),.right_encoder_diagnostics(right_encoder_diagnostics),
        .left_encoder_levels(left_encoder_levels),.right_encoder_levels(right_encoder_levels),
        .left_encoder_trace(left_encoder_trace),.right_encoder_trace(right_encoder_trace),
        .left_encoder_trace_count(left_encoder_trace_count),.right_encoder_trace_count(right_encoder_trace_count));
    motor_telemetry #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.ENCODER_DIAGNOSTICS(1),.ENCODER_B_MASK(1),.ENCODER_RECOVERY_MASK(1),.IMU_ENABLED(1)) telemetry(
        .clk(clk50m),.reset_n(reset_sync),.armed(armed),.flags(flags),.last_sequence(last_sequence),.faults(faults),
        .left_position(left_position),.right_position(right_position),.left_velocity(left_velocity),.right_velocity(right_velocity),
        .left_applied(left_applied),.right_applied(right_applied),.left_invalid(left_invalid),.right_invalid(right_invalid),
        .bad_frames(bad_frames),.rejected_commands(rejected_commands),.tx(uart_tx),
        .imu_frame(auxiliary_frame),.imu_valid(auxiliary_valid),.imu_drops(auxiliary_drops),
        .left_encoder_diagnostics(left_encoder_diagnostics),.right_encoder_diagnostics(right_encoder_diagnostics),
        .left_encoder_levels(left_encoder_levels),.right_encoder_levels(right_encoder_levels),
        .left_encoder_trace(left_encoder_trace),.right_encoder_trace(right_encoder_trace),
        .left_encoder_trace_count(left_encoder_trace_count),.right_encoder_trace_count(right_encoder_trace_count));
    assign led={faults[4:0],flags[0],armed,reset_sync};
endmodule
