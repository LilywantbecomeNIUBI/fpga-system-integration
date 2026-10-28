module motor_core #(
    parameter integer CLK_HZ=50000000, CONTROL_HZ=100, PWM_HZ=10000,
    parameter integer FILTER_CYCLES=8, WATCHDOG_MS=250,
    parameter integer PWM_LIMIT=300, MAX_COUNTS=4096, STALL_TICKS=50,
    parameter integer REVERSE_TICKS=10
)(
    input wire clk, reset_n, estop_n,
    input wire left_enc_a, left_enc_b, right_enc_a, right_enc_b,
    input wire command_valid,
    input wire [7:0] opcode, sequence_id, command_flags,
    input wire signed [15:0] left_target, right_target,
    input wire [15:0] command_kp, command_ki,
    output wire left_in1,left_in2,right_in1,right_in2,
    output reg armed,
    output reg [15:0] faults,
    output reg [7:0] flags, last_sequence,
    output wire signed [31:0] left_position,right_position,left_velocity,right_velocity,
    output wire signed [15:0] left_applied,right_applied,
    output wire [31:0] left_invalid,right_invalid,
    output reg [31:0] rejected_commands
);
    localparam integer WINDOW=CLK_HZ/CONTROL_HZ;
    localparam integer WATCHDOG_CYCLES=(CLK_HZ/1000)*WATCHDOG_MS;
    integer divider, watchdog;
    reg sample_tick, control_tick;
    reg signed [15:0] request_l,request_r;
    reg [15:0] kp,ki;
    reg [31:0] prev_invalid_l, prev_invalid_r;
    integer stall_l,stall_r;
    (* syn_preserve=1 *) reg stop_meta,stop_sync;
    wire [7:0] sequence_delta=sequence_id-last_sequence;
    wire fresh_sequence=(sequence_delta!=0 && sequence_delta<128);
    wire targets_ok=flags[0] ?
        (left_target>=-1024 && left_target<=1024 && right_target>=-1024 && right_target<=1024) :
        (left_target>=-1000 && left_target<=1000 && right_target>=-1000 && right_target<=1000);
    // Asynchronous assertion kills outputs and arm state, synchronous release.
    wire safety_reset_n=reset_n && estop_n;
    wire enable=armed && faults==0 && stop_sync && safety_reset_n;
    always @(posedge clk or negedge safety_reset_n) begin
        if (!safety_reset_n) begin stop_meta<=0; stop_sync<=0; end
        else begin stop_meta<=1; stop_sync<=stop_meta; end
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin divider<=0; sample_tick<=0; control_tick<=0; end
        else begin
            sample_tick<=0; control_tick<=sample_tick;
            if(divider==WINDOW-1) begin divider<=0; sample_tick<=1; end
            else divider<=divider+1;
        end
    end
    encoder_ab #(.FILTER_CYCLES(FILTER_CYCLES)) enc_l(
        .clk(clk),.reset_n(reset_n),.a(left_enc_a),.b(left_enc_b),.sample_tick(sample_tick),
        .reverse(flags[3]),.position(left_position),.velocity(left_velocity),.invalid_count(left_invalid));
    encoder_ab #(.FILTER_CYCLES(FILTER_CYCLES)) enc_r(
        .clk(clk),.reset_n(reset_n),.a(right_enc_a),.b(right_enc_b),.sample_tick(sample_tick),
        .reverse(flags[4]),.position(right_position),.velocity(right_velocity),.invalid_count(right_invalid));
    wheel_control #(.PWM_LIMIT(PWM_LIMIT),.REVERSE_TICKS(REVERSE_TICKS)) wheel_l(
        .clk(clk),.reset_n(safety_reset_n),.tick(control_tick),.enable(enable),.closed_loop(flags[0]),
        .motor_reverse(flags[1]),.target(request_l),.velocity(left_velocity),.kp(kp),.ki(ki),.applied(left_applied));
    wheel_control #(.PWM_LIMIT(PWM_LIMIT),.REVERSE_TICKS(REVERSE_TICKS)) wheel_r(
        .clk(clk),.reset_n(safety_reset_n),.tick(control_tick),.enable(enable),.closed_loop(flags[0]),
        .motor_reverse(flags[2]),.target(request_r),.velocity(right_velocity),.kp(kp),.ki(ki),.applied(right_applied));
    at8236_pwm #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ)) pwm_l(
        .clk(clk),.reset_n(safety_reset_n),.enable(enable),.command(left_applied),.in1(left_in1),.in2(left_in2));
    at8236_pwm #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ)) pwm_r(
        .clk(clk),.reset_n(safety_reset_n),.enable(enable),.command(right_applied),.in1(right_in1),.in2(right_in2));
    // Boot and external stop both require explicit CLEAR then ARM.
    always @(posedge clk or negedge safety_reset_n) begin
        if (!safety_reset_n) begin
            armed<=0; faults<=16'h0001; flags<=0; last_sequence<=0; kp<=256; ki<=0;
            request_l<=0; request_r<=0; watchdog<=0; stall_l<=0; stall_r<=0;
            prev_invalid_l<=0; prev_invalid_r<=0; rejected_commands<=0;
        end else begin
            if (armed && watchdog<WATCHDOG_CYCLES) watchdog<=watchdog+1;
            if (!armed) begin stall_l<=0; stall_r<=0; end
            if(command_valid) begin
                if(opcode==0) begin // STOP always accepted, even duplicated sequence.
                    armed<=0; request_l<=0; request_r<=0;
                end else if(opcode==3 && !armed && stop_sync && left_target==0 && right_target==0) begin
                    faults<=0; request_l<=0; request_r<=0; watchdog<=0;
                end else if(opcode==1 && !armed && faults==0 && stop_sync &&
                            left_target==0 && right_target==0 && command_flags[7:5]==0 &&
                            command_kp<=4096 && command_ki<=1024 &&
                            (!command_flags[0] || command_kp!=0)) begin
                    armed<=1; flags<=command_flags; kp<=command_kp; ki<=command_ki;
                    last_sequence<=sequence_id; request_l<=0; request_r<=0; watchdog<=0;
                end else if(opcode==2 && enable && fresh_sequence && targets_ok &&
                            flags==command_flags && kp==command_kp && ki==command_ki) begin
                    last_sequence<=sequence_id; request_l<=left_target; request_r<=right_target; watchdog<=0;
                end else rejected_commands<=rejected_commands+1;
            end
            // Safety events have priority over every command, including CLEAR/ARM.
            if(armed && watchdog>=WATCHDOG_CYCLES-1) begin faults[1]<=1; armed<=0; end
            if(control_tick) begin
                prev_invalid_l<=left_invalid; prev_invalid_r<=right_invalid;
                if(armed) begin
                    if(left_invalid-prev_invalid_l>8 || right_invalid-prev_invalid_r>8) begin faults[2]<=1; armed<=0; end
                    if(left_velocity>MAX_COUNTS || left_velocity< -MAX_COUNTS ||
                       right_velocity>MAX_COUNTS || right_velocity< -MAX_COUNTS) begin faults[3]<=1; armed<=0; end
                    if((left_applied>=100 || left_applied<=-100) && left_velocity==0) stall_l<=stall_l+1;
                    else stall_l<=0;
                    if((right_applied>=100 || right_applied<=-100) && right_velocity==0) stall_r<=stall_r+1;
                    else stall_r<=0;
                    if(stall_l>=STALL_TICKS-1 || stall_r>=STALL_TICKS-1) begin faults[4]<=1; armed<=0; end
                end
            end
        end
    end
endmodule
