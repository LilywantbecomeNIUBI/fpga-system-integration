// Real M2 sensor diagnostics. No camera, planner, command input or motor actuation.
// All twelve physical motor outputs are constants; M2 encoder/radar remain inputs.
module m2_sensor_diag_top #(
    parameter integer CLK_HZ=50000000, BAUD=115200, PWM_LIMIT=350
)(
    input wire clk50m,reset_n,lidar_rx,
    inout wire imu_i2c_scl,imu_i2c_sda,
    input wire left_enc_a,left_enc_b,right_enc_a,right_enc_b,
    output wire left_in1,left_in2,right_in1,right_in2,uart_tx,
    output wire legacy_m1_disable,
    output wire [5:0] legacy_m1_pwm_disabled,
    output wire [0:0] legacy_cal_disabled,
    output wire [7:0] led
);
    // Async assertion, two-clock release. Input CDC endpoints are in the SDC.
    (* syn_preserve=1 *) reg reset_meta, reset_sync;
    always @(posedge clk50m or negedge reset_n) begin
        if(!reset_n) begin reset_meta<=0; reset_sync<=0; end
        else begin reset_meta<=1; reset_sync<=reset_meta; end
    end
    wire [383:0] imu_frame;wire imu_valid;wire [15:0] imu_drops;
    wire nav_command_valid,nav_running,nav_done,nav_permit,nav_update,nav_armed;
    wire [7:0] nav_opcode,nav_sequence;wire [15:0] nav_a0,nav_a1,nav_a2,nav_a3,nav_faults,nav_front;
    wire signed [31:0] nav_lp,nav_rp,nav_x,nav_y,nav_theta;
    wire signed [15:0] nav_left,nav_right,nav_v,nav_w;wire [4:0] nav_index;wire [71:0] nav_blocked;
    wire [31:0] nav_rejected,nav_plans;
    wire [383:0] nav_frame;wire nav_frame_valid;
    wire [383:0] auxiliary_frame,lidar_frame;wire auxiliary_valid,auxiliary_ready,lidar_valid,lidar_ready;
    reg [383:0] pending_imu,pending_nav;reg imu_pending,nav_pending;
    // A complete-frame handshake prevents radar chunks being overwritten by
    // periodic IMU/nav snapshots. Motor and encoder packets retain priority.
    assign auxiliary_valid=auxiliary_ready && (imu_pending || nav_pending || lidar_valid);
    assign auxiliary_frame=imu_pending ? pending_imu : nav_pending ? pending_nav : lidar_frame;
    assign lidar_ready=auxiliary_ready && !imu_pending && !nav_pending;
    always @(posedge clk50m or negedge reset_sync)begin
        if(!reset_sync)begin pending_imu<=0;pending_nav<=0;imu_pending<=0;nav_pending<=0;end
        else begin
            if(auxiliary_valid && imu_pending)imu_pending<=0;
            else if(auxiliary_valid && nav_pending)nav_pending<=0;
            if(imu_valid)begin pending_imu<=imu_frame;imu_pending<=1;end
            if(nav_frame_valid)begin pending_nav<=nav_frame;nav_pending<=1;end
        end
    end
    wire diag_l1,diag_l2,diag_r1,diag_r2;
    wire [7:0] chassis_led;
    assign {left_in1,left_in2,right_in1,right_in2}=4'b0000;
    assign led={lpackets!=0,lerrors!=0,imu_stale,imu_fault,imu_ready,sample_seq[0],chassis_led[1:0]};
    // Only the auxiliary framed telemetry crosses this boundary. IMU/perception
    // health never bypasses manual ARM, STOP, command lease or wheel watchdog.
    task4_chassis #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.PWM_LIMIT(PWM_LIMIT)) chassis(
        .clk50m(clk50m),.reset_n(reset_n),.uart_rx(1'b1),.uart_tx(uart_tx),
        .left_enc_a(left_enc_a),.left_enc_b(left_enc_b),.right_enc_a(right_enc_a),.right_enc_b(right_enc_b),
        .left_in1(diag_l1),.left_in2(diag_l2),.right_in1(diag_r1),.right_in2(diag_r2),
        .auxiliary_frame(auxiliary_frame),.auxiliary_valid(auxiliary_valid),.auxiliary_drops(imu_drops),.auxiliary_ready(auxiliary_ready),
        .nav_running(nav_running),.nav_permit(nav_permit),.nav_left(nav_left),.nav_right(nav_right),
        .nav_command_valid(nav_command_valid),.nav_opcode(nav_opcode),.nav_sequence(nav_sequence),
        .nav_arg0(nav_a0),.nav_arg1(nav_a1),.nav_arg2(nav_a2),.nav_arg3(nav_a3),
        .nav_left_position(nav_lp),.nav_right_position(nav_rp),.nav_armed(nav_armed),.led(chassis_led));
    wire imu_ready,imu_fault,imu_sample_valid,imu_stale,imu_temperature_valid;
    wire signed [15:0] ax,ay,az,gx,gy,gz,temperature_raw;
    wire [31:0] sample_seq,timestamp_us,now_us;
    wire [7:0] who_am_i,last_error;
    wire [15:0] error_count,invalid_count,observe_to_commit_us,sample_interval_us;
    imu_1khz_core imu(.clk(clk50m),.rst_n(reset_sync),.imu_i2c_scl(imu_i2c_scl),.imu_i2c_sda(imu_i2c_sda),
        .ready(imu_ready),.fault(imu_fault),.sample_valid(imu_sample_valid),.sample_stale(imu_stale),.temperature_valid(imu_temperature_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temperature_raw),
        .sample_seq(sample_seq),.timestamp_us(timestamp_us),.now_us(now_us),.who_am_i(who_am_i),.last_error(last_error),
        .error_count(error_count),.invalid_count(invalid_count),.observe_to_commit_us(observe_to_commit_us),.sample_interval_us(sample_interval_us),
        .commit_timestamp_us(),.sample_age_us());
    imu_status_source #(.CLK_HZ(CLK_HZ),.DECIMATION(20)) imu_status(.clk(clk50m),.reset_n(reset_sync),.sample_valid(imu_sample_valid),
        .ready(imu_ready),.fault(imu_fault),.sample_stale(imu_stale),.temperature_valid(imu_temperature_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temperature_raw),.sample_seq(sample_seq),
        .timestamp_us(timestamp_us),.now_us(now_us),.who_am_i(who_am_i),.last_error(last_error),.error_count(error_count),
        .invalid_count(invalid_count),.observe_to_commit_us(observe_to_commit_us),.sample_interval_us(sample_interval_us),
        .drop_count(imu_drops),.frame(imu_frame),.valid(imu_valid));
    wire lbv,lerr,lpv;wire [7:0] lbyte,lint;wire [15:0] lang,ldist;wire [31:0] lpackets,lerrors,lcrc,lformat,ltimeouts;
    stl27l_uart_rx #(.CLKS_PER_BIT((CLK_HZ+460800)/921600)) lidar_uart(
     .clk(clk50m),.rst_n(reset_sync),.rx(lidar_rx),.byte_valid(lbv),.byte_data(lbyte),.framing_error(lerr));
    stl27l_node_parser #(.BYTE_TIMEOUT_CYCLES(CLK_HZ/500)) lidar_parser(
     .clk(clk50m),.rst_n(reset_sync),.byte_valid(lbv),.byte_in(lbyte),.stream_error(lerr),
     .point_valid(lpv),.distance_mm(ldist),.intensity(lint),.angle_q100(lang),.packet_count(lpackets),.error_count(lerrors),
     .crc_error_count(lcrc),.format_error_count(lformat),.timeout_count(ltimeouts));
    lidar_scan_source #(.CLK_HZ(CLK_HZ)) lidar_display(
     .clk(clk50m),.rst_n(reset_sync),.byte_valid(lbv),.framing_error(lerr),.point_valid(lpv),
     .angle(lang),.distance(ldist),.intensity(lint),.packets(lpackets),.errors(lerrors),
     .crc_errors(lcrc),.format_errors(lformat),.timeouts(ltimeouts),
     .frame(lidar_frame),.valid(lidar_valid),.ready(lidar_ready));
    // Static diagnostics: report standby and raw radar counters only.
    assign nav_running=0,nav_done=0,nav_permit=0,nav_update=0;
    assign nav_faults=0,nav_x=0,nav_y=0,nav_theta=0,nav_v=0,nav_w=0;
    assign nav_left=0,nav_right=0,nav_index=0,nav_front=16'hffff;
    assign nav_blocked=0,nav_rejected=0,nav_plans=0;
    nav_status_source #(.CLK_HZ(CLK_HZ)) nav_status(.clk(clk50m),.rst_n(reset_sync),
     .running(nav_running),.mission_done(nav_done),.permit(nav_permit),.waypoint_index(nav_index),.faults(nav_faults),
     .x_q16(nav_x),.y_q16(nav_y),.theta_q16(nav_theta),.v(nav_v),.w(nav_w),.left_target(nav_left),.right_target(nav_right),
     .front_min(nav_front),.plans(nav_plans),.packets(lpackets),.errors(lerrors),.rejected(nav_rejected),.frame(nav_frame),.valid(nav_frame_valid));
    assign legacy_m1_disable=0;
    assign legacy_m1_pwm_disabled=0, legacy_cal_disabled=0;
endmodule
