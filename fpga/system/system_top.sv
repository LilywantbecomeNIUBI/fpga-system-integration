module system_top #(
    parameter integer CLK_HZ=50000000, BAUD=115200
)(
    input wire clk50m,reset_n,estop_n,uart_rx,
    input wire rgmii_rxc,rgmii_rxctl,input wire [3:0] rgmii_rxd,
    input wire camera_vsync,camera_href,camera_pclk,input wire [7:0] camera_data,
    output wire camera_sclk,camera_rst_n,camera_pwdn,inout wire camera_sdat,
    output wire rgmii_gtxc,rgmii_txen,eth_mdc,eth_rst_n,output wire [3:0] rgmii_txd,
    inout wire eth_mdio,imu_i2c_scl,imu_i2c_sda,
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
    wire [383:0] motor_frame,imu_frame;wire motor_valid,imu_valid;wire [15:0] imu_drops;
    motor_status_source #(.CLK_HZ(CLK_HZ)) telemetry(
        .clk(clk50m),.reset_n(reset_sync),.armed(armed),.flags(flags),.last_sequence(last_sequence),.faults(faults),
        .left_position(left_position),.right_position(right_position),.left_velocity(left_velocity),.right_velocity(right_velocity),
        .left_applied(left_applied),.right_applied(right_applied),.left_invalid(left_invalid),.right_invalid(right_invalid),
        .bad_frames(bad_frames),.rejected_commands(rejected_commands),.frame(motor_frame),.valid(motor_valid));
    wire [31:0] network_debug;
    wire [7:0] camera_led;
    camera_perception camera(.clk50m(clk50m),.reset_n(reset_n),.rgmii_rxc(rgmii_rxc),.rgmii_rxd(rgmii_rxd),.rgmii_rxctl(rgmii_rxctl),
        .camera_sclk(camera_sclk),.camera_sdat(camera_sdat),.camera_vsync(camera_vsync),.camera_href(camera_href),.camera_pclk(camera_pclk),.camera_data(camera_data),
        .camera_rst_n(camera_rst_n),.camera_pwdn(camera_pwdn),.rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen),
        .eth_mdc(eth_mdc),.eth_mdio(eth_mdio),.eth_rst_n(eth_rst_n),.led(camera_led),.network_debug(network_debug),
        .m1_drv_en(),.m2_drv_en(),.lidar_moto_pwm());
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
    imu_status_source #(.CLK_HZ(CLK_HZ)) imu_status(.clk(clk50m),.reset_n(reset_sync),.sample_valid(imu_sample_valid),
        .ready(imu_ready),.fault(imu_fault),.sample_stale(imu_stale),.temperature_valid(imu_temperature_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temperature_raw),.sample_seq(sample_seq),
        .timestamp_us(timestamp_us),.now_us(now_us),.who_am_i(who_am_i),.last_error(last_error),.error_count(error_count),
        .invalid_count(invalid_count),.observe_to_commit_us(observe_to_commit_us),.sample_interval_us(sample_interval_us),
        .drop_count(imu_drops),.frame(imu_frame),.valid(imu_valid));
    frame_uart_mux #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) sender(.clk(clk50m),.reset_n(reset_sync),
        .motor_valid(motor_valid),.motor_frame(motor_frame),.imu_valid(imu_valid),.imu_frame(imu_frame),.tx(uart_tx),.imu_drops(imu_drops));
    assign legacy_m1_disable=0, legacy_m2_disable=0;
    assign legacy_m1_pwm_disabled=0, legacy_m2_pwm_disabled=0, legacy_cal_disabled=0;
    assign led={faults[4:0],flags[0],armed,reset_sync};
endmodule
