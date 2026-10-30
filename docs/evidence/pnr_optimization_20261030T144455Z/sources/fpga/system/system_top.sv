module system_top #(
    parameter integer CLK_HZ=50000000, BAUD=115200, PWM_LIMIT=350
)(
    input wire clk50m,reset_n,uart_rx,
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
    wire [383:0] imu_frame;wire imu_valid;wire [15:0] imu_drops;
    // Only the auxiliary framed telemetry crosses this boundary. IMU/perception
    // health never bypasses manual ARM, STOP, command lease or wheel watchdog.
    brushed_chassis #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.PWM_LIMIT(PWM_LIMIT)) chassis(
        .clk50m(clk50m),.reset_n(reset_n),.uart_rx(uart_rx),.uart_tx(uart_tx),
        .left_enc_a(left_enc_a),.left_enc_b(left_enc_b),.right_enc_a(right_enc_a),.right_enc_b(right_enc_b),
        .left_in1(left_in1),.left_in2(left_in2),.right_in1(right_in1),.right_in2(right_in2),
        .auxiliary_frame(imu_frame),.auxiliary_valid(imu_valid),.auxiliary_drops(imu_drops),.led(led));
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
    imu_status_source #(.CLK_HZ(CLK_HZ),.DECIMATION(20)) imu_status(.clk(clk50m),.reset_n(reset_sync),.sample_valid(imu_sample_valid),
        .ready(imu_ready),.fault(imu_fault),.sample_stale(imu_stale),.temperature_valid(imu_temperature_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temperature_raw),.sample_seq(sample_seq),
        .timestamp_us(timestamp_us),.now_us(now_us),.who_am_i(who_am_i),.last_error(last_error),.error_count(error_count),
        .invalid_count(invalid_count),.observe_to_commit_us(observe_to_commit_us),.sample_interval_us(sample_interval_us),
        .drop_count(imu_drops),.frame(imu_frame),.valid(imu_valid));
    assign legacy_m1_disable=0, legacy_m2_disable=0;
    assign legacy_m1_pwm_disabled=0, legacy_m2_pwm_disabled=0, legacy_cal_disabled=0;
endmodule
