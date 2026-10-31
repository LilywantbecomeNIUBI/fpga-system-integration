module system_task4_top #(
    parameter integer CLK_HZ=50000000, BAUD=115200, PWM_LIMIT=350
)(
    input wire clk50m,reset_n,uart_rx,lidar_rx,
    input wire rgmii_rxc,rgmii_rxctl,input wire [3:0] rgmii_rxd,
    input wire camera_vsync,camera_href,camera_pclk,input wire [7:0] camera_data,
    output wire camera_sclk,camera_rst_n,camera_pwdn,inout wire camera_sdat,
    output wire rgmii_gtxc,rgmii_txen,eth_mdc,eth_rst_n,output wire [3:0] rgmii_txd,
    inout wire eth_mdio,imu_i2c_scl,imu_i2c_sda,
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
    reg [383:0] auxiliary_frame,pending_nav;reg auxiliary_valid,nav_pending;
    always @(posedge clk50m or negedge reset_sync)begin
     if(!reset_sync)begin auxiliary_frame<=0;auxiliary_valid<=0;pending_nav<=0;nav_pending<=0;end
     else begin
      auxiliary_valid<=0;
      if(nav_frame_valid)begin pending_nav<=nav_frame;nav_pending<=1;end
      if(imu_valid)begin auxiliary_frame<=imu_frame;auxiliary_valid<=1;end
      else if(nav_pending || nav_frame_valid)begin auxiliary_frame<=nav_frame_valid?nav_frame:pending_nav;auxiliary_valid<=1;nav_pending<=0;end
     end
    end
    // Only the auxiliary framed telemetry crosses this boundary. IMU/perception
    // health never bypasses manual ARM, STOP, command lease or wheel watchdog.
    task4_chassis #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.PWM_LIMIT(PWM_LIMIT)) chassis(
        .clk50m(clk50m),.reset_n(reset_n),.uart_rx(uart_rx),.uart_tx(uart_tx),
        .left_enc_a(left_enc_a),.left_enc_b(left_enc_b),.right_enc_a(right_enc_a),.right_enc_b(right_enc_b),
        .left_in1(left_in1),.left_in2(left_in2),.right_in1(right_in1),.right_in2(right_in2),
        .auxiliary_frame(auxiliary_frame),.auxiliary_valid(auxiliary_valid),.auxiliary_drops(imu_drops),
        .nav_running(nav_running),.nav_permit(nav_permit),.nav_left(nav_left),.nav_right(nav_right),
        .nav_command_valid(nav_command_valid),.nav_opcode(nav_opcode),.nav_sequence(nav_sequence),
        .nav_arg0(nav_a0),.nav_arg1(nav_a1),.nav_arg2(nav_a2),.nav_arg3(nav_a3),
        .nav_left_position(nav_lp),.nav_right_position(nav_rp),.nav_armed(nav_armed),.led(led));
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
    wire lbv,lerr,lpv;wire [7:0] lbyte,lint;wire [15:0] lang,ldist;wire [31:0] lpackets,lerrors;
    stl27l_uart_rx #(.CLKS_PER_BIT((CLK_HZ+460800)/921600)) lidar_uart(
     .clk(clk50m),.rst_n(reset_sync),.rx(lidar_rx),.byte_valid(lbv),.byte_data(lbyte),.framing_error(lerr));
    stl27l_node_parser #(.BYTE_TIMEOUT_CYCLES(CLK_HZ/500)) lidar_parser(
     .clk(clk50m),.rst_n(reset_sync),.byte_valid(lbv),.byte_in(lbyte),.stream_error(lerr),
     .point_valid(lpv),.distance_mm(ldist),.intensity(lint),.angle_q100(lang),.packet_count(lpackets),.error_count(lerrors));
    nav_core #(.CLK_HZ(CLK_HZ)) navigation(.clk(clk50m),.rst_n(reset_sync),
     .left_position(nav_lp),.right_position(nav_rp),.armed(nav_armed),.imu_valid(imu_sample_valid),.imu_bad(imu_fault||imu_stale||!imu_ready),
     .gx(gx),.gy(gy),.gz(gz),.point_valid(lpv),.point_angle(lang),.point_distance(ldist),.point_intensity(lint),
     .command_valid(nav_command_valid),.opcode(nav_opcode),.sequence_id(nav_sequence),.arg0(nav_a0),.arg1(nav_a1),.arg2(nav_a2),.arg3(nav_a3),
     .running(nav_running),.mission_done(nav_done),.permit(nav_permit),.faults(nav_faults),.update(nav_update),
     .left_target(nav_left),.right_target(nav_right),.x_q16(nav_x),.y_q16(nav_y),.theta_q16(nav_theta),
     .v_mm_s(nav_v),.w_mrad_s(nav_w),.waypoint_index(nav_index),.front_min(nav_front),.blocked(nav_blocked),.rejected(nav_rejected),.plan_count(nav_plans));
    nav_status_source #(.CLK_HZ(CLK_HZ)) nav_status(.clk(clk50m),.rst_n(reset_sync),
     .running(nav_running),.mission_done(nav_done),.permit(nav_permit),.waypoint_index(nav_index),.faults(nav_faults),
     .x_q16(nav_x),.y_q16(nav_y),.theta_q16(nav_theta),.v(nav_v),.w(nav_w),.left_target(nav_left),.right_target(nav_right),
     .front_min(nav_front),.plans(nav_plans),.packets(lpackets),.errors(lerrors),.rejected(nav_rejected),.frame(nav_frame),.valid(nav_frame_valid));
    assign legacy_m1_disable=0;
    assign legacy_m1_pwm_disabled=0, legacy_cal_disabled=0;
endmodule
