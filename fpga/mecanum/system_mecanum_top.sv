// Four-wheel integration. Differential navigation remains in system_task4_top.
// One IMU owner: the teammate 100 Hz gyro reader, yaw and pose loop.
module system_mecanum_top #(parameter integer CLK_HZ=50000000,BAUD=115200,LIDAR_RAW=0,NAVIGATION=0)(
 input wire clk50m,reset_n,uart_rx,lidar_rx,
 input wire [3:0] enc_a,enc_b,output wire [3:0] in1,in2,
 input wire rgmii_rxc,rgmii_rxctl,input wire [3:0] rgmii_rxd,
 input wire camera_vsync,camera_href,camera_pclk,input wire [7:0] camera_data,
 output wire camera_sclk,camera_rst_n,camera_pwdn,inout wire camera_sdat,
 output wire rgmii_gtxc,rgmii_txen,eth_mdc,eth_rst_n,output wire [3:0] rgmii_txd,
 inout wire eth_mdio,imu_i2c_scl,imu_i2c_sda,output wire uart_tx,
 output wire legacy_m1_disable,legacy_m2_disable,
 output wire [5:0] legacy_m1_pwm_disabled,legacy_m2_pwm_disabled,
 output wire [1:0] legacy_cal_disabled,output wire [7:0] led
);
 (* syn_preserve=1 *) reg reset_meta,reset_sync;
 always @(posedge clk50m or negedge reset_n)
  if(!reset_n)begin reset_meta<=0;reset_sync<=0;end
  else begin reset_meta<=1;reset_sync<=reset_meta;end
 wire [383:0] telemetry_frame,lidar_frame;
 wire telemetry_valid,telemetry_ready,lidar_valid,lidar_ready;
 wire [15:0] auxiliary_drops;
 wire motor_clk,motor_reset_n,motor_valid,motor_ready,command_byte_valid;
 wire [191:0] motor_frame;wire [7:0] command_byte;
 wire command_reset_n;
 wire nav_active,nav_permit,nav_lease,nav_cv,nav_armed,nav_yaw_ok,nav_imu_stale,nav_imu_fault;
 wire signed [15:0] nav_x,nav_y,nav_z;wire [175:0] nav_body;
 wire signed [31:0] nav_yaw;wire [31:0] nav_now_us,nav_sample_us;
 wire [127:0] nav_positions,nav_velocities;
 wire [383:0] nav_frame;wire nav_frame_valid,nav_frame_ready,chassis_aux_ready;
 wire flow_safety_valid;wire [63:0] flow_safety_snapshot;
 assign nav_frame_ready=NAVIGATION && nav_frame_valid && chassis_aux_ready;
 assign lidar_ready=chassis_aux_ready && !(NAVIGATION && nav_frame_valid);
 reset_sync command_reset(.clk(clk50m),.arst_n(reset_n && motor_reset_n),.rst_n(command_reset_n));
 mc_command_cdc command_bridge(.src_clk(motor_clk),.src_reset_n(motor_reset_n),
  .src_valid(motor_valid),.src_ready(motor_ready),.src_frame(motor_frame),
  .dst_clk(clk50m),.dst_reset_n(command_reset_n),.byte_valid(command_byte_valid),.byte_data(command_byte));
 mc_mecanum_top #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.TELEMETRY_ETHERNET(1),.COMMAND_ETHERNET(1),.FAST_SPEED_LOOP(1),.CONTROL_SHIFT(6),.LATERAL_TAPER_COUNTS(40),.NAVIGATION(NAVIGATION)) chassis(
  .command_error(!command_reset_n),.command_byte_valid(command_byte_valid),.command_byte(command_byte),
  .clk50m(clk50m),.reset_n(reset_n),.uart_rx(uart_rx),.uart_tx(uart_tx),
  .enc_a(enc_a),.enc_b(enc_b),.in1(in1),.in2(in2),.led(led),
  .imu_scl(imu_i2c_scl),.imu_sda(imu_i2c_sda),
  .legacy_m1_disable(legacy_m1_disable),.legacy_m2_disable(legacy_m2_disable),
  .legacy_m1_pwm_disabled(legacy_m1_pwm_disabled),.legacy_m2_pwm_disabled(legacy_m2_pwm_disabled),
  .legacy_cal_disabled(legacy_cal_disabled),
  .nav_active(nav_active),.nav_permit(nav_permit),.nav_lease(nav_lease),.nav_x(nav_x),.nav_y(nav_y),.nav_z(nav_z),
  .nav_command_valid(nav_cv),.nav_command_body(nav_body),.nav_armed(nav_armed),.nav_yaw_ok(nav_yaw_ok),
  .nav_imu_stale(nav_imu_stale),.nav_imu_fault(nav_imu_fault),.nav_yaw(nav_yaw),.nav_now_us(nav_now_us),.nav_sample_us(nav_sample_us),
  .nav_positions(nav_positions),.nav_velocities(nav_velocities),
  .auxiliary_valid((NAVIGATION && nav_frame_valid && chassis_aux_ready) || (lidar_valid && lidar_ready)),
  .auxiliary_frame(NAVIGATION && nav_frame_valid?nav_frame:lidar_frame),
  .auxiliary_ready(chassis_aux_ready),.auxiliary_drops(auxiliary_drops),
  .telemetry_frame(telemetry_frame),.telemetry_valid(telemetry_valid),.telemetry_ready(telemetry_ready));
    wire [31:0] network_debug;
    wire [7:0] camera_led;
    camera_perception #(.TELEMETRY_ENABLE(1),.MOTOR_ENABLE(1)) camera(.clk50m(clk50m),.reset_n(reset_n),.rgmii_rxc(rgmii_rxc),.rgmii_rxd(rgmii_rxd),.rgmii_rxctl(rgmii_rxctl),
        .camera_sclk(camera_sclk),.camera_sdat(camera_sdat),.camera_vsync(camera_vsync),.camera_href(camera_href),.camera_pclk(camera_pclk),.camera_data(camera_data),
        .camera_rst_n(camera_rst_n),.camera_pwdn(camera_pwdn),.rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen),
        .eth_mdc(eth_mdc),.eth_mdio(eth_mdio),.eth_rst_n(eth_rst_n),.led(camera_led),.network_debug(network_debug),
        .m1_drv_en(),.m2_drv_en(),.lidar_moto_pwm(),
        .telemetry_clk(clk50m),.telemetry_frame(telemetry_frame),.telemetry_valid(telemetry_valid),.telemetry_ready(telemetry_ready),
        .motor_clk(motor_clk),.motor_reset_n(motor_reset_n),.motor_valid(motor_valid),.motor_ready(motor_ready),.motor_frame(motor_frame),
        .flow_safety_valid(flow_safety_valid),.flow_safety_snapshot(flow_safety_snapshot));
    wire lbv,lerr,lpv;wire [7:0] lbyte,lint;wire [15:0] lang,ldist;wire [31:0] lpackets,lerrors,lcrc,lformat,ltimeouts;
    wire [3:0] lindex;
    generate if(NAVIGATION)begin:navigation_enabled
     mc_nav_system #(.CLK_HZ(CLK_HZ)) navigation(.clk(clk50m),.rst_n(command_reset_n),.command_valid(nav_cv),.command_body(nav_body),
      .armed(nav_armed),.positions(nav_positions),.velocities(nav_velocities),.yaw_ok(nav_yaw_ok),.imu_stale(nav_imu_stale),.imu_fault(nav_imu_fault),
      .yaw_q16(nav_yaw),.imu_now_us(nav_now_us),.imu_sample_us(nav_sample_us),.point_valid(lpv),.point_angle(lang),.point_distance(ldist),
      .point_intensity(lint),.point_errors(lerrors),.flow_valid(flow_safety_valid),.flow_snapshot(flow_safety_snapshot),
      .nav_active(nav_active),.nav_permit(nav_permit),.nav_lease(nav_lease),.nav_x(nav_x),.nav_y(nav_y),.nav_z(nav_z),
      .frame_valid(nav_frame_valid),.frame(nav_frame),.frame_ready(nav_frame_ready));
    end else begin:navigation_disabled
     assign nav_active=0,nav_permit=0,nav_lease=0,nav_x=0,nav_y=0,nav_z=0,nav_frame_valid=0,nav_frame=0;
    end endgenerate
    stl27l_uart_rx #(.CLKS_PER_BIT((CLK_HZ+460800)/921600)) lidar_uart(
     .clk(clk50m),.rst_n(reset_sync),.rx(lidar_rx),.byte_valid(lbv),.byte_data(lbyte),.framing_error(lerr));
    stl27l_node_parser #(.BYTE_TIMEOUT_CYCLES(CLK_HZ/500)) lidar_parser(
     .clk(clk50m),.rst_n(reset_sync),.byte_valid(lbv),.byte_in(lbyte),.stream_error(lerr),
     .point_valid(lpv),.point_index(lindex),.distance_mm(ldist),.intensity(lint),.angle_q100(lang),.packet_count(lpackets),.error_count(lerrors),
     .crc_error_count(lcrc),.format_error_count(lformat),.timeout_count(ltimeouts));
    generate if(LIDAR_RAW)begin:ethernet_raw
    lidar_points_source #(.CLK_HZ(CLK_HZ)) lidar_display(
     .clk(clk50m),.rst_n(reset_sync),.byte_valid(lbv),.framing_error(lerr),.point_valid(lpv),.point_index(lindex),
     .angle(lang),.distance(ldist),.intensity(lint),.packets(lpackets),.errors(lerrors),
     .crc_errors(lcrc),.format_errors(lformat),.timeouts(ltimeouts),
     .frame(lidar_frame),.valid(lidar_valid),.ready(lidar_ready));
    end else begin:ethernet_scan
    lidar_scan_360_source #(.CLK_HZ(CLK_HZ)) lidar_display(
     .clk(clk50m),.rst_n(reset_sync),.byte_valid(lbv),.framing_error(lerr),.point_valid(lpv),
     .angle(lang),.distance(ldist),.intensity(lint),.packets(lpackets),.errors(lerrors),
     .crc_errors(lcrc),.format_errors(lformat),.timeouts(ltimeouts),
     .frame(lidar_frame),.valid(lidar_valid),.ready(lidar_ready));
    end endgenerate
endmodule
