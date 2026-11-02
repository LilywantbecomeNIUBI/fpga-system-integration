module mc_mecanum_top #(parameter CLK_HZ=50000000,BAUD=115200,CONTROL_HZ=100,IMU_ENABLED=1,IMU_ADDRESS=7'h68,IMU_REVERSE=0,IMU_CAL_LOG2=8,CONTROL_SHIFT=4,
 parameter [3:0] BASE_ENCODER_REVERSE=4'b0101,BASE_MOTOR_REVERSE=4'b0101,
 parameter LATERAL_START_PWM=150,LATERAL_TAPER_COUNTS=160,LATERAL_REVERSE=0,
 parameter TELEMETRY_ETHERNET=0,COMMAND_ETHERNET=0)(
 input wire command_error,input wire command_byte_valid,input wire [7:0] command_byte,
 input wire clk50m,reset_n,uart_rx,input wire [3:0] enc_a,enc_b,
 inout wire imu_scl,imu_sda,
 output wire uart_tx,output wire [3:0] in1,in2,output wire [7:0] led,
 output wire legacy_m1_disable,legacy_m2_disable,
 output wire [5:0] legacy_m1_pwm_disabled,legacy_m2_pwm_disabled,
 output wire [1:0] legacy_cal_disabled,
 input wire auxiliary_valid,input wire [383:0] auxiliary_frame,
 output wire auxiliary_ready,output wire [15:0] auxiliary_drops,
 output wire [383:0] telemetry_frame,output wire telemetry_valid,input wire telemetry_ready
);
 reg [2:0] rst_sync;
 always @(posedge clk50m or negedge reset_n)if(!reset_n)rst_sync<=0;else rst_sync<={rst_sync[1:0],1'b1};
 wire rst=rst_sync[2];
 localparam PROTOCOL_VERSION=CONTROL_SHIFT==0 ? 2 : 3;
 wire [7:0] rx_data;wire rx_valid,rx_error,command_valid;
 wire [175:0] body;wire [31:0] bad,rejected;
 wire armed;wire [15:0] faults;wire [7:0] seq;
 wire [127:0] pos,vel;wire [63:0] pwm,target;
 wire imu_ready,imu_fault,imu_sample,imu_stale,calibrated,yaw_ok;
 wire signed [15:0] gx,gy,gz,gyro_bias;
 wire [31:0] sample_us,now_us;
 wire [7:0] who;
 wire [3:0] imu_error;
 wire [3:0] imu_bus_error;wire [1:0] imu_bus_lines;
 wire [15:0] imu_fault_count;
 wire signed [31:0] yaw,pose_x,pose_y,pose_heading;
 wire [1:0] mode;wire arrived,imu_recalibrate;
 generate if(IMU_ENABLED)begin:imu
  mc_mpu6050_reader #(.CLK_HZ(CLK_HZ),.ADDRESS(IMU_ADDRESS)) reader(.clk(clk50m),.reset_n(rst),.restart(imu_recalibrate),.scl(imu_scl),.sda(imu_sda),
   .ready(imu_ready),.fault(imu_fault),.sample_valid(imu_sample),.gx(gx),.gy(gy),.gz(gz),
   .sample_us(sample_us),.now_us(now_us),.who_am_i(who),.error_code(imu_error),.stale(imu_stale),.last_bus_error(imu_bus_error),.bus_lines(imu_bus_lines),.bus_error_count(imu_fault_count));
  mc_mpu6050_yaw #(.REVERSE(IMU_REVERSE),.CAL_LOG2(IMU_CAL_LOG2)) estimator(.clk(clk50m),.reset_n(rst),.sample_valid(imu_sample),
   // User-selected indefinite sample hold: transport age/errors are diagnostic
   // only after initialization. Do not revoke calibrated yaw on missing data.
   .sensor_ok(imu_ready),.wheels_still(vel==128'b0),.recalibrate(imu_recalibrate),.gx(gx),.gy(gy),.gz(gz),
   .sample_us(sample_us),.calibrated(calibrated),.yaw_q16(yaw),.bias(gyro_bias),.healthy(yaw_ok));
 end else begin:no_imu
  assign imu_scl=1'bz;assign imu_sda=1'bz;
  assign imu_ready=0;assign imu_fault=0;assign imu_sample=0;assign imu_stale=1;assign calibrated=0;assign yaw_ok=0;
  assign gx=0;assign gy=0;assign gz=0;assign gyro_bias=0;assign yaw=0;assign sample_us=0;assign now_us=0;assign who=0;assign imu_error=0;
  assign imu_bus_error=0;assign imu_bus_lines=0;
  assign imu_fault_count=0;
 end endgenerate
 mc_uart_rx_byte #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver(.clk(clk50m),.reset_n(rst),.rx(uart_rx),.data(rx_data),.valid(rx_valid),.framing_error(rx_error));
 // Keep the decoder reset until the network-domain reset has released.
 // A held reset must not be counted as a framing error on every clock.
 mc_mecanum_protocol #(.CLK_HZ(CLK_HZ),.VERSION(PROTOCOL_VERSION)) protocol(.clk(clk50m),.reset_n(COMMAND_ETHERNET ? (rst && !command_error) : rst),.byte_valid(COMMAND_ETHERNET ? command_byte_valid : rx_valid),.framing_error(COMMAND_ETHERNET ? 1'b0 : rx_error),.byte_data(COMMAND_ETHERNET ? command_byte : rx_data),.valid(command_valid),.body(body),.bad_frames(bad));
 // UDP ARM must also be newer than the last accepted session command.
 // A delayed duplicate ARM after STOP cannot re-arm the previous session.
 wire [7:0] arm_delta=body[39:32]-seq;
 wire core_command_valid=command_valid && (!COMMAND_ETHERNET || body[31:24]!=1 || (arm_delta!=0 && arm_delta<128));
 // Front feedback connectors are exchanged: physical left-front is input 1.
 // Physical right-front is input 0, verified by manual wheel feedback.
 wire [3:0] wheel_enc_a={enc_a[3:2],enc_a[0],enc_a[1]};
 wire [3:0] wheel_enc_b={enc_b[3:2],enc_b[0],enc_b[1]};
 // Actual motor wiring has the same front-channel exchange as feedback.
 // Keep all control, masks and telemetry in logical FL/FR/RL/RR order.
 wire [3:0] wheel_in1,wheel_in2;
 assign in1={wheel_in1[3:2],wheel_in1[0],wheel_in1[1]};
 assign in2={wheel_in2[3:2],wheel_in2[0],wheel_in2[1]};
 // Physical calibration: left-front and left-rear feedback are reversed.
 // Host encoder masks are additional corrections; normally leave them zero.
 mc_mecanum_core #(.CLK_HZ(CLK_HZ),.CONTROL_HZ(CONTROL_HZ),.CONTROL_SHIFT(CONTROL_SHIFT),.BASE_ENCODER_REVERSE(BASE_ENCODER_REVERSE),.BASE_MOTOR_REVERSE(BASE_MOTOR_REVERSE),.LATERAL_START_PWM(LATERAL_START_PWM),.LATERAL_TAPER_COUNTS(LATERAL_TAPER_COUNTS),.LATERAL_REVERSE(LATERAL_REVERSE)) core(.clk(clk50m),.reset_n(rst),.enc_a(wheel_enc_a),.enc_b(wheel_enc_b),.command_valid(core_command_valid),.command_body(body),
  .yaw_ok(yaw_ok),.yaw_q16(yaw),.control_mode(mode),.arrived(arrived),.imu_recalibrate(imu_recalibrate),.pose_x(pose_x),.pose_y(pose_y),.pose_heading(pose_heading),
  .in1(wheel_in1),.in2(wheel_in2),.armed(armed),.faults(faults),.last_sequence(seq),.rejected(rejected),.positions(pos),.velocities(vel),.applied(pwm),.targets(target));
 // 20 Hz atomic snapshot: M4, version2, armed, faults16, seq, reserved,
 // position[4] int32, velocity[4] int16, PWM[4] int16, bad16, reject16, zero16, CRC16.
 reg [367:0] snapshot,pose_snapshot;reg capture;integer divider;integer i;
 reg [15:0] imu_bus_count_previous;reg [2:0] imu_fault_context;
 always @(posedge clk50m or negedge rst)begin
  if(!rst)begin imu_bus_count_previous<=0;imu_fault_context<=0;end
  else begin
   imu_bus_count_previous<=imu_fault_count;
   if(imu_fault_count!=imu_bus_count_previous)begin
    imu_fault_context<={(mode!=0),(|pwm),armed};
   end
  end
 end
 always @(posedge clk50m or negedge rst)begin
  if(!rst)begin snapshot<=0;pose_snapshot<=0;capture<=0;divider<=0;end
  else begin
   capture<=0;
   if(divider==CLK_HZ/20-1)begin
    divider<=0;capture<=1;snapshot<=0;
    snapshot[15:0]<=16'h344d;snapshot[23:16]<=PROTOCOL_VERSION;snapshot[31:24]<={7'b0,armed};
    snapshot[47:32]<=faults;snapshot[55:48]<=seq;snapshot[191:64]<=pos;
    for(i=0;i<4;i=i+1)snapshot[192+i*16+:16]<=vel[i*32+:16];
    snapshot[319:256]<=pwm;snapshot[335:320]<=bad[15:0];snapshot[351:336]<=rejected[15:0];
    snapshot[359:352]<=CONTROL_SHIFT;
    pose_snapshot<=0;pose_snapshot[15:0]<=16'h3450;pose_snapshot[23:16]<=PROTOCOL_VERSION;
    pose_snapshot[31:24]<={2'b0,imu_fault,(mode==2),(mode==1),arrived,calibrated,yaw_ok};
    pose_snapshot[47:32]<=faults;pose_snapshot[55:48]<=seq;pose_snapshot[63:56]<={4'b0,imu_error};
    pose_snapshot[95:64]<=pose_x;pose_snapshot[127:96]<=pose_y;pose_snapshot[159:128]<=pose_heading;pose_snapshot[191:160]<=yaw;
    pose_snapshot[207:192]<=gyro_bias;pose_snapshot[223:208]<=gz;pose_snapshot[255:224]<=sample_us;
    pose_snapshot[263:256]<=CONTROL_SHIFT;pose_snapshot[303:272]<=32'd60000;
    pose_snapshot[271:264]<=8'd1; // Diagnostic extension version.
    pose_snapshot[319:304]<=imu_fault_count;
    pose_snapshot[327:320]<={2'b0,imu_bus_lines,imu_bus_error};
    pose_snapshot[335:328]<={4'b0,imu_stale,imu_fault_context};
    pose_snapshot[343:336]<=who;pose_snapshot[351:344]<=IMU_ENABLED ? 8'd1 : 8'd0;
   end else divider<=divider+1;
  end
 end
 wire [383:0] frame;wire frame_valid;
 mc_frame_crc crc(.clk(clk50m),.reset_n(rst),.capture(capture),.body_in(snapshot),.frame(frame),.valid(frame_valid));
 wire [383:0] pose_frame;wire pose_frame_valid;
 mc_frame_crc pose_crc(.clk(clk50m),.reset_n(rst),.capture(capture),.body_in(pose_snapshot),.frame(pose_frame),.valid(pose_frame_valid));
 // Main-project transport owns complete packets and holds them across Ethernet
 // backpressure. The imported motor/pose payload and CRC bytes stay unchanged.
 frame_uart_mux #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.AUX_ENABLED(TELEMETRY_ETHERNET),.ETHERNET(TELEMETRY_ETHERNET)) transmitter(
  .clk(clk50m),.reset_n(rst),.motor_valid(frame_valid),.motor_frame(frame),
  .imu_valid(pose_frame_valid),.imu_frame(pose_frame),
  .aux_valid(TELEMETRY_ETHERNET && auxiliary_valid),.aux_frame(auxiliary_frame),
  .aux_ready(auxiliary_ready),.aux_drops(auxiliary_drops),.tx(uart_tx),.imu_drops(),
  .ethernet_frame(telemetry_frame),.ethernet_valid(telemetry_valid),.ethernet_ready(telemetry_ready));
 assign led={faults[7:4],2'b0,|faults,armed};
 assign legacy_m1_disable=0;assign legacy_m2_disable=0;
 assign legacy_m1_pwm_disabled=0;assign legacy_m2_pwm_disabled=0;assign legacy_cal_disabled=0;
endmodule
