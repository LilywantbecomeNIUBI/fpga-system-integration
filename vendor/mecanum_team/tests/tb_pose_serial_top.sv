`timescale 1ns/1ps
module tb_pose_serial_top;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,rx=1,freeze=0,present=1;reg signed [15:0] gyro=100;
 tri1 scl,sda;wire tx;wire [3:0] in1,in2;
 mecanum_top #(.CLK_HZ(1000000),.BAUD(100000),.IMU_CAL_LOG2(2)) dut(
 .clk50m(clk),.reset_n(reset_n),.uart_rx(rx),.enc_a(4'b0),.enc_b(4'b0),.imu_scl(scl),.imu_sda(sda),.uart_tx(tx),.in1(in1),.in2(in2),
 .led(),.legacy_m1_disable(),.legacy_m2_disable(),.legacy_m1_pwm_disabled(),.legacy_m2_pwm_disabled(),.legacy_cal_disabled());
 mpu6050_model model(.scl(scl),.sda(sda),.present(present),.freeze(freeze),.corrupt(1'b0),.identity(8'h70),.gx(16'sd0),.gy(16'sd0),.gz(gyro));
 wire [7:0] data;wire data_valid,frame_error;
 uart_rx_byte #(.CLK_HZ(1000000),.BAUD(100000)) receiver(.clk(clk),.reset_n(reset_n),.rx(tx),.data(data),.valid(data_valid),.framing_error(frame_error));
 reg [383:0] received=0,pose_frame=0;integer index=0,pose_frames=0;
 function automatic [15:0] crc(input [367:0] body,input integer length);
 reg [15:0] c;integer i,j;
 begin c=16'hffff;for(i=0;i<length;i=i+1)begin c=c^{body[i*8+:8],8'b0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;end crc=c;end
 endfunction
 always @(negedge clk)if(data_valid)begin
  received[index*8+:8]=data;
  if(index==47)begin
   if(received[383:368]!=crc(received[367:0],46))$fatal(1,"pose top telemetry CRC");
   if(received[15:0]==16'h3450)begin pose_frame=received;pose_frames=pose_frames+1;end
   index=0;
  end else index=index+1;
 end
 task send_byte(input [7:0] v);integer i;begin
  @(negedge clk);rx=0;repeat(10)@(negedge clk);
  for(i=0;i<8;i=i+1)begin rx=v[i];repeat(10)@(negedge clk);end
  rx=1;repeat(10)@(negedge clk);
 end endtask
 task command(input integer op,seq,angle);reg [367:0] body;integer i;begin
  body=0;body[15:0]=16'h5aa5;body[23:16]=3;body[31:24]=op;body[39:32]=seq;body[111:96]=angle;body[127:112]=256;
  if(op==6)begin body[159:144]=40;body[175:160]=4;end
  body[191:176]=crc(body,22);
  for(i=0;i<24;i=i+1)send_byte(body[i*8+:8]);repeat(30)@(negedge clk);
 end endtask
 integer seq=3,i;reg signed [31:0] saved_yaw;
 initial begin
  #1;repeat(5)@(negedge clk);reset_n=1;wait(dut.yaw_ok);
  if(dut.gyro_bias!=100 || dut.who!=8'h70)$fatal(1,"MPU6500 full path identity/calibration");
  command(3,0,0);command(1,1,0);command(6,2,1000);
  if(dut.mode!=2 || !dut.armed)$fatal(1,"serial turn not accepted");
  gyro=1410; // independent physical feedback: +10 deg/s, not a commanded angle.
  for(i=0;i<20;i=i+1)begin command(7,seq,0);seq=seq+1;repeat(47500)@(negedge clk);end
  gyro=100;
  for(i=0;i<8;i=i+1)begin command(7,seq,0);seq=seq+1;repeat(47500)@(negedge clk);end
  if(!dut.arrived || dut.faults || !dut.armed || dut.pose_heading<10000 || dut.pose_heading>12500)$fatal(1,"serial turn/arrival yaw=%d flags=%h",dut.pose_heading,dut.faults);
  if(pose_frames<3 || !pose_frame[24] || !pose_frame[26] || pose_frame[343:336]!=8'h70)$fatal(1,"pose telemetry identity/healthy/arrival flags");
  command(8,seq,0);seq=seq+1;
  if(!dut.yaw_ok || !dut.armed)$fatal(1,"accepted calibration while armed");
  command(0,seq,0);seq=seq+1;
  command(8,seq,0);seq=seq+1;
  if(dut.calibrated || dut.yaw!=0 || dut.armed || in1 || in2)$fatal(1,"manual calibration reset/stop");
  wait(dut.yaw_ok);if(dut.gyro_bias!=100 || dut.yaw!=0)$fatal(1,"manual calibration result");
  command(8,seq-1,0);if(!dut.calibrated)$fatal(1,"replayed calibration reset yaw");
  command(1,1,0);command(6,2,1000);seq=3;
  freeze=1;repeat(5000)@(negedge clk);saved_yaw=dut.yaw;
  repeat(45000)@(negedge clk);
  if(!dut.armed || !dut.yaw_ok || dut.faults[8] || dut.yaw!=saved_yaw)$fatal(1,"no-data hold revoked yaw or integrated missing samples");
  freeze=0;present=0;repeat(40000)@(negedge clk);
  if(!dut.armed || !dut.yaw_ok || dut.faults[8] || !dut.imu_fault || dut.imu_fault_count==0)$fatal(1,"NACK must retain calibrated yaw and record communication error");
  command(0,seq,0);seq=seq+1;command(8,seq,0);
  wait(dut.imu_fault);present=1;
  wait(dut.yaw_ok);
  if(dut.armed || in1 || in2 || dut.who!=8'h70)$fatal(1,"manual reconnect resumed motion or failed WHO");
  $display("PASS full pose serial top: MPU6500 calibration, UART turn, indefinite stale/NACK sample hold, error diagnostics and manual communication recovery without auto-ARM");$finish;
 end
 initial begin #40000000;$fatal(1,"full pose serial timeout");end
endmodule
