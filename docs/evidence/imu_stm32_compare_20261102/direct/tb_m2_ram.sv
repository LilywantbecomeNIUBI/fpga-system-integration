`timescale 1ns/1ps
module tb_m2_sensor_diag;
 reg clk=0,reset_n=0,lidar_rx=1,la=1,lb=1,ra=1,rb=1;
 always #10 clk=~clk;
 tri1 scl,sda;
 wire tx,l1,l2,r1,r2,m1;
 wire [5:0] old_pwm;wire [0:0] old_cal;
 m2_sensor_diag_top #(.CLK_HZ(5000000)) dut(.clk50m(clk),.reset_n(reset_n),.lidar_rx(lidar_rx),
  .imu_i2c_scl(scl),.imu_i2c_sda(sda),.left_enc_a(la),.left_enc_b(lb),.right_enc_a(ra),.right_enc_b(rb),
  .uart_tx(tx),.left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2),
  .legacy_m1_disable(m1),.legacy_m1_pwm_disabled(old_pwm),.legacy_cal_disabled(old_cal));
 defparam dut.imu.POWERUP_US=100;
 defparam dut.imu.RESET_US=1200;
 defparam dut.imu.STARTUP_US=500;
 defparam dut.imu_status.DECIMATION=4;
 mpu6050_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(1'b1),.identity(8'h70),
  .corrupt_readback(1'b0),.invalid_data(1'b0),.freeze_samples(1'b0));
 integer samples=0,bytes=0,file,i,k;
 wire [7:0] byte_data;wire byte_valid,framing_error;
 uart_rx_byte #(.CLK_HZ(5000000)) monitor(.clk(clk),.reset_n(reset_n),.rx(tx),.data(byte_data),.valid(byte_valid),.framing_error(framing_error));
 always @(posedge clk)begin
  if({l1,l2,r1,r2,m1,old_pwm,old_cal}!==12'd0)$fatal(1,"Physical motor pads must remain zero");
  if(reset_n && dut.chassis.armed)$fatal(1,"Read-only diagnostic armed its chassis");
  if(dut.imu_sample_valid)samples=samples+1;
  if(byte_valid)begin $fwrite(file,"%c",byte_data);bytes=bytes+1;end
  if(framing_error)$fatal(1,"Mixed UART framing error");
 end
 reg [7:0] lidar_bytes[0:5122];
 initial begin
  $readmemh("build/task4/lidar_display_input.hex",lidar_bytes);
  wait(reset_n);repeat(10)@(negedge clk);
  for(i=0;i<5123;i=i+1)begin
   lidar_rx=0;#100;for(k=0;k<8;k=k+1)begin lidar_rx=lidar_bytes[i][k];#100;end
   lidar_rx=1;#100;
  end
 end
 integer q;
 initial begin
  wait(reset_n);#100000;
  for(q=0;q<8;q=q+1)begin
   {la,lb}=2'b10;{ra,rb}=2'b01;#2000;
   {la,lb}=2'b00;{ra,rb}=2'b00;#2000;
   {la,lb}=2'b01;{ra,rb}=2'b10;#2000;
   {la,lb}=2'b11;{ra,rb}=2'b11;#2000;
  end
 end
 initial begin
  file=$fopen("build/m2_sensor_diag_direct_20261102/sim_uart.bin","wb");
  // Even an erroneous internal actuator signal cannot reach a motor pad.
  force dut.diag_l1=1;force dut.diag_l2=1;force dut.diag_r1=1;force dut.diag_r2=1;
  repeat(50)@(negedge clk);reset_n=1;
  #26000000;
  if(samples<15 || !dut.imu_ready || dut.imu_fault || dut.who_am_i!=8'h70)$fatal(1,"MPU6050 sample/identity failure");
  if(dut.lpackets!=108 || dut.lcrc!=1)$fatal(1,"Real-parser radar fixture failure");
  if(dut.nav_lp==0 || dut.nav_rp==0)$fatal(1,"Missing encoder movement");
  if(bytes<48*10)$fatal(1,"No mixed sensor telemetry");
  $fclose(file);$display("PASS M2 static top: samples=%0d bytes=%0d radar=%0d encL=%0d encR=%0d, all 12 motor pads LOW",samples,bytes,dut.lpackets,dut.nav_lp,dut.nav_rp);$finish;
 end
 initial begin #40000000;$fatal(1,"M2 static test timeout");end
endmodule
