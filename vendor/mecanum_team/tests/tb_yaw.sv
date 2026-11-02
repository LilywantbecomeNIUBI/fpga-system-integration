`timescale 1ns/1ps
module tb_yaw;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0,ok=1,still=1,recalibrate=0;
 reg signed [15:0] gz=100;
 reg [31:0] stamp=0;
 wire calibrated,healthy;wire signed [31:0] yaw;wire signed [15:0] bias;
 mpu6050_yaw #(.CAL_LOG2(2)) dut(.clk(clk),.reset_n(reset_n),.sample_valid(valid),.sensor_ok(ok),.wheels_still(still),.recalibrate(recalibrate),
 .gx(16'sd0),.gy(16'sd0),.gz(gz),.sample_us(stamp),.calibrated(calibrated),.yaw_q16(yaw),.bias(bias),.healthy(healthy));
 task sample(input integer dt);begin @(negedge clk);stamp=stamp+dt;valid=1;@(negedge clk);valid=0;repeat(10)@(negedge clk);end endtask
 integer i;
 initial begin
  #1;repeat(3)@(negedge clk);reset_n=1;still=0;repeat(8)sample(10000);if(calibrated)$fatal(1,"calibrated moving");
  still=1;repeat(4)sample(10000);if(!healthy || bias!=100 || yaw!=0)$fatal(1,"bias calibration");
  repeat(30)sample(10000);if(yaw!=0)$fatal(1,"bias drift");
  // 10 degrees/s for one second, after subtracting bias.
  gz=1410;repeat(100)sample(10000);if(yaw<11430 || yaw>11446)$fatal(1,"yaw integration %d",yaw);
  // 200 degrees additional => normalized -150 degrees.
  gz=26300;repeat(100)sample(10000);if(yaw< -171650 || yaw> -171500)$fatal(1,"angle wrap %d",yaw);
  recalibrate=1;repeat(2)@(negedge clk);recalibrate=0;
  if(healthy || calibrated || yaw || bias)$fatal(1,"manual zero did not reset state");
  gz=-100;still=0;repeat(4)sample(10000);if(calibrated)$fatal(1,"manual calibration while moving");
  still=1;repeat(4)sample(10000);if(!healthy || bias!=-100 || yaw)$fatal(1,"manual new bias/zero");
  ok=0;@(negedge clk);if(healthy || calibrated)$fatal(1,"health drop");
  ok=1;gz=100;repeat(4)sample(10000);if(!healthy)$fatal(1,"recalibration");
  sample(30000000);if(!calibrated || yaw!=0)$fatal(1,"long sample gap must hold calibrated angle");
  gz=32767;sample(10000);if(!healthy || yaw!=0 || bias!=100)$fatal(1,"saturated sample erased calibrated state");
  gz=1410;sample(2000);if(!healthy || yaw!=0)$fatal(1,"short-interval sample erased calibrated state");
  sample(10000);if(!healthy || yaw<110 || yaw>118)$fatal(1,"fresh sample did not resume integration after skipped sample");
  $display("PASS yaw: stationary gating, bias subtraction, 10/200dps integration, +/-pi wrap, stale and recalibration");$finish;
 end
 initial begin #1000000;$fatal(1,"yaw timeout");end
endmodule
