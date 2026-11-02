`timescale 1ns/1ps
module tb_chassis_feedback;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0;reg [3:0] a=0,b=0;
 mecanum_top #(.CLK_HZ(1000000),.BAUD(100000),.CONTROL_HZ(1000),.IMU_ENABLED(0)) dut(
 .clk50m(clk),.reset_n(reset_n),.uart_rx(1'b1),.enc_a(a),.enc_b(b));
 // Input 1 is physical FL, input 0 is physical FR; rear inputs unchanged.
 task cycle(input integer pin,input integer backwards);
 begin
  if(backwards)a[pin]=1;else b[pin]=1;
  repeat(40)@(negedge clk);a[pin]=1;b[pin]=1;
  repeat(40)@(negedge clk);
  if(backwards)a[pin]=0;else b[pin]=0;
  repeat(40)@(negedge clk);a[pin]=0;b[pin]=0;
  repeat(40)@(negedge clk);
 end endtask
 initial begin
  if(dut.LATERAL_REVERSE!=0)$fatal(1,"deployed lateral polarity reverted to old inverted calibration");
  repeat(10)@(negedge clk);reset_n=1;repeat(100)@(negedge clk);
  cycle(1,1);
  if($signed(dut.pos[0+:32])!=4 || dut.pos[127:32]!=0)$fatal(1,"FL map/polarity");
  cycle(0,0);
  if($signed(dut.pos[32+:32])!=4 || $signed(dut.pos[0+:32])!=4 || dut.pos[127:64]!=0)$fatal(1,"FR map/polarity");
  cycle(2,1);cycle(3,0);
  if(dut.pos!={32'd4,32'd4,32'd4,32'd4})$fatal(1,"rear polarity");
  if(dut.armed || dut.in1 || dut.in2)$fatal(1,"passive check enabled motors");
  $display("PASS chassis physical feedback mapping and built-in direction correction without ARM");$finish;
 end
endmodule
