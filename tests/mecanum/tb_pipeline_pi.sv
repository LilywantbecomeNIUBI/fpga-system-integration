`timescale 1ns/1ps
module tb_pipeline_pi;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,enable=1,reverse=0,stopped=1;
 reg signed [15:0] target=0;reg signed [31:0] velocity=0;
 reg [15:0] kp=0,ki=0,ff=0;wire signed [15:0] actual,expected;
 mc_wheel_control_1khz a(.clk(clk),.reset_n(rst),.tick(tick),.enable(enable),.motor_reverse(reverse),.feedback_stopped(stopped),
 .target(target),.velocity_q8(velocity),.kp(kp),.ki(ki),.feedforward(ff),.applied(actual));
 reference_fast_pi b(.clk(clk),.reset_n(rst),.tick(tick),.enable(enable),.motor_reverse(reverse),.feedback_stopped(stopped),
 .target(target),.velocity_q8(velocity),.kp(kp),.ki(ki),.feedforward(ff),.applied(expected));
 integer i,seed=32'h5a230103,checks=0;
 always @(posedge clk)begin #1;
  if(rst)begin
   if(actual!==expected || a.integral!==b.integral || a.raw_output!==b.raw_output)
    $fatal(1,"PI equivalence failed sample=%0d actual=%0d expected=%0d raw=%0d/%0d",i,actual,expected,a.raw_output,b.raw_output);
   checks=checks+1;
  end
 end
 task sample;begin
  @(negedge clk);tick=1;@(negedge clk);tick=0;
  // These ports must already have been captured; change them mid-calculation.
  target=$random(seed);velocity=$random(seed);kp=$random(seed);ki=$random(seed);ff=$random(seed);
  repeat(50)@(negedge clk);
 end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;
  for(i=0;i<4000;i=i+1)begin
   target=$random(seed);velocity=$random(seed);kp=$random(seed);ki=$random(seed);ff=$random(seed);
   case(i%16)
    0:begin target=-32768;velocity=32'h7fffffff;kp=8191;ki=2047;ff=65535;end
    1:begin target=32767;velocity=32'h80000000;kp=8191;ki=2047;ff=65535;end
    2:begin target=0;velocity=0;kp=4096;ki=1024;ff=65535;end
    3:begin target=10;velocity=9*256;kp=512;ki=1;ff=0;end
    4:begin target=-10;velocity=-9*256;kp=512;ki=1;ff=0;end
   endcase
   reverse=i[3];stopped=i[4];sample();
   if(i%101==0)begin enable=0;repeat(2)@(negedge clk);enable=1;end
  end
  tick=1;@(negedge clk);tick=0;repeat(3)@(negedge clk);enable=0;
  repeat(3)@(negedge clk);if(actual!=0)$fatal(1,"mid-pipeline disable failed");
  $display("PASS pipelined PI: 4000 seeded boundary/random transactions, %0d cycle checks, bit-exact raw/integral/PWM and disable",checks);$finish;
 end
 initial begin #4000000;$fatal(1,"PI pipeline timeout");end
endmodule
