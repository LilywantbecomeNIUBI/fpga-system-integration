`timescale 1ns/1ps
module tb_mc_nav_odometry;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,origin=0,active=1;reg [127:0] positions=0;reg signed [31:0] yaw=0;
 wire signed [31:0] x,y,heading;wire signed [15:0] vx,vy;wire valid,overflow;
 mc_nav_odometry odom(.clk(clk),.rst_n(rst),.tick(tick),.origin(origin),.active(active),.positions(positions),.yaw_q16(yaw),
 .forward_scale_q16(32'd65536),.lateral_scale_q16(32'd65536),.x_q16(x),.y_q16(y),.heading_q16(heading),
 .velocity_x_mm_s(vx),.velocity_y_mm_s(vy),.valid(valid),.overflow(overflow));
 task step(input signed [31:0] fl,fr,rl,rr);begin
  @(negedge clk);positions={rr,rl,fr,fl};tick=1;@(negedge clk);tick=0;while(!valid)@(negedge clk);
  if(overflow)$fatal(1,"normal motion overflow");
 end endtask
 task check(input integer ex,ey);begin
  if(x-ex*65536>131072 || ex*65536-x>131072 || y-ey*65536>131072 || ey*65536-y>131072)$fatal(1,"pose mismatch x=%d y=%d expected %d %d",x,y,ex,ey);
 end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;origin=1;@(negedge clk);origin=0;
  step(100,100,100,100);check(100,0);
  step(0,200,200,0);check(100,100);
  yaw=102944;step(100,300,300,100);check(100,200);
  // Pausing a mission does not assert origin or disable continuous integration.
  step(200,400,400,200);check(100,300);
  origin=1;@(negedge clk);origin=0;step(300,500,500,300);check(100,0);
  @(negedge clk);positions={32'sd1300,32'sd1500,32'sd1500,32'sd1300};tick=1;
  @(negedge clk);tick=0;while(!valid)@(negedge clk);
  if(!overflow || vx!=0 || vy!=0)$fatal(1,"scaled velocity silently wrapped before direction gating");
  $display("PASS continuous four-wheel odometry, forward/lateral, 90-degree frame, pause continuity and explicit new origin");$finish;
 end
 initial begin #1000000;$fatal(1,"odometry deadlock");end
endmodule
