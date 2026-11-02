`timescale 1ns/1ps
module tb_arbitrary_direction;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,tick=0,new_goal=0;
 reg signed [15:0] gx=0,gy=0;
 reg [127:0] pos=0;
 wire signed [15:0] x,y,z,fl,fr,rl,rr;
 wire valid,arrived;wire signed [31:0] px,py;
 mecanum_pose_control #(.LATERAL_REVERSE(0)) dut(.clk(clk),.reset_n(reset_n),.tick(tick),.active(1'b1),.new_goal(new_goal),
 .mode(2'd2),.yaw_ok(1'b1),.yaw_q16(32'd0),.wheel_positions(pos),.wheel_velocities(128'd0),
 .goal_x(gx),.goal_y(gy),.goal_centideg(16'd0),.speed_limit(16'd20),.position_tolerance(16'd4),
 .command_x(x),.command_y(y),.command_z(z),.command_valid(valid),.arrived(arrived),.x_q16(px),.y_q16(py));
 // Match deployed mixer boundary: fixed whole-chassis lateral calibration.
 wire signed [15:0] physical_y=y;
 mecanum_mixer mix(.x(x),.y(physical_y),.z(z),.fl(fl),.fr(fr),.rl(rl),.rr(rr));
 task update;
 begin @(negedge clk);tick=1;@(negedge clk);tick=0;wait(valid);@(negedge clk);end
 endtask
 integer p0=0,p1=0,p2=0,p3=0,i;
 task travel(input signed [15:0] tx,ty);
 begin
  @(negedge clk);gx=tx;gy=ty;new_goal=1;@(negedge clk);new_goal=0;update();
  if(arrived)$fatal(1,"previous completion leaked");
  for(i=0;i<600 && !arrived;i=i+1)begin
   if(z!=0)$fatal(1,"translation introduced yaw command");
   p0=p0+fl;p1=p1+fr;p2=p2+rl;p3=p3+rr;
   pos={32'(p3),32'(p2),32'(p1),32'(p0)};update();
  end
  if(!arrived || x || y || z || px<(tx-4)*65536 || px>(tx+4)*65536 || py<(ty-4)*65536 || py>(ty+4)*65536)
   $fatal(1,"oblique target did not converge goal=%d,%d actual=%d,%d",tx,ty,px,py);
 end
 endtask
 initial begin
  repeat(4)@(negedge clk);reset_n=1;
  travel(240,139);travel(139,240);travel(-196,196);travel(-240,-139);travel(139,-240);travel(196,196);
  $display("PASS arbitrary translation: six oblique directions, calibrated wheel mixing/odometry, fresh origin, heading hold and arrival stop");$finish;
 end
 initial begin #10000000;$fatal(1,"arbitrary translation timeout");end
endmodule
