`timescale 1ns/1ps
module tb_pose;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,tick=0,active=1,new_goal=0,yaw_ok=1;
 reg [1:0] mode=2;
 reg signed [31:0] yaw=0;
 reg [127:0] pos=0,vel=0;
 reg signed [15:0] goal_x=100,goal_y=0,angle=0;
 wire signed [15:0] x,y,z;wire valid,arrived,overflow;wire signed [31:0] px,py,heading;
 mecanum_pose_control dut(.clk(clk),.reset_n(reset_n),.tick(tick),.active(active),.new_goal(new_goal),.mode(mode),.yaw_ok(yaw_ok),.yaw_q16(yaw),
 .wheel_positions(pos),.wheel_velocities(vel),.goal_x(goal_x),.goal_y(goal_y),.goal_centideg(angle),.speed_limit(16'd40),.position_tolerance(16'd4),
 .command_x(x),.command_y(y),.command_z(z),.command_valid(valid),.arrived(arrived),.overflow(overflow),.x_q16(px),.y_q16(py),.heading_q16(heading));
 task goal;begin @(negedge clk);new_goal=1;@(negedge clk);new_goal=0;repeat(4)@(negedge clk);end endtask
 task update;begin @(negedge clk);tick=1;@(negedge clk);tick=0;wait(valid);@(negedge clk);end endtask
 task wheels(input integer fl,fr,rl,rr);begin pos={32'(rr),32'(rl),32'(fr),32'(fl)};end endtask
 integer i,fl,fr,rl,rr;
 initial begin
  #1;repeat(3)@(negedge clk);reset_n=1;goal();update();if(x<=0 || y!=0 || z!=0 || arrived)$fatal(1,"position command x=%d y=%d z=%d",x,y,z);
  // Independent kinematic plant: apply emitted body commands to wheel travel.
  fl=0;fr=0;rl=0;rr=0;
  for(i=0;i<220 && !arrived;i=i+1)begin
   fl=fl+x-y-z;fr=fr+x+y+z;rl=rl+x+y-z;rr=rr+x-y+z;
   wheels(fl,fr,rl,rr);update();
  end
  if(!arrived || px<96*65536 || px>104*65536)$fatal(1,"position plant convergence %d %d",px,arrived);
  // Coasting after capture must not cause a correction tail.
  wheels(fl+12,fr+12,rl+12,rr+12);repeat(3)update();
  if(x || y || z || !arrived)$fatal(1,"completed position restarted after coast");
  fl=fl+12;fr=fr+12;rl=rl+12;rr=rr+12;
  // A fresh goal has its own origin; no old accumulated-position jump.
  goal_x=0;goal_y=100;goal();update();if(y<=0 || x!=0 || arrived)$fatal(1,"lateral goal");
  wheels(fl-100,fr+100,rl+100,rr-100);update();repeat(21)update();if(!arrived || py<96*65536)$fatal(1,"lateral odometry");
  // A 90 degree heading rotates subsequent forward displacement into local +Y.
  goal_x=0;goal_y=100;angle=9000;goal();yaw=102944;update();
  fl=fl-100;fr=fr+100;rl=rl+100;rr=rr-100;
  wheels(fl+100,fr+100,rl+100,rr+100);update();
  if(px>2*65536 || px< -2*65536 || py<98*65536 || z!=0)$fatal(1,"rotated frame %d %d %d",px,py,z);
  // Near +/-pi takes the short path, not a full turn.
  mode=2;goal_x=0;goal_y=0;angle=-17900;yaw=0;goal();yaw=204744;update();
  if(x!=0 || z<=0 || z>3)$fatal(1,"shortest angle %d",z);
  yaw_ok=0;repeat(3)@(negedge clk);if(x || y || z)$fatal(1,"sensor invalid output");
  yaw_ok=1;yaw=0;mode=2;goal_x=0;goal_y=0;angle=0;goal();
  wheels(fl+16101,fr+16101,rl+16101,rr+16101);
  @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(70)@(negedge clk);
  if(!overflow || valid || x || y || z)$fatal(1,"odometry workspace overflow");
  $display("PASS pose: position plant convergence, lateral odometry, new origin, 90deg frame rotation, angle wrap and invalid-feedback stop");$finish;
 end
 initial begin #2000000;$fatal(1,"pose timeout");end
endmodule
