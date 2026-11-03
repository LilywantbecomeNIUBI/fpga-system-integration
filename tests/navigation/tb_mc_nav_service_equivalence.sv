`timescale 1ns/1ps
// Latency-aware comparison against the immutable pre-serialization service.
module tb_mc_nav_service_equivalence;
 reg clk=0;always #10 clk=~clk;reg rst=0;
 mc_nav_system n(.clk(clk),.rst_n(rst),.command_valid(1'b0),.command_body(176'd0),.armed(1'b0),
 .positions(128'd0),.velocities(128'd0),.yaw_ok(1'b1),.imu_stale(1'b0),.imu_fault(1'b0),.yaw_q16(32'sd0),
 .imu_now_us(32'd0),.imu_sample_us(32'd0),.point_valid(1'b0),.point_angle(16'd0),.point_distance(16'd0),
 .point_intensity(8'd0),.point_errors(32'd0),.flow_valid(1'b0),.flow_snapshot(64'd0),.frame_ready(1'b1));
 mc_nav_system_pre_serial b(.clk(clk),.rst_n(rst),.command_valid(1'b0),.command_body(176'd0),.armed(1'b0),
 .positions(128'd0),.velocities(128'd0),.yaw_ok(1'b1),.imu_stale(1'b0),.imu_fault(1'b0),.yaw_q16(32'sd0),
 .imu_now_us(32'd0),.imu_sample_us(32'd0),.point_valid(1'b0),.point_angle(16'd0),.point_distance(16'd0),
 .point_intensity(8'd0),.point_errors(32'd0),.flow_valid(1'b0),.flow_snapshot(64'd0),.frame_ready(1'b1));
 reg signed [31:0] px,py,mx,my,tx,ty;reg signed [15:0] vx,vy;reg [15:0] lever;
 reg signed [31:0] cc,ss,omega;reg signed [63:0] expected0,expected1,expected2,expected3,injected_angle;
 reg [31:0] seed=32'h92758abc;integer i,angle_value,groups=0,sectors=0,waited;
 function [31:0] random_word;begin seed=seed^(seed<<13);seed=seed^(seed>>17);seed=seed^(seed<<5);random_word=seed;end endfunction
 task tick;begin @(negedge clk);end endtask
 task phase(input [5:0] first,last);begin
  n.state=first;b.state=first;waited=0;
  while(b.state!=last)begin tick;waited=waited+1;if(waited>20)$fatal(1,"reference phase timeout");end
  expected0=b.p0;expected1=b.p1;expected2=b.p2;expected3=b.p3;b.state=0;
  while(n.state!=last)begin tick;waited=waited+1;if(waited>32)$fatal(1,"serialized phase tag/deadlock");end
  if(n.p0!==expected0 || n.p1!==expected1 || n.p2!==expected2 || n.p3!==expected3)
   $fatal(1,"service arithmetic differs group=%d phase=%d p0=%d/%d p1=%d/%d",groups,first,n.p0,expected0,n.p1,expected1);
  n.state=0;groups=groups+1;
 end endtask
 task sector(input integer a);integer expected;begin
  while(n.math_busy || n.math_done)tick;
  injected_angle=a;force n.math_result=injected_angle;force n.math_done=1;n.state=50;tick;
  release n.math_done;release n.math_result;waited=0;
  while(n.state!=15)begin tick;waited=waited+1;if(waited>100)$fatal(1,"sector divider timeout");end
  expected=(a<0?a+36000:a)/500;
  if(n.target_probe_index!==expected[6:0] || $signed(n.desired_heading)!=a)$fatal(1,"sector mismatch angle=%d got=%d expected=%d",a,n.target_probe_index,expected);
  n.state=0;sectors=sectors+1;tick;
 end endtask
 initial begin
  repeat(4)tick;rst=1;tick;
  force n.plan_tick=0;force b.plan_tick=0;force n.running=1;force b.running=1;
  force n.map_fresh=1;force b.map_fresh=1;force n.imu_fresh=1;force b.imu_fresh=1;
  force n.odom_overflow=0;force b.odom_overflow=0;force n.map_epoch=1;force b.map_epoch=1;
  n.plan_epoch=1;b.plan_epoch=1;
  for(i=0;i<256;i=i+1)begin
   px=($signed(random_word())%16001)<<<16;py=($signed(random_word())%16001)<<<16;
   mx=$signed(random_word())%4001;my=$signed(random_word())%4001;tx=$signed(random_word())%4001;ty=$signed(random_word())%4001;
   vx=$signed(random_word())%151;vy=$signed(random_word())%151;lever=50+random_word()%751;
   cc=$signed(random_word())%65537;ss=$signed(random_word())%65537;omega=$signed(random_word())%22877;
   force n.px_q16=px;force b.px_q16=px;force n.py_q16=py;force b.py_q16=py;
   force n.map_x=mx;force b.map_x=mx;force n.map_y=my;force b.map_y=my;
   force n.target_x=tx;force b.target_x=tx;force n.target_y=ty;force b.target_y=ty;
   force n.actual_x=vx;force b.actual_x=vx;force n.actual_y=vy;force b.actual_y=vy;
   force n.planned_x=vx;force b.planned_x=vx;force n.planned_y=vy;force b.planned_y=vy;
   force n.rotation_lever=lever;force b.rotation_lever=lever;
   n.cs=cc;b.cs=cc;n.sn=ss;b.sn=ss;n.body_cs=cc;b.body_cs=cc;n.body_sn=ss;b.body_sn=ss;n.omega=omega;b.omega=omega;
   n.local_x=mx;b.local_x=mx;n.local_y=my;b.local_y=my;n.local_tx=tx;b.local_tx=tx;n.local_ty=ty;b.local_ty=ty;
   phase(3,7);phase(8,12);phase(19,23);phase(24,25);phase(30,34);phase(45,49);
  end
  for(angle_value=-18000;angle_value<=18000;angle_value=angle_value+500)begin
   sector(angle_value);if(angle_value>-18000)sector(angle_value-1);if(angle_value<18000)sector(angle_value+1);
  end
  n.nav_x=777;n.nav_y=-888;n.nav_z=999;n.state=3;tick;force n.running=0;#1;
  if(n.nav_permit)$fatal(1,"fault authorized an in-flight multiplier result");
  repeat(4)tick;if(n.state!=0 || n.nav_x || n.nav_y || n.nav_z || n.plan_valid)$fatal(1,"fault did not cancel tagged service pipeline");
  $display("PASS shared service multiplier: %d old/new product groups, %d signed sector boundaries, bounded latency and fault cancellation",groups,sectors);$finish;
 end
 initial begin #3000000;$fatal(1,"service equivalence timeout");end
endmodule
