`timescale 1ns/1ps
module tb_outer_core;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0,yaw_ok=0;reg signed [31:0] yaw=0;
 reg [175:0] body=0;
 wire [3:0] in1,in2;wire armed;wire [15:0] faults;wire [7:0] seq;wire [31:0] rejected;
 wire [127:0] pos,vel;wire [63:0] pwm,targets;wire [1:0] mode;wire arrived;wire signed [31:0] px,py,ph;
 mecanum_core #(.CLK_HZ(100000),.CONTROL_HZ(1000),.PWM_HZ(1000),.REVERSE_TICKS(1)) dut(
 .clk(clk),.reset_n(reset_n),.enc_a(4'b0),.enc_b(4'b0),.command_valid(valid),.command_body(body),.yaw_ok(yaw_ok),.yaw_q16(yaw),
 .in1(in1),.in2(in2),.armed(armed),.faults(faults),.last_sequence(seq),.rejected(rejected),.positions(pos),.velocities(vel),.applied(pwm),.targets(targets),
 .control_mode(mode),.arrived(arrived),.pose_x(px),.pose_y(py),.pose_heading(ph));
 task command(input integer op,s,x,y,z,limit);
 begin
  @(negedge clk);body=0;body[31:24]=op;body[39:32]=s;body[79:64]=x;body[95:80]=y;body[111:96]=z;body[127:112]=256;
  if(op==5 || op==6)begin body[159:144]=limit;body[175:160]=4;end
  valid=1;@(negedge clk);valid=0;repeat(3)@(negedge clk);
 end endtask
 task start_session;begin command(0,0,0,0,0,0);command(3,0,0,0,0,0);command(1,1,0,0,0,0);end endtask
 reg [31:0] previous_rejected;integer i;
 initial begin
  #1;repeat(3)@(negedge clk);reset_n=1;start_session();
  command(6,2,100,0,0,40);if(mode!=0 || targets!=0)$fatal(1,"accepted uncalibrated pose");
  yaw_ok=1;command(6,2,100,0,0,40);repeat(300)@(negedge clk);
  if(mode!=2 || seq!=2 || $signed(targets[15:0])<=0)$fatal(1,"pose routing");
  command(7,3,0,0,0,0);if(dut.goal_x!=100 || dut.new_goal)$fatal(1,"heartbeat moved goal");
  // A completion flag belongs to its old goal and must be gated immediately.
  force dut.pose_arrived=1'b1;
  #1;
  if(!arrived)$fatal(1,"completion fixture");
  @(negedge clk);body[31:24]=6;body[39:32]=4;body[79:64]=200;body[159:144]=40;body[175:160]=4;valid=1;
  @(negedge clk);valid=0;if(arrived)$fatal(1,"old completion leaked into new goal");
  release dut.pose_arrived;
  repeat(3)@(negedge clk);
  command(7,5,0,0,0,0);
  previous_rejected=rejected;command(7,5,0,0,0,0);if(rejected!=previous_rejected+1)$fatal(1,"heartbeat replay");
  previous_rejected=rejected;command(6,6,100,0,0,0);if(rejected!=previous_rejected+1 || seq!=5)$fatal(1,"zero speed cap");
  yaw=6554;repeat(300)@(negedge clk);
  if($signed(targets[15:0])<=$signed(targets[31:16]))$fatal(1,"heading correction signs");
  yaw_ok=0;repeat(5)@(negedge clk);if(armed || !faults[8] || in1 || in2)$fatal(1,"IMU loss global stop");
  yaw_ok=1;yaw=0;start_session();command(5,2,20,0,0,40);repeat(300)@(negedge clk);
  if(mode!=0 || targets!=0 || seq!=1)$fatal(1,"removed heading velocity accepted");
  command(0,2,0,0,0,0);if(armed || in1 || in2)$fatal(1,"outer STOP");
  start_session();command(6,2,0,0,9000,40);repeat(300)@(negedge clk);
  if($signed(targets[15:0])>=0 || $signed(targets[31:16])<=0)$fatal(1,"turn signs");
  repeat(25500)@(negedge clk);if(armed || !faults[1] || in1 || in2)$fatal(1,"outer watchdog");
  $display("PASS outer core: calibration gate, pose/turn routing and removed heading rejection, goal-preserving heartbeat/replay, bounds, IMU fault, STOP and watchdog");$finish;
 end
 initial begin #1000000;$fatal(1,"outer core timeout");end
endmodule
