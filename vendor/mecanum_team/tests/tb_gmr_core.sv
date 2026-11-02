`timescale 1ns/1ps
module tb_gmr_core;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0;reg [175:0] body=0;reg [3:0] a=0,b=0;
 wire [127:0] pos,vel;wire [63:0] pwm,targets;wire armed;wire [15:0] faults;
 wire signed [31:0] px,py,heading;
 mecanum_core #(.CLK_HZ(100000),.CONTROL_HZ(125),.PWM_HZ(1000),.FILTER_CYCLES(1),.CONTROL_SHIFT(4)) dut(
 .clk(clk),.reset_n(reset_n),.enc_a(a),.enc_b(b),.command_valid(valid),.command_body(body),.yaw_ok(1'b1),.yaw_q16(32'sd0),
 .in1(),.in2(),.armed(armed),.faults(faults),.last_sequence(),.rejected(),.positions(pos),.velocities(vel),.applied(pwm),.targets(targets),
 .control_mode(),.arrived(),.pose_x(px),.pose_y(py),.pose_heading(heading));
 task command(input integer op,s,x);begin
  @(negedge clk);body=0;body[31:24]=op;body[39:32]=s;body[79:64]=x;body[127:112]=256;
  if(op==5 || op==6)begin body[159:144]=s==2 ? 10 : 20;body[175:160]=4;end
  valid=1;@(negedge clk);valid=0;repeat(3)@(negedge clk);
 end endtask
 task phase(input [1:0] p);begin @(negedge clk);{a,b}={{4{p[1]}},{4{p[0]}}};repeat(7)@(negedge clk);end endtask
 task forward_cycle;begin phase(1);phase(3);phase(2);phase(0);end endtask
 initial begin
  #1;repeat(4)@(negedge clk);reset_n=1;repeat(20)@(negedge clk);
  command(3,0,0);command(1,1,0);command(6,2,16000);
  repeat(500)forward_cycle();
  // ~100 raw counts/window must feed the PI as 6 or 7 control counts,
  // so target 10 produces positive output rather than treating it as overspeed.
  if(!armed || faults || $signed(vel[31:0])<80 || $signed(pwm[15:0])<=0 || $signed(pwm[15:0])>10)$fatal(1,"GMR core PI units raw=%d pwm=%d",$signed(vel[31:0]),$signed(pwm[15:0]));
  repeat(1000)@(negedge clk);command(6,3,128);
  repeat(16)forward_cycle();repeat(2000)@(negedge clk);
  if($signed(pos[31:0])!=2064 || px<3*65536 || px>5*65536 || py>65536 || py< -65536)$fatal(1,"GMR position units raw=%d local=%d",$signed(pos[31:0]),px);
  command(0,0,0);
  $display("PASS GMR core integration: raw telemetry retained, PI uses normalized feedback, pose consumes normalized displacement");$finish;
 end
 initial begin #1000000;$fatal(1,"GMR core timeout");end
endmodule
