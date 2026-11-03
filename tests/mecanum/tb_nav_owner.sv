`timescale 1ns/1ps
module tb_nav_owner;
 reg clk=0;always #5 clk=~clk;reg rst=0,cv=0,active=0,permit=0;
 reg [175:0] body=0;wire armed;wire [3:0] in1,in2;wire [63:0] applied;
 mc_mecanum_core #(.CLK_HZ(1000000),.CONTROL_SHIFT(6),.FAST_SPEED_LOOP(1),.NAVIGATION(1)) dut(
 .clk(clk),.reset_n(rst),.enc_a(4'd0),.enc_b(4'd0),.command_valid(cv),.command_body(body),
 .yaw_ok(1'b1),.yaw_q16(32'sd0),.nav_active(active),.nav_permit(permit),.nav_lease(1'b1),
 .nav_x(16'sd2560),.nav_y(16'sd0),.nav_z(16'sd0),.in1(in1),.in2(in2),.armed(armed),.applied(applied));
 task command(input [7:0] op,seq);begin
  @(negedge clk);body=0;body[31:24]=op;body[39:32]=seq;body[127:112]=2048;body[143:128]=128;
  cv=1;@(negedge clk);cv=0;
 end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;command(3,1);command(1,2);active=1;permit=1;
  repeat(20000)@(negedge clk);if(!armed || applied==0)$fatal(1,"navigation did not drive PI");
  permit=0;#1;if(dut.drive_enable)$fatal(1,"permission did not revoke immediately");
  repeat(3)@(negedge clk);if(applied || in1 || in2)$fatal(1,"revocation retained PWM");
  permit=1;repeat(20000)@(negedge clk);if(applied==0)$fatal(1,"restored permit did not drive");
  active=0;#1;if(dut.drive_enable)$fatal(1,"owner release retained drive between ticks");
  repeat(5)@(negedge clk);if(applied || in1 || in2)$fatal(1,"owner release replayed old PWM");
  active=1;permit=1;repeat(20000)@(negedge clk);command(0,3);
  repeat(3)@(negedge clk);if(armed || applied || in1 || in2)$fatal(1,"STOP lost to nav lease");
  $display("PASS real core navigation permission, between-tick ownership release clears PI/PWM, same-edge STOP beats lease");$finish;
 end
endmodule
