`timescale 1ns/1ps
module tb_chassis_pwm;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0;reg [175:0] body=0;
 wire [63:0] pwm;wire armed;wire [15:0] faults;
 mecanum_core #(.CLK_HZ(1000000),.CONTROL_HZ(1000),.BASE_MOTOR_REVERSE(4'b0101)) dut(
 .clk(clk),.reset_n(reset_n),.enc_a(4'b0),.enc_b(4'b0),.command_valid(valid),.command_body(body),
 .yaw_ok(1'b1),.yaw_q16(32'd0),.armed(armed),.faults(faults),.applied(pwm));
 task command(input [7:0] op,seq,input [15:0] speed);
 begin
  @(negedge clk);body=0;body[31:24]=op;body[39:32]=seq;
  body[79:64]=speed*64;if(op==6)begin body[159:144]=40;body[175:160]=4;end body[127:112]=256;valid=1;
  @(negedge clk);valid=0;
 end endtask
 initial begin
  repeat(10)@(negedge clk);reset_n=1;
  command(3,0,0);command(1,1,0);command(6,2,20);
  // Wait through the reverse interlock, before the no-feedback stall timeout.
  repeat(25000)@(negedge clk);
  if(!armed || faults)$fatal(1,"unexpected stop");
  if($signed(pwm[0+:16])!=-20 || $signed(pwm[16+:16])!=20 ||
     $signed(pwm[32+:16])!=-20 || $signed(pwm[48+:16])!=20)$fatal(1,"built-in FL/RL PWM correction");
  command(0,3,0);repeat(10)@(negedge clk);
  if(armed || pwm)$fatal(1,"STOP");
  $display("PASS chassis PWM: FL/RL reversed, forward targets retained, STOP zeros all wheels");$finish;
 end
endmodule
