`timescale 1ns/1ps
module tb_lateral_start_core;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0;reg [175:0] body=0;
 wire armed;wire [15:0] faults;wire [63:0] pwm;
 mecanum_core #(.CLK_HZ(1000000),.CONTROL_HZ(1000),.CONTROL_SHIFT(4),.LATERAL_START_PWM(150)) dut(
 .clk(clk),.reset_n(reset_n),.enc_a(4'b0),.enc_b(4'b0),.command_valid(valid),.command_body(body),
 .yaw_ok(1'b1),.yaw_q16(32'd0),.armed(armed),.faults(faults),.applied(pwm));
 task command(input [7:0] op,seq,input signed [15:0] y);
 begin
  @(negedge clk);body=0;body[31:24]=op;body[39:32]=seq;body[95:80]=y;body[127:112]=256;
  if(op==6)begin body[159:144]=20;body[175:160]=4;end
  valid=1;@(negedge clk);valid=0;
 end endtask
 initial begin
  repeat(5)@(negedge clk);reset_n=1;
  command(3,0,0);command(1,1,0);command(6,2,320);
  repeat(30000)@(negedge clk);
  if(!armed || faults || dut.lateral_compensation!=150)$fatal(1,"lateral mode activation");
  if($signed(pwm[0+:16])!=-155 || $signed(pwm[16+:16])!=155 ||
     $signed(pwm[32+:16])!=155 || $signed(pwm[48+:16])!=-155)$fatal(1,"lateral PWM routing/sign/feedforward");
  command(0,3,0);repeat(10)@(negedge clk);
  if(armed || pwm || dut.lateral_compensation)$fatal(1,"STOP must remove all compensation");
  $display("PASS relative lateral start core: 15 percent compensation plus PI, correct four-wheel signs, ramp/interlock and STOP");$finish;
 end
endmodule
