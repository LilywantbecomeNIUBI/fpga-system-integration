`timescale 1ns/1ps
module tb_mc_nav_fractional;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,enable=1;reg signed [15:0] target=64;wire signed [15:0] pwm;
 mc_wheel_control_1khz #(.FRACTIONAL_TARGET(1),.REVERSE_TICKS(2)) control(.clk(clk),.reset_n(rst),.tick(tick),.enable(enable),.motor_reverse(1'b0),
 .feedback_stopped(1'b1),.target(target),.target_fractional(1'b1),.velocity_q8(32'sd0),.kp(16'd4096),.ki(16'd0),.feedforward(16'd0),.applied(pwm));
 integer i;
 task step;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(60)@(negedge clk);end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;for(i=0;i<20;i=i+1)step;
  if(pwm<=0 || pwm>5)$fatal(1,"sub-count positive target lost or rescaled: %d",pwm);
  target=-64;for(i=0;i<30;i=i+1)step;
  if(pwm>=0 || pwm< -5)$fatal(1,"signed sub-count negative target lost: %d",pwm);
  enable=0;@(negedge clk);@(negedge clk);if(pwm!=0)$fatal(1,"fractional STOP failed");
  $display("PASS Q8 sub-count navigation targets, sign and immediate disable; no 25mm/s quantization deadlock");$finish;
 end
endmodule
