`timescale 1ns/1ps
module tb_fast_pi;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fast_tick=0,slow_tick=0,enable=1;reg signed [15:0] target=20;
 reg signed [31:0] velocity=16;reg [15:0] kp=512,ki=32,ff=0;wire signed [15:0] fast,slow;
 mc_wheel_control_1khz a(.clk(clk),.reset_n(rst),.tick(fast_tick),.enable(enable),.motor_reverse(1'b0),.feedback_stopped(1'b1),
 .target(target),.velocity_q8(velocity*32'sd256),.kp(kp),.ki(ki),.feedforward(ff),.applied(fast));
 mc_wheel_control b(.clk(clk),.reset_n(rst),.tick(slow_tick),.enable(enable),.motor_reverse(1'b0),.target(target),
 .velocity(velocity),.kp(kp),.ki(ki),.feedforward(ff),.applied(slow));
 integer n;
 task step(input slow_enable);begin
 @(negedge clk);fast_tick=1;slow_tick=slow_enable;@(negedge clk);fast_tick=0;slow_tick=0;repeat(50)@(negedge clk);
 end endtask
 initial begin
 repeat(4)@(negedge clk);rst=1;
 for(n=1;n<=1000;n=n+1)begin step(n%10==0);
 if(n%10==0 && (fast-slow>1 || slow-fast>1))$fatal(1,"100Hz-reference Ki/slew changed fast=%d slow=%d n=%d",fast,slow,n);
 end
 enable=0;repeat(3)@(negedge clk);if(fast!=0)$fatal(1,"disable failed");
 target=10;velocity=20;kp=2048;ki=0;ff=150;enable=1;
 for(n=0;n<150;n=n+1)step(0);
 if(fast!=70)$fatal(1,"feedforward/PI sum wrong %d",fast);
 target=0;ff=0;for(n=0;n<160;n=n+1)step(0);
 if(fast!=0 || a.integral!=0)$fatal(1,"zero target did not reset integral/coast");
 target=-10;velocity=0;kp=2048;ki=0;
 for(n=0;n<150;n=n+1)step(0);
 if(fast>=0)$fatal(1,"negative demand/reversal failed");
 enable=0;repeat(3)@(negedge clk);if(fast!=0)$fatal(1,"stop priority failed");
 $display("PASS 1kHz PI: matched 100Hz-reference integral/slew, signed feedforward, reversal and STOP");$finish;
 end
endmodule
