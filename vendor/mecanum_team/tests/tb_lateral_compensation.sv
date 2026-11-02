`timescale 1ns/1ps
module tb_lateral_compensation;
 reg clk=0;always #5 clk=~clk;reg reset_n=0;
 reg active=1,arrived=0;reg [1:0] mode=2;
 reg signed [15:0] x=0,y=320,angle=0;reg signed [31:0] py=0;
 wire [15:0] boost;
 lateral_pwm_compensation dut(.clk(clk),.reset_n(reset_n),.active(active),.arrived(arrived),.mode(mode),.goal_x(x),.goal_y(y),.goal_angle(angle),.position_y(py),.compensation(boost));
 task settle;begin repeat(6)@(negedge clk);end endtask
 initial begin
  repeat(3)@(negedge clk);reset_n=1;settle();if(boost!=150)$fatal(1,"left start must have 15 percent compensation");
  y=-320;settle();if(boost!=150)$fatal(1,"right start compensation");
  py=-240*65536;settle();if(boost!=75)$fatal(1,"near-goal taper");
  py=-320*65536;settle();if(boost!=0)$fatal(1,"at-goal creep");
  py=0;mode=1;#1;if(boost!=0)$fatal(1,"heading mode changed");
  mode=0;#1;if(boost!=0)$fatal(1,"manual mode changed");
  mode=2;x=1;#1;if(boost!=0)$fatal(1,"diagonal move changed");
  x=0;angle=1;#1;if(boost!=0)$fatal(1,"turning pose changed");
  angle=0;arrived=1;#1;if(boost!=0)$fatal(1,"arrival boost");
  arrived=0;active=0;#1;if(boost!=0)$fatal(1,"disabled boost");
  $display("PASS lateral feedforward: 15 percent both directions, taper, no creep, pure relative lateral moves only");$finish;
 end
endmodule
