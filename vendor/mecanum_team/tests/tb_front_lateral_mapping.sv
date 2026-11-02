`timescale 1ns/1ps
module tb_front_lateral_mapping;
 reg signed [15:0] x=0,y=0,z=0;
 wire signed [15:0] fl,fr,rl,rr;
 mecanum_mixer dut(.x(x),.y(y),.z(z),.fl(fl),.fr(fr),.rl(rl),.rr(rr));
 initial begin
  x=20;#1;if(fl!=20 || fr!=20 || rl!=20 || rr!=20)$fatal(1,"forward altered");
  x=0;z=20;#1;if(fl!=-20 || fr!=20 || rl!=-20 || rr!=20)$fatal(1,"rotation altered");
  z=0;y=20;#1;if(fl!=-20 || fr!=20 || rl!=20 || rr!=-20)$fatal(1,"standard lateral mapping");
  if(-fl+fr-rl+rr!=0)$fatal(1,"lateral command contains rotational component");
  y=-20;#1;if(fl!=20 || fr!=-20 || rl!=-20 || rr!=20)$fatal(1,"opposite lateral mapping");
  $display("PASS restored lateral mapping: opposed diagonals, no rotational component, forward/rotation preserved");$finish;
 end
endmodule
