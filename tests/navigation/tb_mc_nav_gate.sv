`timescale 1ns/1ps
module tb_mc_nav_gate;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,global_permit=1,flow=0,rotation=0;reg [7:0] mask=0;
 reg signed [15:0] tx=100,ty=0,ax=0,ay=0;wire permit;wire [7:0] blocked;
 mc_nav_direction_gate gate(.clk(clk),.rst_n(rst),.global_permit(global_permit),.danger_mask(mask),.forward_flow_risk(flow),.rotation_requested(rotation),
  .target_x(tx),.target_y(ty),.actual_x(ax),.actual_y(ay),.permit(permit),.blocked_mask(blocked));
 task tick;begin @(negedge clk);end endtask
 task check(input bit expected);begin repeat(60)tick;if(permit!==expected)$fatal(1,"gate mask=%h target %d,%d actual %d,%d got %b",mask,tx,ty,ax,ay,permit);end endtask
 initial begin
  repeat(3)tick;rst=1;check(1);
  mask=1;#1;if(permit)$fatal(1,"new hazard did not immediately revoke old permit");check(0);
  tx=0;ty=30;check(1); // tangential escape, no forward component
  ax=20;check(0);ax=0;check(1); // actual momentum also matters
  flow=1;tx=10;check(0);tx=0;check(1);
  mask=0;rotation=1;check(0);rotation=0;check(1); // flow hazard permits translation tangent, never an unchecked rotating corner
  mask=16;flow=0;tx=10;ty=-10;check(1); // NE normal dot tangent = 0
  ty=0;check(0);rotation=1;tx=0;ty=-10;check(0);
  rotation=0;global_permit=0;check(0);
  $display("PASS directional three-stage normals, tangent escape, actual momentum, flow, diagonal dot, immediate revoke and global STOP");$finish;
 end
endmodule
