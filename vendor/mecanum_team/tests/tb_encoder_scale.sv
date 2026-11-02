`timescale 1ns/1ps
module tb_encoder_scale;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,tick=0;reg signed [31:0] raw=0;
 wire signed [31:0] position,velocity;
 encoder_control_scale #(.SHIFT(4)) dut(.clk(clk),.reset_n(reset_n),.sample_tick(tick),.raw_position(raw),.raw_velocity(32'sd0),
 .control_position(position),.control_velocity(velocity));
 task update(input signed [31:0] value);begin
  @(negedge clk);raw=value;tick=1;@(negedge clk);tick=0;repeat(2)@(negedge clk);
 end endtask
 integer i;
 initial begin
  #1;repeat(3)@(negedge clk);reset_n=1;
  for(i=1;i<=15;i=i+1)update(i);
  if(position!=0)$fatal(1,"fractional counts not retained");
  update(16);if(position!=1 || velocity!=1)$fatal(1,"slow pulse accumulation");
  for(i=15;i>=0;i=i-1)update(i);
  if(position!=0 || velocity!=-1)$fatal(1,"slow reverse accumulation");
  update(60000);if(position!=3750 || velocity!=3750)$fatal(1,"GMR 60000 raw = 3750 control");
  update(0);if(position!=0 || velocity!=-3750)$fatal(1,"reverse output shaft turn");
  update(-60000);if(position!=-3750)$fatal(1,"negative nominal turn");
  // 32-bit raw counter wrap must become an 8-count delta, not a huge jump.
  reset_n=0;repeat(3)@(negedge clk);reset_n=1;
  update(32'sh7ffffff8);update(32'sh80000000);
  if(position!=134217728 || velocity!=1)$fatal(1,"raw positive wrap %d %d",position,velocity);
  update(32'sh7ffffff8);if(position!=134217728 || velocity!=0)$fatal(1,"reverse wrap residual");
  update(32'sh7ffffff0);if(position!=134217727 || velocity!=-1)$fatal(1,"reverse wrap accumulation");
  $display("PASS GMR scale: 60000 raw/rev -> 3750 control/rev, signed residuals, slow pulses and raw counter wrap");$finish;
 end
 initial begin #100000;$fatal(1,"scale timeout");end
endmodule
