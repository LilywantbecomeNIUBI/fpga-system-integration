`timescale 1ns/1ps
module tb_pipeline_observer;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,clear=1,tick=0,compare=1;reg signed [31:0] position=0;
 wire signed [31:0] actual,expected;wire recent,ref_recent,fault,ref_fault,mode,ref_mode;
 wheel_speed_hybrid #(.CLK_HZ(1000000)) a(.clk(clk),.reset_n(rst),.clear(clear),.tick(tick),.position(position),
 .velocity_q8(actual),.motion_recent(recent),.fault(fault),.count_mode(mode));
 reference_speed_hybrid #(.CLK_HZ(1000000)) b(.clk(clk),.reset_n(rst),.clear(clear),.tick(tick),.position(position),
 .velocity_q8(expected),.motion_recent(ref_recent),.fault(ref_fault),.count_mode(ref_mode));
 integer phase=0,i,seed=32'h04250311,checks=0;
 task cycle(input integer delta);begin
  @(negedge clk);position=position+delta;tick=phase==999;
  phase=phase==999 ? 0 : phase+1;
  @(posedge clk);#1;
  if(compare && phase==40)begin
   if(actual!==expected || mode!==ref_mode || fault!==ref_fault || recent!==ref_recent)
    $fatal(1,"observer equivalence at %0d actual=%0d expected=%0d faults=%0d/%0d",i,actual,expected,fault,ref_fault);
   if(a.dividing || a.sample_state!=0)$fatal(1,"observer missed core's 40-clock deadline");
   checks=checks+1;
  end
 end endtask
 task restart(input signed [31:0] p);begin
  @(negedge clk);clear=1;position=p;tick=0;phase=0;
  repeat(3)@(negedge clk);clear=0;
 end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;restart(32'h7ffffff0);
  for(i=0;i<120000;i=i+1)cycle(i%173==0 ? 2 : 0);
  for(i=0;i<30000;i=i+1)cycle(i%239==0 ? -1 : 0);
  for(i=0;i<40000;i=i+1)cycle(i%5500==0 ? -2 : 0);
  for(i=0;i<52000;i=i+1)cycle(0);
  if(actual!=0 || recent)$fatal(1,"stop aging failed");
  // Full physically representable bucket: two position units each clock.
  restart(0);for(i=0;i<12000;i=i+1)cycle(2);
  for(i=0;i<40;i=i+1)cycle(2);
  if(fault || !mode || actual!=5120000)$fatal(1,"positive bucket bound %0d fault=%0d",actual,fault);
  for(i=0;i<12000;i=i+1)cycle(-2);
  for(i=0;i<40;i=i+1)cycle(-2);
  if(fault || !mode || actual!=-5120000)$fatal(1,"negative bucket bound %0d fault=%0d",actual,fault);
  restart(0);
  for(i=0;i<40000;i=i+1)cycle(($random(seed)&31)==0 ? (($random(seed)&1) ? 1 : -2) : 0);
  // Sampling stops: the new narrow bucket must latch a fault, not wrap.
  compare=0;restart(0);
  for(i=0;i<1100;i=i+1)begin @(negedge clk);position=position+2;end
  repeat(3)@(negedge clk);if(!fault || actual!=0)$fatal(1,"missing tick silently truncated bucket");
  restart(position);repeat(3)@(negedge clk);if(fault || actual!=0)$fatal(1,"clear left in-flight result");
  $display("PASS observer pipeline: %0d reference samples, 40-clock deadline, signed full buckets, wrap/reverse/stop/random and overflow fault",checks);$finish;
 end
 initial begin #4000000;$fatal(1,"observer pipeline timeout");end
endmodule
