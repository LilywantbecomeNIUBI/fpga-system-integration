`timescale 1ns/1ps
module tb_intent_mux;
 reg clk=0;always #10 clk=~clk;reg rst=0,cv=0,armed=0,nr=0,np=0;
 reg[7:0] op=0,id=0,flags=1,cs=250;reg signed[15:0] l=0,r=0;reg[15:0] kp=256,ki=16;
 wire v,owner,inhibit;wire[7:0] co,ci,cf;wire signed[15:0] cl,cr;wire[15:0] ck,ck2;
 nav_command_mux #(.CLK_HZ(1000)) dut(clk,rst,cv,op,id,flags,l,r,kp,ki,armed,cs,nr,np,16'sd12,16'sd8,v,co,ci,cf,cl,cr,ck,ck2,owner,inhibit);
 initial begin
  repeat(3)@(negedge clk);rst=1;cv=1;op=1;id=10;@(negedge clk);cv=0;
  if(!v || co!=1)$fatal(1,"explicit arm missing");armed=1;nr=1;repeat(12)@(negedge clk);
  if(!owner || !inhibit)$fatal(1,"unknown auto not gated");np=1;wait(v);@(negedge clk);
  if(co!=2 || ci!=251 || cl!=12 || cr!=8 || ck!=256 || ck2!=16)$fatal(1,"auto target/gains/sequence incorrect");
  np=0;#1;if(!inhibit)$fatal(1,"hazard gate delayed");nr=0;repeat(2)@(negedge clk);
  if(!inhibit)$fatal(1,"completion reopened stale PWM");armed=0;repeat(2)@(negedge clk);if(owner)$fatal(1,"owner not released");
  nr=1;np=1;armed=1;repeat(2)@(negedge clk);cv=1;op=0;@(negedge clk);cv=0;if(co!=0 || !v)$fatal(1,"STOP lost");
  $display("PASS motor intent ownership, explicit ARM, immediate gate, STOP priority");$finish;
 end
endmodule
