`timescale 1ns/1ps
module tb_histogram;
 reg clk=0;always #10 clk=~clk;reg rst=0,pv=0,clear=0;
 reg[15:0] now_ms=0,angle=0,distancev=4000;reg[7:0] intensity=100;
 reg[6:0] index=0;reg[31:0] clear_seq=0;
 wire[15:0] range_out;wire known,hazard,overflow,ready;wire[31:0] seq,settled;
 nav_histogram dut(clk,rst,now_ms,pv,angle,distancev,intensity,16'sd0,1'b0,16'd200,16'd450,8'd1,index,range_out,known,clear,clear_seq,hazard,seq,settled,overflow,ready);
 task point(input[15:0] a,d,input[7:0] q);
 begin @(negedge clk);pv=1;angle=a;distancev=d;intensity=q;@(negedge clk);pv=0;repeat(80)@(negedge clk);end endtask
 task readbin(input[6:0] b);begin index=b;repeat(3)@(negedge clk);end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;wait(ready);readbin(0);if(known || !hazard)$fatal(1,"reset unknown");
  point(100,4000,100);readbin(0);if(known)$fatal(1,"single point made free");
  point(200,3500,100);readbin(0);if(!known || range_out!=3500)$fatal(1,"two-point commit failed");
  now_ms=199;readbin(0);if(!known)$fatal(1,"early expiry");now_ms=200;readbin(0);if(known)$fatal(1,"deadline expiry");
  // Wrap-safe timestamps and a fresh complete visit supersede old minima.
  now_ms=65530;point(600,4000,100);point(100,4200,100);point(200,4100,100);now_ms=10;
  readbin(0);if(!known || range_out!=4100)$fatal(1,"timestamp wrap/minimum refresh");
  point(600,4000,100);point(100,200,100);readbin(0);if(range_out!=200 || !hazard)$fatal(1,"near sample bypass");
  point(200,0,0);readbin(0);if(known)$fatal(1,"invalid return treated free");
  point(300,5000,100);readbin(0);if(known)$fatal(1,"tainted visit released");
  point(600,4000,100);point(100,4000,100);point(200,4000,100);readbin(0);if(!known)$fatal(1,"new valid visit failed");
  // Old scan generation cannot clear a newly queued raw hazard.
  clear_seq=seq;point(100,180,100);clear=1;@(negedge clk);clear=0;if(!hazard)$fatal(1,"stale generation release");
  repeat(4)@(negedge clk);if(settled!=seq)$fatal(1,"settled generation did not catch up");
  @(negedge clk);pv=1;angle=35999;distancev=4000;repeat(70)@(negedge clk);pv=0;
  repeat(2)@(negedge clk);if(!overflow || !hazard)$fatal(1,"FIFO overflow not fail closed");
  $display("PASS histogram burst/coverage/invalid/stale/wrap/hazard generations/overflow");$finish;
 end
endmodule
