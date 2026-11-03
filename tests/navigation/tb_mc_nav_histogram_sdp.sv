`timescale 1ns/1ps
// Compare settled behavior with the immutable original, including same-address
// read/write forwarding, invalid reads, native bursts, timestamp wrap and clear.
module tb_mc_nav_histogram_sdp;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,clear=0,pv=0;reg [15:0] now_ms=0,angle=0,distancev=0;reg [7:0] intensity=100;
 reg [6:0] index=0;wire [15:0] d0,d1;wire k0,k1,h0,h1,o0,o1,r0,r1;
 wire [31:0] seq0,seq1;wire ref_rst=rst && !clear;
 nav_histogram #(.SYNC_CLEAR(1)) dut(.clk(clk),.rst_n(rst),.sync_clear(clear),.now_ms(now_ms),
  .point_valid(pv),.angle_q100(angle),.distance_mm(distancev),.intensity(intensity),.angle_offset(16'sd0),.angle_reverse(1'b0),
  .timeout_ms(16'd200),.stop_mm(16'd450),.min_intensity(8'd1),.read_index(index),.read_distance(d0),.read_known(k0),
  .clear_hazard(1'b0),.clear_sequence(32'd0),.hazard(h0),.hazard_sequence(seq0),.overflow(o0),.ready(r0));
 nav_histogram_reference refdut(.clk(clk),.rst_n(ref_rst),.now_ms(now_ms),
  .point_valid(pv),.angle_q100(angle),.distance_mm(distancev),.intensity(intensity),.angle_offset(16'sd0),.angle_reverse(1'b0),
  .timeout_ms(16'd200),.stop_mm(16'd450),.min_intensity(8'd1),.read_index(index),.read_distance(d1),.read_known(k1),
  .clear_hazard(1'b0),.clear_sequence(32'd0),.hazard(h1),.hazard_sequence(seq1),.overflow(o1),.ready(r1));
 integer comparisons=0,n,j;reg [31:0] seed=32'h4d415052;
 task tick;begin @(negedge clk);end endtask
 task compare_all;
  begin
   for(j=0;j<128;j=j+1)begin index=j;repeat(3)tick;
    if(k0!==k1 || (j<72 && d0!==d1))$fatal(1,"hist SRAM/original differs index=%0d distancev=%0d/%0d known=%0b/%0b",j,d0,d1,k0,k1);
    comparisons=comparisons+1;
   end
   if(h0!==h1 || seq0!==seq1 || o0!==o1)$fatal(1,"raw hazard contract changed");
  end
 endtask
 task point(input integer a,d,q);
  begin tick;angle=a;distancev=d;intensity=q;pv=1;tick;pv=0;repeat(90)tick;end
 endtask
 initial begin
  repeat(4)tick;rst=1;repeat(76)tick;compare_all;
  for(n=0;n<180;n=n+1)begin
   seed=seed^(seed<<13);seed=seed^(seed>>17);seed=seed^(seed<<5);
   point((n%72)*500+100,seed%60000+1,n%11==0?0:100);
   point((n%72)*500+200,seed%50000+1,100);
   if(n%12==0)compare_all;
  end
  // Consecutive native points including repeated sectors.
  tick;pv=1;intensity=100;
  for(n=0;n<12;n=n+1)begin angle=2000+n*20;distancev=1500-n*10;tick;end
  pv=0;repeat(1100)tick;compare_all;
  now_ms=65530;point(600,4000,100);point(100,4200,100);point(200,4100,100);
  now_ms=10;compare_all;now_ms=195;compare_all;
  // Synchronous clear wins even in an in-flight read/modify/write transaction.
  tick;pv=1;angle=100;distancev=123;tick;pv=0;repeat(4)tick;
  clear=1;#1;if(dut.rst_n!==1)$fatal(1,"soft clear reached asynchronous reset");
  tick;if(r0 || k0 || !h0 || dut.wp || dut.rp)$fatal(1,"soft clear did not discard FIFO/state");
  tick;clear=0;repeat(76)tick;compare_all;
  $display("PASS histogram SDP/original %0d settled reads, two-point qualification, native bursts, same-word commits, age wrap, invalid addresses, in-flight sync clear",comparisons);
  $finish;
 end
 initial begin #5000000;$fatal(1,"histogram SDP watchdog");end
endmodule
