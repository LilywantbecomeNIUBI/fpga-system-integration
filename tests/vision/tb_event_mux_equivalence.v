`timescale 1ns/1ps
module tb_event_mux_equivalence;
 reg clk=0;always #4 clk=~clk;
 reg rst=0,av=0,bv=0,ready=0;reg [255:0] a=0,b=0;
 wire ar0,ar1,br0,br1,v0,v1;wire [255:0] p0,p1;
 event_packet_mux dut(clk,rst,av,bv,a,b,ar0,br0,ready,v0,p0);
 event_packet_mux_reference refdut(clk,rst,av,bv,a,b,ar1,br1,ready,v1,p1);
 integer i,j;reg [31:0] seed=32'h4d555843;
 initial begin
  repeat(4)@(negedge clk);rst=1;
  for(i=0;i<5000;i=i+1)begin
   @(negedge clk);seed=seed^(seed<<13);seed=seed^(seed>>17);seed=seed^(seed<<5);
   av=seed[0];bv=seed[1];ready=seed[2] && i%101>20;
   for(j=0;j<8;j=j+1)begin a[j*32+:32]=seed+i+j;b[j*32+:32]=~seed+i+j;end
   #1;if({ar0,br0,v0}!=={ar1,br1,v1} || (v0 && p0!==p1))$fatal(1,"mux handshake/held payload changed cycle%0d",i);
  end
  $display("PASS event mux5000 cycle-exact arbitration, backpressure and immutable-payload comparisons");$finish;
 end
endmodule
