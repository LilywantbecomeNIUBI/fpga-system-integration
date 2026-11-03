`timescale 1ns/1ps
module tb_mc_nav_mailbox;
 reg sc=0,dc=0,srun=1,drun=1,arst=0,sv=0;reg [63:0] data=0;
 always #7 if(srun)sc=~sc;else sc=0;
 always #11 if(drun)dc=~dc;else dc=0;
 wire sr,dr,dv;wire [63:0] dd;
 reset_sync source_reset(.clk(sc),.arst_n(arst),.rst_n(sr));
 reset_sync dest_reset(.clk(dc),.arst_n(arst),.rst_n(dr));
 vision_mailbox #(.WIDTH(64)) dut(.src_clk(sc),.src_rst_n(sr),.src_valid(sv),.src_data(data),
 .dst_clk(dc),.dst_rst_n(dr),.dst_valid(dv),.dst_data(dd),.src_reset_bank(4'd0),.dst_reset_bank(2'd0));
 integer received=0,i,old_count;
 always @(negedge dc)if(dr && dv)begin
  if(dd[63:32]!==~dd[31:0])$fatal(1,"torn navigation time/risk snapshot");received=received+1;
 end
 task send(input [31:0] n);begin @(negedge sc);data={n,~n};sv=1;end endtask
 initial begin
  repeat(5)@(negedge sc);arst=1;repeat(5)@(negedge sc);
  for(i=1;i<=50;i=i+1)send(i);
  drun=0;
  for(i=51;i<=200;i=i+1)send(i);
  @(negedge sc);sv=0;repeat(10)@(negedge sc);drun=1;
  repeat(100)@(negedge sc);if(dd!={32'd200,~32'd200})$fatal(1,"backpressure lost latest coalesced snapshot");
  send(201);@(negedge sc);sv=0;srun=0;old_count=received;
  repeat(30)@(negedge dc);if(received!=old_count+1)$fatal(1,"stopped source duplicated/lost in-flight generation");
  drun=0;arst=0;#100;if(dv || dd)$fatal(1,"common asynchronous reset failed while clocks stopped");
  arst=1;drun=1;repeat(20)@(negedge dc);if(dv || dd)$fatal(1,"reset replayed old generation");
  srun=1;repeat(10)@(negedge sc);send(400);@(negedge sc);sv=0;
  repeat(50)@(negedge sc);if(dd!={32'd400,~32'd400})$fatal(1,"mailbox did not recover after stopped-clock reset");
  $display("PASS atomic 64bit navigation mailbox, clock stop, backpressure/coalescing, common reset and new generation");$finish;
 end
 initial begin #1000000;$fatal(1,"mailbox clock-stop deadlock");end
endmodule
