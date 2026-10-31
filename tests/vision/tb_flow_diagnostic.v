`timescale 1ns/1ps
module tb_flow_diagnostic;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0;wire busy,sv,sg;wire [143:0] diagnostic;
 flow_grid_pyramid #(.WIDTH(24),.HEIGHT(24)) dut(.clk(clk),.rst_n(rst_n),.start(start),
  .invalidate(1'b0),.frame_id(16'd123),.busy(busy),.sample_valid(sv),.sample_good(sg),.read_data(8'd0),.debug_status(diagnostic));
 task launch;
 begin repeat(4)@(negedge clk);rst_n=1;@(negedge clk);start=1;@(negedge clk);start=0;end
 endtask
 initial begin
  launch();wait(dut.patch.busy);@(negedge clk);
  force dut.patch.state=dut.patch.WAIT_U;force dut.patch.div_done=0;
  repeat(1000)@(negedge clk);
  if(!busy || diagnostic[135:128]!=2 || diagnostic[79:72]!=dut.patch.WAIT_U || diagnostic[47:32]<900 || diagnostic[31:16]<900)
   $fatal(1,"Stalled divider wait was not observable: %h",diagnostic);
  release dut.patch.state;release dut.patch.div_done;
  rst_n=0;launch();force dut.pd=0;
  repeat(5000)@(negedge clk);
  if(!busy || diagnostic[135:128]!=2 || diagnostic[79:72]!=0 || diagnostic[15:8]!=1 || diagnostic[7:0]!=0)
   $fatal(1,"Lost patch completion was not distinguishable: %h",diagnostic);
  release dut.pd;
  rst_n=0;launch();wait(dut.patch.busy);@(negedge clk);force dut.patch.state=5'd31;
  @(negedge clk);release dut.patch.state;
  wait(dut.pd);@(negedge clk);
  if(dut.patch.busy || dut.pg || !dut.patch.debug_fault[11] || dut.patch.debug_fault[10] || dut.patch.debug_fault[9:5]!=31)
   $fatal(1,"Illegal patch state did not return invalid completion with sticky evidence");
  wait(sv);@(negedge clk);
  if(sg || !diagnostic[55] || diagnostic[54:50]!=31)
   $fatal(1,"Grid did not consume invalid completion / report first fault");
  rst_n=0;launch();wait(dut.patch.busy);@(negedge clk);force dut.patch.state=5'd0;
  @(negedge clk);release dut.patch.state;
  wait(dut.pd);@(negedge clk);
  if(dut.patch.busy || dut.pg || !dut.patch.debug_fault[11] || !dut.patch.debug_fault[10] || dut.patch.debug_fault[9:5]!=0)
   $fatal(1,"Observed IDLE/busy contradiction did not return invalid done");
  $display("PASS flow diagnostic: waits, illegal encoding and observed IDLE/busy fault handled");$finish;
 end
 initial begin #200000;$fatal(1,"timeout");end
endmodule
