`timescale 1ns/1ps
module tb_mc_nav_deadline;
 reg clk=0;always #10 clk=~clk;reg rst=0,start=0,abort=0;
 reg [31:0] epoch=1;wire busy,done,safe,timeout;wire [31:0] cycles;
 mc_nav_sweep #(.MAX_CYCLES(100)) dut(.clk(clk),.rst_n(rst),.start(start),.abort(abort),.map_epoch(epoch),
 .x_mm(32'sd0),.y_mm(32'sd0),.theta_q16(32'sd0),.step_x(16'sd0),.step_y(16'sd0),.step_theta(32'sd0),.steps(6'd20),
 .front_mm(16'd100),.back_mm(16'd100),.half_width_mm(16'd50),.radius_mm(16'd120),.pad_mm(16'd68),
 .busy(busy),.done(done),.safe(safe),.timed_out(timeout),.cycles(cycles),
 .read_ready(1'b0),.read_done(1'b0),.read_code(2'd0));
 task launch;begin start=1;@(negedge clk);start=0;end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;launch;
  while(!done)@(negedge clk);
  if(safe || !timeout || busy || cycles>102)$fatal(1,"deadline failed to stop stalled transaction");
  @(negedge clk);if(done)$fatal(1,"deadline emitted repeated completion");
  launch;repeat(5)@(negedge clk);epoch=2;while(!done)@(negedge clk);
  if(safe || timeout || busy)$fatal(1,"map generation abort failed");
  @(negedge clk);launch;repeat(5)@(negedge clk);abort=1;
  while(!done)@(negedge clk);if(safe || busy)$fatal(1,"explicit abort failed");
  $display("PASS hard deadline, stalled RAM, single completion, map epoch abort and explicit abort");$finish;
 end
 initial begin #100000;$fatal(1,"timeout deadlock");end
endmodule
