`timescale 1ns/1ps
module tb_gmr_rate;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,a=0,b=0;reg [8:0] divider=0;
 wire tick=divider==499;
 always @(posedge clk)if(!reset_n || tick)divider<=0;else divider<=divider+1'b1;
 wire signed [31:0] raw_position,raw_velocity,control_position,control_velocity;
 encoder_ab #(.FILTER_CYCLES(8)) encoder(.clk(clk),.reset_n(reset_n),.a(a),.b(b),.sample_tick(tick),.reverse(1'b0),
 .position(raw_position),.velocity(raw_velocity),.invalid_count(),.diagnostics(),.diagnostic_levels(),.transition_trace(),.transition_count());
 encoder_control_scale #(.SHIFT(4)) scale(.clk(clk),.reset_n(reset_n),.sample_tick(tick),.raw_position(raw_position),.raw_velocity(raw_velocity),
 .control_position(control_position),.control_velocity(control_velocity));
 // At 50MHz, 136 clocks/AB edge corresponds to about 91.9k AB cycles/s,
 // covering MG513P30 nominal 366 rpm: 366/60 * 30 * 500 = 91.5k cycles/s.
 task phase(input [1:0] value);begin @(negedge clk);{a,b}=value;repeat(136)@(negedge clk);end endtask
 integer i;
 initial begin
  #1;repeat(4)@(negedge clk);reset_n=1;repeat(30)@(negedge clk);
  repeat(500)begin phase(1);phase(3);phase(2);phase(0);end
  repeat(1100)@(negedge clk);
  if(raw_position!=2000 || control_position!=125)$fatal(1,"GMR 500ppr high-rate forward %d %d",raw_position,control_position);
  repeat(500)begin phase(2);phase(3);phase(1);phase(0);end
  repeat(1100)@(negedge clk);
  if(raw_position!=0 || control_position!=0)$fatal(1,"GMR high-rate reverse");
  $display("PASS GMR nominal-rate acquisition: 500 AB cycles in each direction with 8-clock filter, raw and normalized positions");$finish;
 end
 initial begin #10000000;$fatal(1,"GMR rate timeout");end
endmodule
