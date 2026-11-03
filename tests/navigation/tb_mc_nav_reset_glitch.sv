`timescale 1ns/1ps
module tb_mc_nav_reset_glitch;
 reg clk=0;always #10 clk=~clk;reg rst=0,window=0;
 reg [31:0] now_us=65535,sample_us=0;integer old_pulses=0,new_pulses=0;
 mc_nav_system n(.clk(clk),.rst_n(rst),.command_valid(1'b0),.command_body(176'd0),.armed(1'b0),
 .positions(128'd0),.velocities(128'd0),.yaw_ok(1'b1),.imu_stale(1'b0),.imu_fault(1'b0),.yaw_q16(32'sd0),
 .imu_now_us(now_us),.imu_sample_us(sample_us),.point_valid(1'b0),.point_angle(16'd0),.point_distance(16'd0),
 .point_intensity(8'd0),.point_errors(32'd0),.flow_valid(1'b0),.flow_snapshot(64'd0),.frame_ready(1'b1));
 mc_nav_system_pre_serial b(.clk(clk),.rst_n(rst),.command_valid(1'b0),.command_body(176'd0),.armed(1'b0),
 .positions(128'd0),.velocities(128'd0),.yaw_ok(1'b1),.imu_stale(1'b0),.imu_fault(1'b0),.yaw_q16(32'sd0),
 .imu_now_us(now_us),.imu_sample_us(sample_us),.point_valid(1'b0),.point_angle(16'd0),.point_distance(16'd0),
 .point_intensity(8'd0),.point_errors(32'd0),.flow_valid(1'b0),.flow_snapshot(64'd0),.frame_ready(1'b1));
 always @(negedge b.map_reset_n)if(window)old_pulses=old_pulses+1;
 always @(negedge n.grid.rst_n)if(window)new_pulses=new_pulses+1;
 task tick;begin @(negedge clk);end endtask
 initial begin
  repeat(4)tick;rst=1;repeat(5)tick;
  if(!n.map_enable_q || !b.map_reset_n || n.grid.rst_n!==rst)$fatal(1,"fresh pose failed to release map clear");
  // Emulate clock-to-Q / carry skew between timer bits, between sample edges.
  window=1;@(posedge clk);#1;now_us=131071;#1;now_us=65536;#1;
  if(old_pulses==0 || new_pulses!=0 || !n.map_enable_q)$fatal(1,"synchronous map clear did not reject the bus-skew pulse");
  window=0;tick;now_us=100001;
  #1;if(!n.map_enable_q)$fatal(1,"expiry bypassed registered clear predicate");
  tick;if(n.map_enable_q || n.grid.rst_n!==rst)$fatal(1,"legitimate expiry used generated async reset");
  sample_us=90000;tick;if(!n.map_enable_q || n.grid_valid)$fatal(1,"recovery revived an old published map");
  rst=0;#1;if(n.grid.rst_n)$fatal(1,"common reset lost async assertion");
  $display("PASS timer carry-skew rejected; map fault/origin uses synchronous clear, common reset alone drives async pins; expiry/recovery");$finish;
 end
 initial begin #100000;$fatal(1,"reset skew test deadlock");end
endmodule
