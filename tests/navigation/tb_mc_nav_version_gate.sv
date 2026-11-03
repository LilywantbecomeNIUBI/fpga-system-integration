`timescale 1ns/1ps
// Service-level authorization boundary: independent of planner geometry tests.
module tb_mc_nav_version_gate;
 reg clk=0;always #10 clk=~clk;reg rst=0;
 wire permit;
 mc_nav_system dut(.clk(clk),.rst_n(rst),.command_valid(1'b0),.command_body(176'd0),
 .armed(1'b1),.positions(128'd0),.velocities(128'd0),.yaw_ok(1'b1),.imu_stale(1'b0),.imu_fault(1'b0),
 .yaw_q16(32'sd0),.imu_now_us(32'd100),.imu_sample_us(32'd100),.point_valid(1'b0),
 .point_angle(16'd0),.point_distance(16'd0),.point_intensity(8'd0),.point_errors(32'd0),
 .flow_valid(1'b0),.flow_snapshot(64'd0),.nav_permit(permit),.frame_ready(1'b1));
 initial begin
  repeat(3)@(negedge clk);rst=1;
  force dut.running=1;force dut.calibrated=1;force dut.map_fresh=1;force dut.imu_fresh=1;
  force dut.odom_overflow=0;force dut.hist_overflow=0;force dut.heading=0;
  force dut.plan_valid=1;force dut.map_epoch=10;force dut.plan_epoch=10;force dut.command_epoch=10;
  force dut.now_ms=100;force dut.command_ms=100;
  repeat(30)@(negedge clk);if(!permit)$fatal(1,"current plan not authorized");
  force dut.map_epoch=11;#1;if(permit)$fatal(1,"old-map plan escaped final gate");
  repeat(30)@(negedge clk);if(permit)$fatal(1,"old-map plan recovered without replan");
  force dut.plan_epoch=11;repeat(30)@(negedge clk);if(permit)$fatal(1,"starting a new plan retagged old motor command");
  force dut.command_epoch=11;repeat(30)@(negedge clk);if(!permit)$fatal(1,"committed new generation not authorized");
  force dut.running=0;#1;if(permit)$fatal(1,"stop did not revoke authorization");
  // Native laser bins must include both body/map heading and mounting yaw.
  force dut.running=1;force dut.local_x=0;force dut.local_y=0;
  force dut.local_tx=1000;force dut.local_ty=0;force dut.relative_heading=0;force dut.laser_yaw=102944;
  dut.state=43;force dut.math_done=1;force dut.math_result=64'h0001000000000000;
  @(negedge clk);release dut.math_done;release dut.math_result;
  wait(dut.state==15);@(negedge clk);
  if(dut.desired_heading< -9010 || dut.desired_heading> -8990 || dut.target_probe_index<53 || dut.target_probe_index>54)
   $fatal(1,"sensor mounting yaw ignored by histogram probe: %d bin=%d",dut.desired_heading,dut.target_probe_index);
  force dut.planner_start=0;force dut.blocked={72{1'b1}};force dut.hist_known=1;force dut.hist_distance=5000;
  dut.state=17;repeat(2)@(negedge clk);if(!dut.path_occluded)$fatal(1,"inflated VFH blockage failed to suppress cross-track penalty");
  force dut.blocked=0;
  repeat(3)begin dut.state=17;repeat(2)@(negedge clk);end
  if(dut.path_occluded || dut.clear_plans!=3)$fatal(1,"three clear plans did not restore rejoin ramp");
  force dut.imu_fresh=0;#1;
  if(permit)$fatal(1,"stale pose retained permission");
  @(negedge clk);if(dut.map_enable_q || dut.grid.rst_n!==rst)$fatal(1,"stale pose clear or common reset wiring failed");
  @(negedge clk);force dut.imu_fresh=1;release dut.map_fresh;
  @(negedge clk);if(dut.map_fresh || dut.grid_valid)$fatal(1,"pose recovery revived old map without complete scan");
  $display("PASS service final output blocks old-map generation immediately and preserves global STOP");$finish;
 end
 initial begin #1000000;$fatal(1,"service boundary test deadlock");end
endmodule
