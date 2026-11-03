`timescale 1ns/1ps
// Test the actual registered service/planner datapaths, not copied formulas.
module tb_mc_nav_reciprocals;
 reg clk=0;always #10 clk=~clk;reg rst=0;
 mc_nav_system dut(.clk(clk),.rst_n(rst),.command_valid(1'b0),.command_body(176'd0),.armed(1'b0),
  .positions(128'd0),.velocities(128'd0),.yaw_ok(1'b1),.imu_stale(1'b0),.imu_fault(1'b0),.yaw_q16(32'sd0),
  .imu_now_us(32'd0),.imu_sample_us(32'd0),.point_valid(1'b0),.point_angle(16'd0),.point_distance(16'd0),
  .point_intensity(8'd0),.point_errors(32'd0),.flow_valid(1'b0),.flow_snapshot(64'd0),.frame_ready(1'b1));
 integer value,waited,cases=0,expected;
 reg [15:0] acceleration=0;
 task tick;begin @(negedge clk);end endtask
 task angular(input integer a);
  begin
   dut.omega=a;dut.state=56;waited=0;
   while(dut.state!=30)begin tick;waited=waited+1;if(waited>10)$fatal(1,"omega reciprocal deadlock");end
   expected=a/10;
   if($signed(dut.omega_step_q16)!=expected)$fatal(1,"omega division %0d ->%0d expected%0d",a,dut.omega_step_q16,expected);
   dut.state=0;cases=cases+1;
  end
 endtask
 task steps(input integer a,b);
  begin
   dut.planner.test_x=a;dut.planner.test_y=b;dut.planner.state=10;waited=0;
   while(dut.planner.state!=9)begin tick;waited=waited+1;if(waited>7)$fatal(1,"step reciprocal deadlock");end
   if($signed(dut.planner.step_x_q)!=a/10 || $signed(dut.planner.step_y_q)!=b/10)$fatal(1,"step division %0d %0d",a,b);
   dut.planner.state=0;dut.planner.sweep_start=0;cases=cases+1;
  end
 endtask
 initial begin
  repeat(4)tick;rst=1;tick;
  force dut.running=1;force dut.map_fresh=1;force dut.imu_fresh=1;force dut.plan_tick=0;
  force dut.odom_overflow=0;force dut.map_epoch=1;dut.plan_epoch=1;
  force dut.planner_start=0;force dut.planner.sweep.start=0;force dut.planner.accel_mm_s2=acceleration;
  // Every reachable angular value, with exact signed truncation toward zero.
  for(value=-22876;value<=22876;value=value+1)angular(value);
  // Entire s16 step interface including -32768 and all quotient boundaries.
  for(value=-32768;value<=32767;value=value+1)steps(value,~value);
  // Full unsigned16 acceleration interface, including the suggested formula's
  // first counterexample43699. The implementation uses a valid wider reciprocal.
  for(value=0;value<=65535;value=value+1)begin
   acceleration=value;dut.planner.state=0;force dut.planner.start=1;tick;release dut.planner.start;tick;
   expected=value<20?1:value/20;
   if($signed(dut.planner.accel)!=expected)$fatal(1,"acceleration division %0d ->%0d",value,dut.planner.accel);
   dut.planner.state=0;dut.planner.busy=0;
  end
  $display("PASS registered reciprocal datapaths: %0d signed omega/step cases plus65536 acceleration values",cases);
  $finish;
 end
 initial begin #30000000;$fatal(1,"reciprocal watchdog");end
endmodule
