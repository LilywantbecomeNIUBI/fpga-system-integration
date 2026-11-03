`timescale 1ns/1ps
module tb_mc_nav_planner;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,begin_scan=0,publish=0,uv=0,start=0,abort=0;
 reg [13:0] ua=0;reg [1:0] uc=1;wire ur,ud,init,valid,lock,rv,rr,rd;wire [13:0] ra;wire [1:0] rc;wire [31:0] epoch;
 reg [7:0] dangers=0;reg flow=0,degraded=0;reg signed [15:0] actual_x=0,actual_y=0;
 wire busy,done,found,limited,timeout,unknown_motion;wire signed [15:0] vx,vy;wire [31:0] cycles;
 reg [15:0] coarse=2000;
 mc_nav_grid grid(.clk(clk),.rst_n(rst),.now_ms(32'd0),.scan_begin(begin_scan),.scan_abort(1'b0),.publish(publish),.planner_lock(lock),
  .cell_valid(uv),.cell_address(ua),.cell_code(uc),.cell_ready(ur),.cell_done(ud),.read_valid(rv),.read_address(ra),.read_ready(rr),.read_done(rd),.read_code(rc),
  .map_valid(valid),.epoch(epoch),.map_ms(),.foreground(),.initializing(init),.scan_bad());
 mc_nav_planner planner(.clk(clk),.rst_n(rst),.start(start),.abort(abort),.map_epoch(epoch),.x_mm(32'sd0),.y_mm(32'sd0),.theta_q16(32'sd0),
  .target_x_mm(32'sd1000),.target_y_mm(32'sd0),.current_vx(actual_x),.current_vy(actual_y),.omega_step_q16(16'sd0),
  .direction_cos_q16(32'sd65536),.direction_sin_q16(32'sd0),.path_cos_q16(32'sd65536),.path_sin_q16(32'sd0),.path_weight_q8(9'd256),
  .speed_mm_s(16'd100),.accel_mm_s2(16'd1000),.front_mm(16'd100),.back_mm(16'd100),.half_width_mm(16'd50),.radius_mm(16'd120),.pad_mm(16'd56),
  .coarse_clearance_mm(coarse),.danger_mask(dangers),.flow_risk(flow),.flow_degraded(degraded),.path_occluded(1'b1),.allow_zero_translation(1'b0),
  .suggested_heading_q100(16'sd0),.busy(busy),.done(done),.found(found),.search_limited(limited),.timed_out(timeout),.unknown_motion(unknown_motion),
  .vx_mm_s(vx),.vy_mm_s(vy),.cycles(cycles),.planner_lock(lock),.read_valid(rv),.read_address(ra),.read_ready(rr),.read_done(rd),.read_code(rc));
 task tick;begin @(negedge clk);end endtask
 task put(input [13:0] address,input [1:0] code);begin
  while(!ur)tick;ua=address;uc=code;uv=1;tick;uv=0;while(!ud)tick;tick;
 end endtask
 integer i,n;
 task free_map;begin
  begin_scan=1;tick;begin_scan=0;while(!ur)tick;
  for(i=0;i<16384;i=i+1)put(i,1);
 end endtask
 task commit;reg [31:0] old;begin old=epoch;publish=1;tick;publish=0;while(epoch==old)tick;tick;end endtask
 task plan;begin start=1;tick;start=0;while(!done)tick;if(timeout || cycles>=2500000)$fatal(1,"planner deadline exceeded %d",cycles);tick;end endtask
 initial begin
  repeat(3)tick;rst=1;while(init)tick;
  free_map;commit;plan;
  if(!found || vx<=0)$fatal(1,"free-space navigation did not progress: %d %d",vx,vy);
  $display("RESULT free %d %d %d",vx,vy,cycles);
  dangers=1;flow=1;plan;
  if(!found || vx>0 || vy==0)$fatal(1,"front hazard deadlocked tangent escape %d %d",vx,vy);
  $display("RESULT escape %d %d %d",vx,vy,cycles);
  dangers=0;flow=0;degraded=1;plan;
  if(!found || (vx<0?-vx:vx)+(vy<0?-vy:vy)>50)$fatal(1,"degraded speed exceeded");
  free_map;put({7'd67,7'd65},2);commit;coarse=190;degraded=0;plan;
  if(!found)$fatal(1,"circle overlap incorrectly vetoed rectangle-safe motion");
  $display("RESULT circle_overlap %d %d %d",vx,vy,cycles);
  free_map;put({7'd64,7'd68},2);commit;actual_x=50;coarse=225;
  plan;if(found)$fatal(1,"first batch should be obstructed to exercise continuation");
  for(n=0;n<12 && !found;n=n+1)plan;
  if(!found)$fatal(1,"remaining candidates never reconsidered");
  $display("RESULT later_batch %d %d %d batches=%d",vx,vy,cycles,n);
  actual_x=1000;plan;
  if(found || !limited || vx!=0 || vy!=0)$fatal(1,"speed clamp fabricated an unreachable acceleration window");
  actual_x=0;
  abort=1;start=1;tick;start=0;while(!done)tick;if(found)$fatal(1,"aborted result authorized");
  $display("PASS bounded top-eight search, circle false positives, directional tangent escape, degraded cap and continuation");$finish;
 end
 initial begin #1000000000;$fatal(1,"planner deadlock");end
endmodule
