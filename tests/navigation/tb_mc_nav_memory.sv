`timescale 1ns/1ps
module tb_mc_nav_memory;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;reg [31:0] ms=0;reg begin_scan=0,abort_scan=0,publish=0,lock=0;
 reg uv=0;reg [13:0] ua=0;reg [1:0] uc=0;wire ur,ud;
 reg manual_rv=0;reg [13:0] manual_ra=0;wire rr,rd;wire [1:0] rc;
 wire valid,fg,initializing,bad;wire [31:0] epoch,stamp;
 reg sweep_start=0,sweep_abort=0,use_sweep=0;
 wire busy,done,safe,timeout,unknown_seen,srv;wire [13:0] sra;wire [31:0] cost,cycles;
 reg signed [31:0] px=0,py=0,angle=0;reg signed [15:0] sx=0,sy=0;reg [5:0] steps=1;
 mc_nav_grid grid(.clk(clk),.rst_n(rst),.now_ms(ms),.scan_begin(begin_scan),.scan_abort(abort_scan),.publish(publish),.planner_lock(lock),
 .cell_valid(uv),.cell_address(ua),.cell_code(uc),.cell_ready(ur),.cell_done(ud),
 .read_valid(use_sweep?srv:manual_rv),.read_address(use_sweep?sra:manual_ra),.read_ready(rr),.read_done(rd),.read_code(rc),
 .map_valid(valid),.epoch(epoch),.map_ms(stamp),.foreground(fg),.initializing(initializing),.scan_bad(bad));
 mc_nav_sweep sweep(.clk(clk),.rst_n(rst),.start(sweep_start),.abort(sweep_abort),.map_epoch(epoch),
 .x_mm(px),.y_mm(py),.theta_q16(angle),.step_x(sx),.step_y(sy),.step_theta(32'sd0),.steps(steps),
 .front_mm(16'd100),.back_mm(16'd100),.half_width_mm(16'd50),.radius_mm(16'd120),.pad_mm(16'd36),
 .busy(busy),.done(done),.safe(safe),.timed_out(timeout),.unknown_seen(unknown_seen),.unknown_cost(cost),.cycles(cycles),
 .read_valid(srv),.read_address(sra),.read_ready(rr),.read_done(rd),.read_code(rc));
 task tick;begin @(negedge clk);end endtask
 task begin_map;begin tick;begin_scan=1;tick;begin_scan=0;while(!ur)tick;end endtask
 task update_cell(input [6:0] x,y,input [1:0] code);begin
  while(!ur)tick;ua={y,x};uc=code;uv=1;tick;uv=0;while(!ud)tick;tick;
 end endtask
 task publish_map;reg [31:0] prev;begin prev=epoch;publish=1;tick;publish=0;while(epoch==prev)tick;tick;end endtask
 task check_cell(input [6:0] x,y,input [1:0] code);begin
  manual_ra={y,x};manual_rv=1;while(!rr)tick;tick;manual_rv=0;while(!rd)tick;
  if(rc!==code)$fatal(1,"cell (%d,%d) expected %d got %d",x,y,code,rc);tick;
 end endtask
 task check_sweep(input bit expect_safe);begin
  lock=1;use_sweep=1;sweep_start=1;tick;sweep_start=0;while(!done)tick;
  if(safe!==expect_safe || timeout)$fatal(1,"sweep safe=%b expected=%b cycles=%d",safe,expect_safe,cycles);
  lock=0;use_sweep=0;tick;
 end endtask
 integer i;
 initial begin
  repeat(3)tick;rst=1;while(initializing)tick;
  if(valid)$fatal(1,"boot map must be unavailable");
  begin_map;
  update_cell(64,64,1);update_cell(65,64,2);update_cell(66,64,3);
  update_cell(65,64,1);update_cell(66,64,1);update_cell(67,64,1);
  publish_map;lock=1;
  check_cell(64,64,1);check_cell(65,64,2);check_cell(66,64,3);check_cell(67,64,1);check_cell(68,64,0);
  lock=0;
  // Obstacle inside the filled rectangle, not merely on the perimeter.
  check_sweep(0);
  begin_map;update_cell(64,64,1);publish_map;check_sweep(1);
  if(!unknown_seen || cost==0)$fatal(1,"UNKNOWN must produce finite nonzero cost");
  // An intermediate trajectory pose intersects a cell; endpoint-only testing fails.
  begin_map;update_cell(70,64,2);publish_map;px=0;sx=100;steps=7;check_sweep(0);
  // Foreground cannot change under lock, including a pending read response.
  begin_map;update_cell(64,64,2);lock=1;publish=1;tick;publish=0;i=epoch;
  repeat(10)tick;if(epoch!=i)$fatal(1,"published while locked");lock=0;while(epoch==i)tick;
  // Bad scan cannot replace the foreground.
  begin_map;abort_scan=1;tick;abort_scan=0;i=epoch;publish=1;tick;publish=0;repeat(10)tick;
  if(epoch!=i)$fatal(1,"published poisoned scan");
  ms=stamp+250;tick;if(valid)$fatal(1,"expired map still valid");
  rst=0;tick;rst=1;while(initializing)tick;
  begin_map;publish_map;lock=1;check_cell(65,64,0);lock=0;
  $display("PASS packed RAM RMW, occupied priority, map epochs/lock/reset/expiry, filled sweep/UNKNOWN/intermediate collision");$finish;
 end
 initial begin #100000000;$fatal(1,"deadlocked kernel");end
endmodule
