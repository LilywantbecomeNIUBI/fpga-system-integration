`timescale 1ns/1ps
module tb_mc_nav_raymap;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;reg [31:0] ms=0,errors=0;reg pv=0;reg [15:0] angle=0,distance=2025;reg [7:0] intensity=50;
 wire begin_scan,abort_scan,publish,cv,cr,cd,initializing,valid,bad;wire [13:0] ca;wire [1:0] cc;
 wire [31:0] epoch,stamp,meta,loss,last_ms;wire [7:0] dangers;
 reg [31:0] self_start=0,self_end=0,self_distance=0;
 reg lock=0,rv=0;reg [13:0] ra=0;wire rr,rd;wire [1:0] rc;
 mc_nav_raymap mapper(.clk(clk),.rst_n(rst),.enable(1'b1),.now_ms(ms),.point_valid(pv),.angle_q100(angle),.distance_mm(distance),.intensity(intensity),.error_count(errors),
  .pose_x_q16(32'sd0),.pose_y_q16(32'sd0),.heading_q16(32'sd0),.laser_x_mm(16'sd0),.laser_y_mm(16'sd0),.laser_yaw_q16(32'sd0),
  .self_start(self_start),.self_end(self_end),.self_distance(self_distance),
  .front_mm(16'd100),.back_mm(16'd100),.half_width_mm(16'd50),.stop_margin_mm(16'd200),.grid_ready(cr),.cell_done(cd),.grid_epoch(epoch),.initializing(initializing),
  .scan_begin(begin_scan),.scan_abort(abort_scan),.publish(publish),.cell_valid(cv),.cell_address(ca),.cell_code(cc),
  .anchor_x_mm(),.anchor_y_mm(),.anchor_heading(),.published_x_mm(),.published_y_mm(),.published_heading(),.metadata_epoch(meta),.overflow_count(loss),.danger_mask(dangers),.last_point_ms(last_ms));
 mc_nav_grid grid(.clk(clk),.rst_n(rst),.now_ms(ms),.scan_begin(begin_scan),.scan_abort(abort_scan),.publish(publish),.planner_lock(lock),
  .cell_valid(cv),.cell_address(ca),.cell_code(cc),.cell_ready(cr),.cell_done(cd),.read_valid(rv),.read_address(ra),.read_ready(rr),.read_done(rd),.read_code(rc),
  .map_valid(valid),.epoch(epoch),.map_ms(stamp),.foreground(),.initializing(initializing),.scan_bad(bad));
 task tick;begin @(negedge clk);end endtask
 task check_cell(input [6:0] x,y,input [1:0] expected);begin
  lock=1;ra={y,x};rv=1;while(!rr)tick;tick;rv=0;while(!rd)tick;
  if(rc!==expected)$fatal(1,"ray cell %d,%d got %d expected %d",x,y,rc,expected);
  lock=0;tick;
 end endtask
 integer i;
 initial begin
  repeat(4)tick;rst=1;repeat(2200)tick;
  // Twelve consecutive points per packet, followed by the real wire gap.
  for(i=0;i<1104;i=i+1)begin angle=(35000+i*100)%36000;pv=1;tick;pv=0;
   if(i%12==11)repeat(25000)tick;
  end
  repeat(20000)tick;
  if(!valid || epoch<2 || meta!=epoch || loss!=0)$fatal(1,"scan assembly epoch=%d meta=%d loss=%d",epoch,meta,loss);
  check_cell(64,64,1);check_cell(70,64,1);check_cell(104,64,2);check_cell(127,127,0);
  // Sparse long ray: retain its true slope, stop when it first exits the map.
  // A near self return is ignored only inside the calibrated body; a real
  // front obstacle in the same angular mask MUST remain occupied.
  self_end=200;self_distance=500;
  for(i=1104;i<1848;i=i+1)begin
   angle=(35000+i*100)%36000;distance=angle==1000?20000:angle==0?275:angle==100?50:0;
   pv=1;tick;pv=0;if(i%12==11)repeat(25000)tick;
  end
  repeat(20000)tick;
  check_cell(64,64,1);check_cell(69,64,2);check_cell(100,70,1);check_cell(80,80,0);
  if(!dangers[0])$fatal(1,"self mask hid a real obstacle outside body");
  angle=4000;distance=50;pv=1;tick;pv=0;repeat(3000)tick;
  if(dangers!=255)$fatal(1,"unmasked interior return authorized escape");
  // Saturation must invalidate the affected scan, never silently publish it.
  i=epoch;errors=1;distance=0;
  repeat(100)begin angle=(angle+100)%36000;pv=1;tick;end
  pv=0;repeat(20000)tick;
  if(loss==0 || epoch!=i)$fatal(1,"FIFO overflow published incomplete map");
  $display("PASS native 12-point bursts -> complete double-buffer maps, ray FREE/end OCCUPIED, body FREE, metadata epoch, zero FIFO loss");$finish;
 end
 initial begin #1000000000;$fatal(1,"ray/map deadlock");end
endmodule
