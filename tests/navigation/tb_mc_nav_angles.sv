`timescale 1ns/1ps
module tb_mc_nav_angles;
 reg clk=0;always #10 clk=~clk;reg rst=0;
 mc_nav_raymap dut(.clk(clk),.rst_n(rst),.enable(1'b1),.now_ms(32'd0),.point_valid(1'b0),
 .angle_q100(16'd0),.distance_mm(16'd0),.intensity(8'd0),.error_count(32'd0),
 .pose_x_q16(32'sd0),.pose_y_q16(32'sd0),.heading_q16(32'sd0),
 .laser_x_mm(16'sd0),.laser_y_mm(16'sd0),.laser_yaw_q16(32'sd205887),
 .front_mm(16'd100),.back_mm(16'd100),.half_width_mm(16'd50),.stop_margin_mm(16'd100),
 .self_start(32'd0),.self_end(32'd0),.self_distance(32'd0),.grid_ready(1'b1),.cell_done(1'b1),.grid_epoch(32'd0),.initializing(1'b0));
 initial begin
  repeat(3)@(negedge clk);rst=1;@(negedge clk);
  // A valid +15deg body, -180deg anchor, +180deg laser mount and +180deg
  // ray needs normalization BEFORE summing the three angle components.
  dut.record={32'd0,32'd0,32'sd17157,16'd0,16'd0,8'd50,16'd2000,16'd18000};
  dut.anchor_heading=-205887;dut.anchor_cs=-65536;dut.anchor_sn=0;
  dut.angle_product=64'sd205887<<<20;dut.state=8;
  wait(dut.state==9 && dut.trig_mode==1);#1;
  if(dut.trig_angle< -188740 || dut.trig_angle> -188720)$fatal(1,"angle normalization aliased valid ray %d",dut.trig_angle);
  $display("PASS body/anchor wrap plus 180-degree laser mount and ray remains within signed CORDIC range");$finish;
 end
 initial begin #100000;$fatal(1,"angle pipeline deadlock");end
endmodule
