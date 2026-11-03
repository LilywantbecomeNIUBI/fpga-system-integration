`timescale 1ns/1ps
module tb_mc_nav_mission;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,byte_valid=0;reg [7:0] byte_data=0;wire cv;wire [175:0] body;wire [31:0] bad;
 reg [31:0] ms=0;reg armed=0,stop=0;wire active,running,origin,lease;
 wire [3:0] status,reason;wire [15:0] id;wire [4:0] wi,count;wire [7:0] ack,error;
 wire calibrated;wire signed [15:0] gx,gy;reg goal_checked=0,goal_reached=0;
 reg [7:0] input_bytes[0:383];integer length,i,j;
 mc_mecanum_protocol #(.NAVIGATION(1),.VERSION(3)) protocol(.clk(clk),.reset_n(rst),.byte_valid(byte_valid),.framing_error(1'b0),.byte_data(byte_data),.valid(cv),.body(body),.bad_frames(bad));
 mc_nav_mission mission(.clk(clk),.rst_n(rst),.now_ms(ms),.command_valid(cv),.body(body),.stop(stop),.armed(armed),.imu_fresh(1'b1),.map_fresh(1'b1),
 .invalidate_origin(1'b0),.goal_checked(goal_checked),.goal_reached(goal_reached),.planner_progress(1'b0),.task_active(active),.running(running),.origin(origin),.lease_pulse(lease),
 .status(status),.reason(reason),.mission_id(id),.waypoint_index(wi),.point_count(count),.from_x(),.from_y(),.goal_x(gx),.goal_y(gy),.waypoint_speed(),
 .ack_sequence(ack),.ack_error(error),.rejected(),.calibrated(calibrated),.front_mm(),.back_mm(),.half_width_mm(),.radius_mm(),.pad_mm(),
 .speed_mm_s(),.accel_mm_s2(),.forward_scale_q16(),.lateral_scale_q16(),.rotation_lever_mm(),.laser_x_mm(),.laser_y_mm(),.laser_yaw_q16());
 task tick;begin @(negedge clk);end endtask
 task send_packet(input integer k);begin
  for(j=0;j<24;j=j+1)begin byte_data=input_bytes[k*24+j];byte_valid=1;tick;byte_valid=0;tick;end
  repeat(12)tick;
 end endtask
 initial begin
  $readmemh("build/mecanum_navigation/mission.hex",input_bytes);
  repeat(3)tick;rst=1;
  for(i=0;i<11;i=i+1)send_packet(i);
  if(!calibrated || status!=2 || error!=0 || count!=2 || gx!=200 || gy!=0)$fatal(1,"upload/calibration failed status=%d err=%d count=%d",status,error,count);
  send_packet(11);if(error!=7 || active)$fatal(1,"unarmed start admitted");
  armed=1;send_packet(12);if(!active || !running)$fatal(1,"armed start rejected %d",error);
  goal_checked=1;goal_reached=1;tick;goal_checked=0;goal_reached=0;tick;
  if(wi!=1 || gx!=200 || gy!=200)$fatal(1,"point advance failed");
  ms=10;send_packet(13);ms=200;send_packet(13);ms=261;tick;tick;
  if(running || !active || reason!=2)$fatal(1,"duplicate heartbeat extended lease");
  send_packet(14);if(!running || wi!=1)$fatal(1,"resume lost progress");
  stop=1;tick;stop=0;tick;if(running || !active)$fatal(1,"STOP must pause retained mission");
  send_packet(15);if(active)$fatal(1,"cancel did not release owner");
  if(bad!=0)$fatal(1,"valid V4 frames rejected by byte protocol");
  $display("PASS V4 byte CRC/upload/commit/calibration, explicit ARM, per-point progress, duplicate lease refusal, pause/resume/cancel");$finish;
 end
 initial begin #10000000;$fatal(1,"mission FSM deadlock");end
endmodule
