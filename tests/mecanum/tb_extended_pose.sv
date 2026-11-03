`timescale 1ns/1ps
module tb_extended_pose;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,new_goal=0,continuous=1;
 reg signed [31:0] yaw=0,angle=0;reg signed [15:0] gx=0,gy=0;
 reg [127:0] pos=0;wire signed [15:0] x,y,z;wire valid,arrived,overflow;wire signed [31:0] px,py,heading;
 mc_mecanum_pose_control #(.EXTENDED_TURNS(1),.ANGLE_SPEED_SHIFT(13)) dut(
 .clk(clk),.reset_n(rst),.tick(tick),.active(1'b1),.new_goal(new_goal),.mode(2'd2),.yaw_ok(1'b1),.yaw_q16(yaw),
 .wheel_positions(pos),.wheel_velocities(128'd0),.goal_x(gx),.goal_y(gy),.goal_centideg(angle),.continuous_turn(continuous),
 .speed_limit(16'd6),.position_tolerance(16'd4),.command_x(x),.command_y(y),.command_z(z),.command_valid(valid),
 .arrived(arrived),.overflow(overflow),.x_q16(px),.y_q16(py),.heading_q16(heading));
 task goal;begin @(negedge clk);new_goal=1;@(negedge clk);new_goal=0;repeat(4)@(negedge clk);end endtask
 task update;begin @(negedge clk);tick=1;@(negedge clk);tick=0;wait(valid);@(negedge clk);if(overflow)$fatal(1,"unexpected position overflow");end endtask
 function automatic integer yaw_word(input real degrees);
 real wrapped;begin wrapped=degrees;while(wrapped>180)wrapped=wrapped-360;while(wrapped< -180)wrapped=wrapped+360;
 yaw_word=$rtoi(wrapped*3.141592653589793/180*65536);end endfunction
 real physical,origin,target;integer i,fl=0,fr=0,rl=0,rr=0;
 task turn(input integer centideg,input real start_deg);begin
 gx=0;gy=0;angle=centideg;continuous=1;physical=start_deg;origin=start_deg;target=centideg/100.0;yaw=yaw_word(physical);goal();update();
 if((centideg>0 && z<=0)||(centideg<0 && z>=0))$fatal(1,"wrong full-turn start direction");
 for(i=0;i<6000 && !arrived;i=i+1)begin
 physical=physical+z*0.3;yaw=yaw_word(physical);update();
 if(arrived && ((physical-origin-target)>1.2 || (physical-origin-target)< -1.2))$fatal(1,"premature full-turn arrival");
 end
 if(!arrived || z!=0)$fatal(1,"full turn did not settle angle=%d heading=%d",centideg,heading);
 $display("PASS continuous turn %0d centidegrees, crossed IMU wrap, measured %0.2f degrees",centideg,physical-origin);
 end endtask
 task translate(input integer target_x,target_y);begin
 continuous=0;angle=0;gx=target_x;gy=target_y;goal();update();
 for(i=0;i<6000 && !arrived;i=i+1)begin
 fl=fl+x-y;fr=fr+x+y;rl=rl+x+y;rr=rr+x-y;
 pos={32'(rr),32'(rl),32'(fr),32'(fl)};update();
 end
 if(!arrived || $signed(px)<(target_x-5)*65536 || $signed(px)>(target_x+5)*65536 || $signed(py)<(target_y-5)*65536 || $signed(py)>(target_y+5)*65536)
 $fatal(1,"long translation failed %d %d pos=%d/%d",target_x,target_y,px,py);
 $display("PASS long translation %0d/%0d control units, no wrap/overflow",target_x,target_y);
 end endtask
 initial begin
 repeat(4)@(negedge clk);rst=1;
 turn(36000,170.0);turn(-36000,-170.0);turn(72000,150.0);turn(-72000,-150.0);
 translate(13926,0);translate(9847,9847);translate(-13926,0);translate(15915,0);
 $display("PASS extended pose: signed 360/720 full turns and 3.5m straight/diagonal, 4m boundary");$finish;
 end
 initial begin #30000000;$fatal(1,"extended pose timeout");end
endmodule
