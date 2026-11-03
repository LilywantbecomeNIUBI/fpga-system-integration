// V4 navigation transaction manager. V3 motor ARM/CLEAR remain explicit.
// No RAM-array reset: 16 point registers use a loaded-count and table CRC.
module mc_nav_mission #(parameter CLK_HZ=50000000)(
 input wire clk,rst_n,input wire [31:0] now_ms,input wire command_valid,input wire [175:0] body,
 input wire stop,invalidate_origin,input wire armed,imu_fresh,map_fresh,goal_checked,goal_reached,planner_progress,
 output reg task_active,running,origin,lease_pulse,output reg [3:0] status,reason,
 output reg [15:0] mission_id,output reg [4:0] waypoint_index,point_count,
 output wire signed [15:0] from_x,from_y,goal_x,goal_y,output wire [15:0] waypoint_speed,
 output reg [7:0] ack_sequence,ack_error,output reg [31:0] rejected,
 output reg calibrated,output reg [15:0] front_mm,back_mm,half_width_mm,radius_mm,pad_mm,speed_mm_s,accel_mm_s2,
 output reg [31:0] forward_scale_q16,lateral_scale_q16,
 output reg [15:0] rotation_lever_mm,
 output reg [31:0] self_start,self_end,self_distance,
 output reg signed [15:0] laser_x_mm,laser_y_mm,output reg signed [31:0] laser_yaw_q16
);
 reg signed [15:0] wx[0:15],wy[0:15];reg [15:0] ws[0:15];
 reg [4:0] loaded;reg committed,sequence_seen;reg [7:0] last_sequence;reg [175:0] last_body;
 reg [15:0] table_crc;reg [5:0] configured;
 reg [31:0] lease_ms,progress_ms,settle_ms;reg settling;
 reg [2:0] calibration_state;reg [31:0] r2,f2,b2,w2;reg geometry_ok;
 reg [79:0] crc_shift;reg [3:0] crc_left;
 wire [7:0] op=body[31:24],seq=body[39:32],index=body[87:80],count=body[95:88];
 wire [15:0] id=body[79:64],c=body[175:160];
 wire signed [31:0] a=body[127:96],b=body[159:128];
 wire [7:0] delta=seq-last_sequence;
 wire fresh=!sequence_seen || (delta!=0 && delta<128);
 assign from_x=waypoint_index==0?16'sd0:wx[waypoint_index-1'b1];
 assign from_y=waypoint_index==0?16'sd0:wy[waypoint_index-1'b1];
 assign goal_x=!committed?16'sd0:wx[waypoint_index[3:0]],goal_y=!committed?16'sd0:wy[waypoint_index[3:0]];
 assign waypoint_speed=!committed?16'd0:ws[waypoint_index[3:0]]<speed_mm_s?ws[waypoint_index[3:0]]:speed_mm_s;
 function automatic [15:0] crc_byte(input [15:0] old,input [7:0] v);
  reg [15:0] q;integer j;begin q=old^{v,8'd0};for(j=0;j<8;j=j+1)q=q[15]?(q<<1)^16'h1021:q<<1;crc_byte=q;end
 endfunction
 function automatic [15:0] crc_point(input [15:0] old,input [79:0] bytes_in);
  reg [15:0] q;integer j;begin q=old;for(j=0;j<10;j=j+1)q=crc_byte(q,bytes_in[j*8+:8]);crc_point=q;end
 endfunction
 task automatic reject(input [7:0] code);begin ack_error<=code;rejected<=rejected+1'b1;end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   task_active<=0;running<=0;origin<=0;lease_pulse<=0;status<=0;reason<=0;mission_id<=0;waypoint_index<=0;point_count<=0;
   ack_sequence<=0;ack_error<=0;rejected<=0;calibrated<=0;front_mm<=0;back_mm<=0;half_width_mm<=0;radius_mm<=0;pad_mm<=0;
   speed_mm_s<=100;accel_mm_s2<=100;forward_scale_q16<=0;lateral_scale_q16<=0;laser_x_mm<=0;laser_y_mm<=0;laser_yaw_q16<=0;
   loaded<=0;committed<=0;sequence_seen<=0;last_sequence<=0;last_body<=0;table_crc<=16'hffff;configured<=0;
   lease_ms<=0;progress_ms<=0;settle_ms<=0;settling<=0;
   rotation_lever_mm<=0;
   self_start<=0;self_end<=0;self_distance<=0;
   calibration_state<=0;r2<=0;f2<=0;b2<=0;w2<=0;geometry_ok<=0;
   crc_shift<=0;crc_left<=0;
  end else begin
   origin<=0;lease_pulse<=0;
   if(crc_left!=0)begin table_crc<=crc_byte(table_crc,crc_shift[7:0]);crc_shift<=crc_shift>>8;crc_left<=crc_left-1'b1;end
   case(calibration_state)
    1:begin r2<=radius_mm*radius_mm;calibration_state<=2;end
    2:begin f2<=front_mm*front_mm;calibration_state<=3;end
    3:begin b2<=back_mm*back_mm;calibration_state<=4;end
    4:begin w2<=half_width_mm*half_width_mm;calibration_state<=5;end
    5:begin geometry_ok<=r2>=f2+w2 && r2>=b2+w2;calibration_state<=6;end
    6:begin calibrated<=geometry_ok;if(!geometry_ok)reject(4);calibration_state<=0;end
    default:;
   endcase
   if(command_valid && body[23:16]==4)begin
    ack_sequence<=seq;
    if(!fresh)begin if(seq!=last_sequence || body!=last_body)reject(1);end
    else begin
     ack_error<=0;
     sequence_seen<=1;last_sequence<=seq;last_body<=body;
     if(crc_left!=0)reject(9);
     else if(body[63:40]!=0)reject(2);
     else case(op)
      8'h40:if(task_active || armed)reject(3);else begin
       calibrated<=0;calibration_state<=0;
       case(index)
        0:if(a>=50 && a<=600 && b>=50 && b<=600 && c>=30 && c<=300)begin
         front_mm<=a;back_mm<=b;half_width_mm<=c;configured<=1;self_start<=0;self_end<=0;self_distance<=0;
        end else reject(4);
        1:if(a>=50 && a<=800 && b>=56 && b<=400)begin radius_mm<=a;pad_mm<=b;configured[1]<=1;end else reject(4);
        2:if(a>0 && a<=65536 && b>0 && b<=65536)begin forward_scale_q16<=a;lateral_scale_q16<=b;configured[2]<=1;end else reject(4);
        3:if(a>=-500 && a<=500 && b>=-500 && b<=500 && $signed(c)>=-18000 && $signed(c)<=18000)begin
          laser_x_mm<=a;laser_y_mm<=b;laser_yaw_q16<=($signed(c)*64'sd11993645)>>>20;configured[3]<=1;end else reject(4);
        4:if(a>=1 && a<=150 && b>=20 && b<=1000)begin speed_mm_s<=a;accel_mm_s2<=b;configured[4]<=1;end else reject(4);
        5:if(a>=50 && a<=800)begin rotation_lever_mm<=a;configured[5]<=1;end else reject(4);
        6,7:if(a>=0 && a<36000 && b>=0 && b<36000 && c<=500)begin
         if(index==6)begin self_start[15:0]<=a;self_end[15:0]<=b;self_distance[15:0]<=c;end
         else begin self_start[31:16]<=a;self_end[31:16]<=b;self_distance[31:16]<=c;end
        end else reject(4);
        15:if(configured==63 && c==16'h4e41)calibration_state<=1;else reject(4);
        default:reject(4);
       endcase
      end
      8'h50:if(task_active || armed || count==0 || count>16)reject(3);else begin
       mission_id<=id;point_count<=count;loaded<=0;committed<=0;table_crc<=16'hffff;waypoint_index<=0;status<=1;
      end
      8'h51:if(task_active || armed || committed || id!=mission_id || index!=loaded || index>=point_count || a< -4000 || a>4000 || b< -4000 || b>4000 || c==0 || c>150 || c>speed_mm_s)reject(5);
       else begin wx[index[3:0]]<=a;wy[index[3:0]]<=b;ws[index[3:0]]<=c;loaded<=loaded+1'b1;crc_shift<=body[175:96];crc_left<=10;end
      8'h52:if(task_active || armed || id!=mission_id || loaded!=point_count || point_count==0 || c!=table_crc)reject(6);
       else begin committed<=1;status<=2;end
      8'h53:if(task_active || !committed || !calibrated || !armed || !imu_fresh || id!=mission_id)reject(7);
       else begin task_active<=1;running<=1;origin<=1;lease_pulse<=1;lease_ms<=now_ms;progress_ms<=now_ms;status<=3;reason<=0;waypoint_index<=0;settling<=0;end
      8'h54:if(id!=mission_id || !task_active)reject(8);else begin running<=0;status<=4;reason<=1;end
      8'h55:if(id!=mission_id || !task_active || running || !armed || !imu_fresh || !map_fresh)reject(8);
       else begin running<=1;lease_ms<=now_ms;progress_ms<=now_ms;lease_pulse<=1;status<=3;reason<=0;settling<=0;end
      8'h56:begin task_active<=0;running<=0;committed<=0;status<=0;reason<=0;settling<=0;end
      8'h57:if(id!=mission_id || !task_active || !running)reject(8);else begin lease_ms<=now_ms;lease_pulse<=1;end
      default:reject(2);
     endcase
    end
   end
   if(running)begin
    if(planner_progress)progress_ms<=now_ms;
    if(goal_checked && goal_reached)begin
     if(waypoint_index+1<point_count)begin waypoint_index<=waypoint_index+1'b1;progress_ms<=now_ms;end
     else if(!settling)begin settling<=1;settle_ms<=now_ms;end
     else if(now_ms-settle_ms>=300)begin running<=0;task_active<=0;status<=5;reason<=0;end
    end else if(goal_checked)settling<=0;
    if(now_ms-progress_ms>=5000)begin running<=0;status<=4;reason<=5;end
    if((!map_fresh && now_ms-progress_ms>=250) || !imu_fresh)begin running<=0;status<=4;reason<=4;end
    if(!armed || now_ms-lease_ms>=250)begin running<=0;status<=4;reason<=2;end
   end
   if(stop && task_active)begin running<=0;status<=4;if(running)reason<=1;end
   if(invalidate_origin && task_active)begin task_active<=0;running<=0;committed<=0;status<=0;reason<=4;settling<=0;end
  end
 end
endmodule
