// Entire navigation/control-intent computation runs on clk50m.
// Positions: mm Q16; heading: 0.01 degree Q16; wheel commands: counts/10ms.
module nav_core #(parameter CLK_HZ=50000000)(
 input wire clk,rst_n,input wire signed [31:0] left_position,right_position,
 input wire armed,input wire imu_valid,imu_bad,input wire signed [15:0] gx,gy,gz,
 input wire point_valid,input wire [15:0] point_angle,point_distance,input wire [7:0] point_intensity,
 input wire command_valid,input wire [7:0] opcode,sequence_id,
 input wire [15:0] arg0,arg1,arg2,arg3,
 output reg running,mission_done,output wire permit,output reg [15:0] faults,
 output reg update,output wire signed [15:0] left_target,right_target,
 output reg signed [31:0] x_q16,y_q16,theta_q16,
 output reg signed [15:0] v_mm_s,w_mrad_s,output reg [4:0] waypoint_index,
 output wire [15:0] front_min,output wire [71:0] blocked,
 output reg [31:0] rejected,output reg [31:0] plan_count);
 reg [31:0] cfg[0:31];reg signed [15:0] wx[0:15],wy[0:15];reg [15:0] wspeed[0:15];reg [15:0] wp_loaded;
 reg [31:0] divider,ms_div;reg [15:0] now_ms,imu_age,lease_age,lidar_age;reg tick,tick_ms;
 reg signed [15:0] gyro;
 reg [7:0] last_nav_sequence;reg sequence_seen;
 wire [7:0] sequence_delta=sequence_id-last_nav_sequence;
 wire fresh_sequence=!sequence_seen || (sequence_delta!=0 && sequence_delta<128);
 wire config_ok=cfg[0]==15 && cfg[1]>0 && cfg[1]<=65536 && cfg[2]>0 && cfg[2]<=65536 &&
  cfg[3]>=50 && cfg[3]<=2000 && cfg[4]>=20 && cfg[4]<=1500 && cfg[5]<=500 &&
  cfg[6]>cfg[4]+cfg[5] && cfg[6]<cfg[7] && cfg[7]<=8000 &&
  cfg[8]>0 && cfg[8]<=500 && cfg[9]>0 && cfg[9]<=1500 && cfg[8]<=cfg[9]*8 &&
  cfg[10]>=100 && cfg[10]<=cfg[11] && cfg[11]<=2000 && cfg[12]<=3000 &&
  cfg[13]>=20 && cfg[13]<=1000 && cfg[14]>=cfg[7] && cfg[14]<cfg[15] && cfg[15]<=8000 &&
  cfg[17]>=50 && cfg[17]<=1000 && cfg[18]>0 && cfg[18]<=65536 && cfg[19]>0 && cfg[19]<=65536 &&
  cfg[20]<=6 && cfg[20][1:0]!=3 && $signed(cfg[21])>=-32767 && $signed(cfg[21])<=32767 && cfg[22]>0 && cfg[22]<=10000 && cfg[23]<=256 &&
  $signed(cfg[24])>=-18000 && $signed(cfg[24])<=18000 && cfg[25]<=1 && cfg[26]<=cfg[8] &&
  cfg[27]>0 && cfg[27]<=16 && cfg[28]>=2 && cfg[28]<=16 && cfg[29]>0 && cfg[29]<=128 &&
  cfg[30]>=2 && cfg[30]<=100 && {1'b0,cfg[16][7:0]}>({1'b0,cfg[16][15:8]}+{1'b0,cfg[16][23:16]});
 wire waypoints_ok=(wp_loaded & ((17'd1<<cfg[27])-1))==((17'd1<<cfg[27])-1);
 wire [6:0] hist_index;wire [15:0] hist_distance;wire hist_known,hist_hazard,hist_overflow,hist_ready;
 wire [31:0] hazard_sequence,settled_sequence,clear_sequence;wire clear_hazard;
 nav_histogram histogram(.clk(clk),.rst_n(rst_n),.now_ms(now_ms),
  .point_valid(point_valid),.angle_q100(point_angle),.distance_mm(point_distance),.intensity(point_intensity),
  .angle_offset(cfg[24][15:0]),.angle_reverse(cfg[25][0]),.timeout_ms(cfg[17][15:0]),.stop_mm(cfg[6][15:0]),.min_intensity(8'd1),
  .read_index(hist_index),.read_distance(hist_distance),.read_known(hist_known),
  .clear_hazard(clear_hazard),.clear_sequence(clear_sequence),.hazard(hist_hazard),.hazard_sequence(hazard_sequence),.settled_sequence(settled_sequence),.overflow(hist_overflow),.ready(hist_ready));
 reg vfh_start,retry_turn;wire vfh_busy,vfh_done,vfh_found,front_known,rotation_clear;
 reg signed [15:0] target_heading,previous_heading;wire signed [15:0] safe_heading;
 reg [15:0] turn_radius;wire [23:0] vfh_cost;
 wire [15:0] safe_radius=cfg[4][15:0]+cfg[5][15:0];
 nav_vfh vfh(.clk(clk),.rst_n(rst_n),.start(vfh_start),.now_ms(now_ms),
  .target_heading(target_heading),.previous_heading(previous_heading),.safe_radius(safe_radius),
  .block_low(cfg[14][15:0]),.block_high(cfg[15][15:0]),.stop_mm(cfg[6][15:0]),.slow_mm(cfg[7][15:0]),.turn_radius(turn_radius),
  .w_target(cfg[16][7:0]),.w_current(cfg[16][15:8]),.w_previous(cfg[16][23:16]),.valley_width(cfg[28][7:0]),
  .read_index(hist_index),.read_distance(hist_distance),.read_known(hist_known),.hazard_sequence(settled_sequence),
  .clear_hazard(clear_hazard),.clear_sequence(clear_sequence),.busy(vfh_busy),.done(vfh_done),.found(vfh_found),
  .front_known(front_known),.rotation_clear(rotation_clear),.front_min(front_min),.safe_heading(safe_heading),.blocked(blocked),.best_cost(vfh_cost));
 reg math_start;reg [2:0] math_op;reg signed [63:0] math_a;reg signed [31:0] math_b;
 wire math_busy,math_done,math_error;wire signed [63:0] math_result;
 nav_math math(.clk(clk),.rst_n(rst_n),.start(math_start),.op(math_op),.a(math_a),.b(math_b),.busy(math_busy),.done(math_done),.error(math_error),.result(math_result));
 reg [7:0] state,return_state;reg signed [63:0] result,t0,t1;
 reg signed [31:0] prev_l,prev_r,dl,dr,dlq,drq,dsq,enc_turn,delta_turn;
 reg signed [31:0] cs,sn,segx,segy,goalx,goaly,dx,dy,carrotx,carroty,bx,by;
 reg signed [31:0] segment_length,along,projection,lookahead,goal_distance,carrot_squared;
 reg signed [31:0] segment_start_x,segment_start_y,previous_world_heading;
 reg signed [31:0] speed,turn_term,wheel_l,wheel_r,count_l,count_r,max_count;
 reg signed [15:0] out_l,out_r;
 reg [31:0] state_age;
 wire healthy=imu_age<cfg[30] && !imu_bad && !hist_overflow && lidar_age<cfg[17];
 assign permit=running && faults==0 && armed && healthy && lease_age<200 && !hist_hazard && front_known && vfh_found;
 assign left_target=permit?out_l:16'sd0,right_target=permit?out_r:16'sd0;
 function automatic signed [31:0] wrapq(input signed [63:0] a);
  if(a>=1179648000)wrapq=a-64'sd2359296000;else if(a< -1179648000)wrapq=a+64'sd2359296000;else wrapq=a;
 endfunction
 function automatic signed [15:0] wrapa(input signed [31:0] a);
  if(a>=18000)wrapa=a-36000;else if(a< -18000)wrapa=a+36000;else wrapa=a;
 endfunction
 function automatic signed [31:0] roundq(input signed [63:0] a);
  roundq=a<0 ? -(((-a)+32768)>>>16) : (a+32768)>>>16;
 endfunction
 function automatic signed [31:0] abs32(input signed [31:0] a);abs32=a<0?-a:a;endfunction
 task automatic calc(input [2:0] op,input signed [63:0] a,input signed [31:0] b,input [7:0] next_state);
  begin math_op<=op;math_a<=a;math_b<=b;math_start<=1;return_state<=next_state;state<=255;end
 endtask
 integer k;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   running<=0;mission_done<=0;faults<=0;update<=0;x_q16<=0;y_q16<=0;theta_q16<=0;v_mm_s<=0;w_mrad_s<=0;waypoint_index<=0;
   rejected<=0;plan_count<=0;wp_loaded<=0;divider<=0;ms_div<=0;now_ms<=0;imu_age<=65535;lease_age<=65535;lidar_age<=65535;tick<=0;tick_ms<=0;
   gyro<=0;last_nav_sequence<=0;sequence_seen<=0;vfh_start<=0;retry_turn<=0;target_heading<=0;previous_heading<=0;turn_radius<=0;
   math_start<=0;math_op<=0;math_a<=0;math_b<=0;state<=0;return_state<=0;result<=0;t0<=0;t1<=0;prev_l<=0;prev_r<=0;
   dl<=0;dr<=0;dlq<=0;drq<=0;dsq<=0;enc_turn<=0;delta_turn<=0;cs<=65536;sn<=0;segx<=0;segy<=0;goalx<=0;goaly<=0;
   dx<=0;dy<=0;carrotx<=0;carroty<=0;bx<=0;by<=0;segment_length<=0;along<=0;projection<=0;lookahead<=0;goal_distance<=0;carrot_squared<=0;
   segment_start_x<=0;segment_start_y<=0;previous_world_heading<=0;speed<=0;turn_term<=0;wheel_l<=0;wheel_r<=0;count_l<=0;count_r<=0;max_count<=0;
   out_l<=0;out_r<=0;state_age<=0;
   for(k=0;k<32;k=k+1)cfg[k]<=0;
   cfg[6]<=450;cfg[7]<=1000;cfg[8]<=120;cfg[9]<=400;cfg[10]<=300;cfg[11]<=800;cfg[12]<=1000;cfg[13]<=120;
   cfg[14]<=1200;cfg[15]<=1400;cfg[16]<=32'h00030105;cfg[17]<=200;cfg[22]<=2000;cfg[23]<=192;cfg[26]<=30;cfg[28]<=8;cfg[29]<=32;cfg[30]<=20;
  end else begin
   update<=0;tick<=0;tick_ms<=0;math_start<=0;vfh_start<=0;
   if(divider==CLK_HZ/100-1)begin divider<=0;tick<=1;end else divider<=divider+1'b1;
   if(ms_div==CLK_HZ/1000-1)begin ms_div<=0;now_ms<=now_ms+1'b1;tick_ms<=1;end else ms_div<=ms_div+1'b1;
   if(tick_ms)begin if(imu_age<65535)imu_age<=imu_age+1'b1;if(lease_age<65535)lease_age<=lease_age+1'b1;if(lidar_age<65535)lidar_age<=lidar_age+1'b1;end
   if(point_valid)lidar_age<=0;
   if(imu_valid)begin imu_age<=0;case(cfg[20][1:0])0:gyro<=gx;1:gyro<=gy;2:gyro<=gz;default:gyro<=0;endcase end
   if(state!=0)state_age<=state_age+1'b1;else state_age<=0;
   if(!running)begin state<=0;out_l<=0;out_r<=0;v_mm_s<=0;w_mrad_s<=0;end
   else case(state)
    0:begin if(tick && !vfh_busy && !math_busy)begin
 dl<=left_position-prev_l;dr<=right_position-prev_r;prev_l<=left_position;prev_r<=right_position;state<=1;
 end end
    1:begin if(abs32(dl)>2048 || abs32(dr)>2048)begin faults[2]<=1;running<=0;end
 else calc(0,dl,$signed(cfg[1]),2); end
    2:begin dlq<=result;calc(0,dr,$signed(cfg[2]),3); end
    3:begin drq<=result;state<=4; end
    4:begin dsq<=($signed(dlq)+$signed(drq))>>>1;calc(0,$signed(drq)-$signed(dlq),572958,5); end
    5:begin calc(1,result,$signed(cfg[3])*100,6); end
    6:begin if(result>327680000 || result< -327680000)begin faults[2]<=1;running<=0;end else begin enc_turn<=result;calc(0,cfg[20][2]?-$signed(gyro)+$signed(cfg[21]):$signed(gyro)-$signed(cfg[21]),$signed(cfg[22]),7);end end
    7:begin calc(0,$signed(result[31:0])-enc_turn,$signed(cfg[23]),8); end
    8:begin delta_turn<=enc_turn+(result>>>8);state<=9; end
    9:begin if(abs32(delta_turn)>327680000)begin faults[2]<=1;running<=0;end
 else begin theta_q16<=wrapq($signed(theta_q16)+$signed(delta_turn));calc(4,wrapq($signed(theta_q16)+($signed(delta_turn)>>>1))>>>16,0,10);end end
    10:begin cs<=result[63:32];sn<=result[31:0];state<=11; end
    11:begin calc(0,dsq,cs,12); end
    12:begin t0<=$signed(x_q16)+(result>>>16);calc(0,dsq,sn,13); end
    13:begin t1<=$signed(y_q16)+(result>>>16);state<=14; end
    14:begin if(t0>1048576000 || t0< -1048576000 || t1>1048576000 || t1< -1048576000)begin faults[7]<=1;running<=0;end
 else begin x_q16<=t0;y_q16<=t1;goalx<=wx[waypoint_index];goaly<=wy[waypoint_index];state<=15;end end
    15:begin dx<=goalx-($signed(x_q16)>>>16);dy<=goaly-($signed(y_q16)>>>16);segx<=goalx-segment_start_x;segy<=goaly-segment_start_y;state<=16; end
    16:begin calc(0,dx,dx,17); end
    17:begin t0<=result;calc(0,dy,dy,18); end
    18:begin calc(2,result+t0,0,19); end
    19:begin goal_distance<=result;
 if(result<$signed(cfg[13]))begin
  segment_start_x<=goalx;segment_start_y<=goaly;out_l<=0;out_r<=0;update<=1;state<=0;
  if(waypoint_index+1>=cfg[27])begin running<=0;mission_done<=1;end else waypoint_index<=waypoint_index+1'b1;
 end else calc(0,segx,segx,20); end
    20:begin t0<=result;calc(0,segy,segy,21); end
    21:begin calc(2,result+t0,0,22); end
    22:begin segment_length<=result;
 if(result==0)begin faults[0]<=1;running<=0;end
 else calc(0,($signed(x_q16)>>>16)-segment_start_x,segx,23); end
    23:begin t0<=result;calc(0,($signed(y_q16)>>>16)-segment_start_y,segy,24); end
    24:begin calc(1,result+t0,segment_length,25); end
    25:begin projection<=result<0?0:result>segment_length?segment_length:result;calc(0,abs32(dsq)>>>6,$signed(cfg[12]),26); end
    26:begin calc(1,result,10240,27); end
    27:begin lookahead<=result<$signed(cfg[10])?$signed(cfg[10]):result>$signed(cfg[11])?$signed(cfg[11]):result;state<=28; end
    28:begin along<=projection+lookahead>segment_length?segment_length:projection+lookahead;state<=29; end
    29:begin calc(0,segx,along,30); end
    30:begin calc(1,result,segment_length,31); end
    31:begin carrotx<=segment_start_x+result-($signed(x_q16)>>>16);calc(0,segy,along,32); end
    32:begin calc(1,result,segment_length,33); end
    33:begin carroty<=segment_start_y+result-($signed(y_q16)>>>16);calc(4,$signed(theta_q16)>>>16,0,34); end
    34:begin cs<=result[63:32];sn<=result[31:0];state<=35; end
    35:begin calc(0,carrotx,cs,36); end
    36:begin t0<=result;calc(0,carroty,sn,37); end
    37:begin bx<=(t0+result)>>>16;calc(0,-carrotx,sn,38); end
    38:begin t0<=result;calc(0,carroty,cs,39); end
    39:begin by<=(t0+result)>>>16;state<=40; end
    40:begin calc(3,bx,by,41); end
    41:begin target_heading<=result;previous_heading<=wrapa(previous_world_heading-($signed(theta_q16)>>>16));calc(0,bx,bx,42); end
    42:begin t0<=result;calc(0,by,by,43); end
    43:begin carrot_squared<=t0+result;
 speed<=wspeed[waypoint_index] < cfg[8] ? wspeed[waypoint_index] : cfg[8];state<=44; end
    44:begin if(carrot_squared<=0)begin faults[4]<=1;running<=0;end
 else if(by==0)state<=47;
 else calc(0,carrot_squared,$signed(cfg[9]),45); end
    45:begin calc(1,result,abs32(by)*2000,46); end
    46:begin if(result<speed)speed<=result;state<=47; end
    47:begin calc(0,goal_distance,$signed(cfg[8]),48); end
    48:begin calc(1,result,$signed(cfg[10])*2,49); end
    49:begin if(result<speed)speed<=result;state<=50; end
    50:begin if(speed<$signed(cfg[26]))speed<=cfg[26];state<=51; end
    51:begin calc(0,speed,1000,52); end
    52:begin calc(1,result,$signed(cfg[9]),53); end
    53:begin turn_radius<=result>8000?8000:result;retry_turn<=0;vfh_start<=1;state<=54; end
    54:begin if(vfh_done)begin
 if(!vfh_found && turn_radius!=0 && !retry_turn)begin turn_radius<=0;retry_turn<=1;vfh_start<=1;end
 else if(!vfh_found || !front_known || front_min<=cfg[6])begin speed<=0;state<=58;end
 else if(retry_turn)begin speed<=0;state<=55;end
 else if(front_min<cfg[7])begin speed<=speed>>>1;state<=55;end else state<=55;
 end end
    55:begin if(retry_turn || abs32($signed(safe_heading))>4500)begin speed<=0;if(!rotation_clear)begin v_mm_s<=0;w_mrad_s<=0;state<=65;end else state<=58;end
 else state<=58; end
    58:begin v_mm_s<=speed;
 if(!vfh_found || !front_known || front_min<=cfg[6])begin w_mrad_s<=0;state<=65;end
 else if(abs32(wrapa($signed(safe_heading)-$signed(target_heading)))<=250 && speed>0)calc(0,by*2000,speed,59);
 else calc(0,safe_heading,349,61); end
    59:begin calc(1,result,carrot_squared,60); end
    60:begin w_mrad_s<=result>$signed(cfg[9])?$signed(cfg[9]):result< -$signed(cfg[9])?-$signed(cfg[9]):result;state<=65; end
    61:begin calc(1,result,1000,60); end
    65:begin calc(0,w_mrad_s,$signed(cfg[3]),66); end
    66:begin calc(1,result,2000,67); end
    67:begin wheel_l<=$signed(v_mm_s)-result;wheel_r<=$signed(v_mm_s)+result;state<=68; end
    68:begin calc(0,wheel_l,$signed(cfg[18]),69); end
    69:begin count_l<=roundq(result);calc(0,wheel_r,$signed(cfg[19]),70); end
    70:begin count_r<=roundq(result);state<=71; end
    71:begin max_count<=abs32(count_l)>abs32(count_r)?abs32(count_l):abs32(count_r);state<=72; end
    72:begin if(max_count>$signed(cfg[29]))calc(0,count_l,$signed(cfg[29]),73);else state<=78; end
    73:begin calc(1,result,max_count,74); end
    74:begin count_l<=result;calc(0,count_r,$signed(cfg[29]),75); end
    75:begin calc(1,result,max_count,76); end
    76:begin count_r<=result;state<=78; end
    78:begin out_l<=count_l;out_r<=count_r;previous_world_heading<=wrapa(($signed(theta_q16)>>>16)+$signed(safe_heading));
 update<=1;plan_count<=plan_count+1'b1;state<=0; end
    255:if(math_done)begin
     if(math_error)begin faults[4]<=1;running<=0;state<=0;end
     else begin result<=math_result;state<=return_state;end
    end
    default:begin faults[4]<=1;running<=0;state<=0;end
   endcase
   // UART STOP/cancel and loss of health dominate every calculation/commit.
   if(command_valid && opcode>=8'h40 && opcode<=8'h44)begin
    if(!fresh_sequence && opcode!=8'h44)rejected<=rejected+1'b1;
    else begin
     last_nav_sequence<=sequence_id;sequence_seen<=1;
     case(opcode)
      8'h40:if(!running && !armed && arg0<32)cfg[arg0[4:0]]<={arg2,arg1};else rejected<=rejected+1'b1;
      8'h41:if(!running && !armed && arg0<16 && $signed(arg1)>=-16000 && $signed(arg1)<=16000 &&
       $signed(arg2)>=-16000 && $signed(arg2)<=16000 && arg3>0 && arg3<=500)begin
        wx[arg0[3:0]]<=arg1;wy[arg0[3:0]]<=arg2;wspeed[arg0[3:0]]<=arg3;wp_loaded[arg0[3:0]]<=1;
       end else rejected<=rejected+1'b1;
      8'h42:if(!running && armed && config_ok && waypoints_ok && healthy && hist_ready && !math_busy && !vfh_busy)begin
        running<=1;mission_done<=0;faults<=0;state<=0;lease_age<=0;waypoint_index<=0;
        prev_l<=left_position;prev_r<=right_position;x_q16<=0;y_q16<=0;theta_q16<=0;
        segment_start_x<=0;segment_start_y<=0;previous_world_heading<=0;out_l<=0;out_r<=0;
       end else begin rejected<=rejected+1'b1;faults[0]<=1;end
      8'h43:if(running && lease_age<200)lease_age<=0;else rejected<=rejected+1'b1;
      8'h44:begin running<=0;out_l<=0;out_r<=0;end
     endcase
    end
   end
   if(command_valid && (opcode==0 || (opcode==2 && running)))begin running<=0;out_l<=0;out_r<=0;end
   if(running && (lease_age>=199 && tick_ms))begin running<=0;faults[5]<=1;end
   if(running && (!armed || !healthy))begin running<=0;if(!armed)faults[6]<=1;if(!healthy)faults[1]<=1;end
   if(running && state_age>=CLK_HZ/50)begin running<=0;faults[4]<=1;end
  end
 end
endmodule
