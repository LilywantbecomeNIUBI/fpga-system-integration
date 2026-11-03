// Mecanum navigation kernel: all intent computation is on the real clk50m.
module mc_nav_system_pre_serial #(parameter CLK_HZ=50000000,CONTROL_SHIFT=6)(
 input wire clk,rst_n,command_valid,input wire [175:0] command_body,
 input wire armed,input wire [127:0] positions,velocities,
 input wire yaw_ok,imu_stale,imu_fault,input wire signed [31:0] yaw_q16,
 input wire [31:0] imu_now_us,imu_sample_us,
 input wire point_valid,input wire [15:0] point_angle,point_distance,input wire [7:0] point_intensity,input wire [31:0] point_errors,
 input wire flow_valid,input wire [63:0] flow_snapshot,
 output wire nav_active,output wire nav_permit,output wire nav_lease,
 output reg signed [15:0] nav_x,nav_y,nav_z,
 output reg frame_valid,output reg [383:0] frame,input wire frame_ready
);
 integer ms_div,odom_div,plan_div,status_div;reg [31:0] now_ms;reg odom_tick,plan_tick,status_tick;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin ms_div<=0;odom_div<=0;plan_div<=0;status_div<=0;now_ms<=0;odom_tick<=0;plan_tick<=0;status_tick<=0;end
  else begin
   odom_tick<=0;plan_tick<=0;status_tick<=0;
   if(ms_div==CLK_HZ/1000-1)begin ms_div<=0;now_ms<=now_ms+1'b1;end else ms_div<=ms_div+1;
   if(odom_div==CLK_HZ/100-1)begin odom_div<=0;odom_tick<=1;end else odom_div<=odom_div+1;
   if(plan_div==CLK_HZ/20-1)begin plan_div<=0;plan_tick<=1;end else plan_div<=plan_div+1;
   if(status_div==CLK_HZ/10-1)begin status_div<=0;status_tick<=1;end else status_div<=status_div+1;
  end
 end
 wire running,origin,calibrated;wire [3:0] task_status,reason;wire [15:0] mission_id;
 wire [4:0] waypoint_index,point_count;wire signed [15:0] from_x,from_y,goal_x,goal_y,laser_x,laser_y;
 wire [15:0] waypoint_speed,front,back,width,radius,pad,max_speed,accel,rotation_lever;
 wire [31:0] forward_scale,lateral_scale,rejected;wire signed [31:0] laser_yaw;
 wire [31:0] self_start,self_end,self_distance;
 wire [7:0] ack_seq,ack_error;
 wire imu_fresh=yaw_ok && !imu_stale && !imu_fault && $unsigned(imu_now_us-imu_sample_us)<=100000;
 wire map_fresh;reg goal_checked,goal_reached,progress;
 mc_nav_mission #(.CLK_HZ(CLK_HZ)) mission(.clk(clk),.rst_n(rst_n),.now_ms(now_ms),.command_valid(command_valid && command_body[23:16]==4),.body(command_body),
  .stop(command_valid && command_body[23:16]==3 && command_body[31:24]==0),.armed(armed),.imu_fresh(imu_fresh),.map_fresh(map_fresh),
  .invalidate_origin(command_valid && command_body[23:16]==3 && command_body[31:24]==8),
  .goal_checked(goal_checked),.goal_reached(goal_reached),.planner_progress(progress),.task_active(nav_active),.running(running),.origin(origin),.lease_pulse(nav_lease),
  .status(task_status),.reason(reason),.mission_id(mission_id),.waypoint_index(waypoint_index),.point_count(point_count),
  .from_x(from_x),.from_y(from_y),.goal_x(goal_x),.goal_y(goal_y),.waypoint_speed(waypoint_speed),.ack_sequence(ack_seq),.ack_error(ack_error),.rejected(rejected),
  .calibrated(calibrated),.front_mm(front),.back_mm(back),.half_width_mm(width),.radius_mm(radius),.pad_mm(pad),.speed_mm_s(max_speed),.accel_mm_s2(accel),
  .forward_scale_q16(forward_scale),.lateral_scale_q16(lateral_scale),.rotation_lever_mm(rotation_lever),.self_start(self_start),.self_end(self_end),.self_distance(self_distance),.laser_x_mm(laser_x),.laser_y_mm(laser_y),.laser_yaw_q16(laser_yaw));
 wire signed [31:0] px_q16,py_q16,heading;wire odom_valid,odom_overflow;
 wire signed [15:0] actual_x,actual_y;
 mc_nav_odometry odometry(.clk(clk),.rst_n(rst_n),.tick(odom_tick),.origin(origin),.active(nav_active),.positions(positions),.yaw_q16(yaw_q16),
  .forward_scale_q16(forward_scale),.lateral_scale_q16(lateral_scale),.x_q16(px_q16),.y_q16(py_q16),.heading_q16(heading),
  .velocity_x_mm_s(actual_x),.velocity_y_mm_s(actual_y),.valid(odom_valid),.overflow(odom_overflow));
 // Reset map ownership on a new mission; memory is still cleared by its FSM.
 // A pose freshness fault invalidates both ownership and queued scan poses.
 // Recovery must build a complete new scan before explicit RESUME is allowed.
 wire map_reset_n=rst_n && !origin && imu_fresh && !odom_overflow;
 wire scan_begin,scan_abort,publish,cell_valid,cell_ready,cell_done,initializing,scan_bad;
 wire [13:0] cell_address;wire [1:0] cell_code;
 wire [31:0] map_epoch,map_ms,metadata_epoch,ray_overflows,last_point_ms;
 wire signed [31:0] working_x,working_y,working_angle,map_x,map_y,map_heading;
 wire [7:0] dangers;
 mc_nav_raymap raymap(.clk(clk),.rst_n(map_reset_n),.enable(calibrated),.now_ms(now_ms),
  .point_valid(point_valid),.angle_q100(point_angle),.distance_mm(point_distance),.intensity(point_intensity),.error_count(point_errors),
  .pose_x_q16(px_q16),.pose_y_q16(py_q16),.heading_q16(heading),.laser_x_mm(laser_x),.laser_y_mm(laser_y),.laser_yaw_q16(laser_yaw),
  .self_start(self_start),.self_end(self_end),.self_distance(self_distance),
  .front_mm(front),.back_mm(back),.half_width_mm(width),.stop_margin_mm(pad+16'd100),.grid_ready(cell_ready),.cell_done(cell_done),.grid_epoch(map_epoch),.initializing(initializing),
  .scan_begin(scan_begin),.scan_abort(scan_abort),.publish(publish),.cell_valid(cell_valid),.cell_address(cell_address),.cell_code(cell_code),
  .anchor_x_mm(working_x),.anchor_y_mm(working_y),.anchor_heading(working_angle),.published_x_mm(map_x),.published_y_mm(map_y),.published_heading(map_heading),
  .metadata_epoch(metadata_epoch),.overflow_count(ray_overflows),.danger_mask(dangers),.last_point_ms(last_point_ms));
 wire planner_lock,grid_valid,read_valid,read_ready,read_done;wire [13:0] read_address;wire [1:0] read_code;
 mc_nav_grid grid(.clk(clk),.rst_n(map_reset_n),.now_ms(now_ms),.scan_begin(scan_begin),.scan_abort(scan_abort),.publish(publish),.planner_lock(planner_lock),
  .cell_valid(cell_valid),.cell_address(cell_address),.cell_code(cell_code),.cell_ready(cell_ready),.cell_done(cell_done),.read_valid(read_valid),
  .read_address(read_address),.read_ready(read_ready),.read_done(read_done),.read_code(read_code),.map_valid(grid_valid),.epoch(map_epoch),.map_ms(map_ms),
  .foreground(),.initializing(initializing),.scan_bad(scan_bad));
 assign map_fresh=grid_valid && metadata_epoch==map_epoch && now_ms-last_point_ms<200;
 reg [31:0] flow_ms,flow_offset;reg [15:0] flow_id;reg flow_seen,flow_good,flow_risk;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin flow_ms<=0;flow_offset<=0;flow_id<=0;flow_seen<=0;flow_good<=0;flow_risk<=0;end
  else if(flow_valid)begin
   if(!flow_snapshot[0])flow_good<=0;
   else if(!flow_seen || ((flow_snapshot[19:4]-flow_id!=0 && $unsigned(flow_snapshot[19:4]-flow_id)<32768) && $unsigned(now_ms-(flow_snapshot[63:32]+flow_offset))<100))begin
    if(!flow_seen)flow_offset<=now_ms-flow_snapshot[63:32];
    flow_ms<=now_ms;flow_id<=flow_snapshot[19:4];flow_seen<=1;flow_good<=1;flow_risk<=|flow_snapshot[3:1];
   end
  end
 end
 wire flow_degraded=!flow_good || now_ms-flow_ms>=100;
 wire active_flow_risk=flow_good && now_ms-flow_ms<100 && flow_risk;
 wire [6:0] hist_vfh_index;reg [6:0] hist_probe_index;wire [15:0] hist_distance;wire hist_known;
 wire hist_hazard,hist_overflow,hist_ready;wire [31:0] hazard_epoch,settled_epoch,clear_epoch;wire clear_hazard;
 wire vfh_busy,vfh_done,vfh_found,vfh_front_known,vfh_rotation;reg vfh_start;
 wire [15:0] front_min;wire signed [15:0] vfh_heading;reg signed [15:0] desired_heading;
 wire [71:0] blocked;wire [23:0] vfh_cost;
 reg [15:0] coarse_min;reg coarse_known;reg [6:0] target_probe_index;
 wire [15:0] laser_offset_bound=(laser_x<0?-$signed(laser_x):laser_x)+(laser_y<0?-$signed(laser_y):laser_y);
 wire [15:0] coarse_clearance=coarse_known && coarse_min>laser_offset_bound?coarse_min-laser_offset_bound:16'd0;
 nav_histogram histogram(.clk(clk),.rst_n(map_reset_n),.now_ms(now_ms[15:0]),.point_valid(point_valid),.angle_q100(point_angle),
  .distance_mm(point_distance),.intensity(point_intensity),.angle_offset(16'sd0),.angle_reverse(1'b0),.timeout_ms(16'd200),.stop_mm(16'd1),.min_intensity(8'd1),
  .read_index(vfh_busy?hist_vfh_index:hist_probe_index),.read_distance(hist_distance),.read_known(hist_known),.clear_hazard(clear_hazard),.clear_sequence(clear_epoch),
  .hazard(hist_hazard),.hazard_sequence(hazard_epoch),.settled_sequence(settled_epoch),.overflow(hist_overflow),.ready(hist_ready));
 nav_vfh advisory(.clk(clk),.rst_n(map_reset_n),.start(vfh_start),.now_ms(now_ms[15:0]),.target_heading(desired_heading),.previous_heading(desired_heading),
  .safe_radius(radius+pad),.block_low(16'd1200),.block_high(16'd1400),.stop_mm(16'd1),.slow_mm(16'd500),.turn_radius(16'd0),
  .w_target(8'd5),.w_current(8'd1),.w_previous(8'd3),.valley_width(8'd8),.read_index(hist_vfh_index),.read_distance(hist_distance),.read_known(hist_known),
  .hazard_sequence(settled_epoch),.clear_hazard(clear_hazard),.clear_sequence(clear_epoch),.busy(vfh_busy),.done(vfh_done),.found(vfh_found),
  .front_known(vfh_front_known),.rotation_clear(vfh_rotation),.front_min(front_min),.safe_heading(vfh_heading),.blocked(blocked),.best_cost(vfh_cost));
 reg path_start;wire path_busy,path_done,path_error;wire signed [31:0] target_x,target_y;wire [15:0] goal_distance;
 mc_nav_path_target path(.clk(clk),.rst_n(rst_n),.start(path_start),.from_x(from_x),.from_y(from_y),.goal_x(goal_x),.goal_y(goal_y),
  .pose_x(px_q16[31:16]),.pose_y(py_q16[31:16]),.lookahead_mm(16'd300),.busy(path_busy),.done(path_done),.error(path_error),.target_x(target_x),.target_y(target_y),.distance_mm(goal_distance));
 reg planner_start;wire planner_busy,planner_done,planner_found,search_limited,planner_timeout,unknown_motion;wire signed [15:0] planned_x,planned_y;wire [31:0] planner_cycles;
 reg signed [31:0] local_x,local_y,local_tx,local_ty,relative_heading;
 reg signed [15:0] previous_vx,previous_vy,measured_map_vx,measured_map_vy;
 reg path_occluded,occlusion_seen;reg [1:0] clear_plans;reg [31:0] rejoin_ms;
 reg signed [31:0] path_cs,path_sn;reg [8:0] rejoin_weight;
 reg signed [31:0] body_cs,body_sn;
 wire signed [31:0] omega_step_full=$signed(omega)/32'sd10;
 mc_nav_planner #(.MAX_CYCLES(CLK_HZ/20>20000?CLK_HZ/20-20000:CLK_HZ/25)) planner(.clk(clk),.rst_n(rst_n),.start(planner_start),.abort(!running || !map_fresh || !imu_fresh),.map_epoch(map_epoch),
  .x_mm(local_x),.y_mm(local_y),.theta_q16(relative_heading),.target_x_mm(local_tx),.target_y_mm(local_ty),.current_vx(measured_map_vx),.current_vy(measured_map_vy),
  .omega_step_q16(omega_step_full[15:0]),.direction_cos_q16(body_cs),.direction_sin_q16(body_sn),.speed_mm_s(waypoint_speed),.accel_mm_s2(accel),.front_mm(front),.back_mm(back),.half_width_mm(width),.radius_mm(radius),.pad_mm(pad),
  .coarse_clearance_mm(coarse_clearance),
  .path_cos_q16(path_cs),.path_sin_q16(path_sn),.path_weight_q8(rejoin_weight),
  .danger_mask(dangers),.flow_risk(active_flow_risk),.flow_degraded(flow_degraded),.path_occluded(path_occluded || clear_plans<3),
  .allow_zero_translation(goal_distance<=100),
  .suggested_heading_q100(vfh_heading),.busy(planner_busy),.done(planner_done),.found(planner_found),.search_limited(search_limited),.timed_out(planner_timeout),
  .unknown_motion(unknown_motion),.vx_mm_s(planned_x),.vy_mm_s(planned_y),.cycles(planner_cycles),.planner_lock(planner_lock),
  .read_valid(read_valid),.read_address(read_address),.read_ready(read_ready),.read_done(read_done),.read_code(read_code));
 // Serialized coordinate transform / unit conversion. Existing nav_math is
 // reused for bounded division and target bearing; never use combinational /.
 reg [5:0] state;reg trig_start;wire trig_done,trig_busy;wire signed [31:0] cosine,sine;
 reg signed [31:0] trig_angle,cs,sn;
 reg signed [63:0] p0,p1,p2,p3;reg signed [31:0] body_vx,body_vy,omega,angular_mm_s;
 reg math_start;reg [2:0] math_op;reg signed [63:0] math_a;reg signed [31:0] math_b;
 wire math_busy,math_done,math_error;wire signed [63:0] math_result;
 reg [31:0] command_ms,plan_epoch,command_epoch;reg [15:0] best_distance;reg plan_valid;
 reg [4:0] previous_waypoint;
 mc_cordic_sincos coordinate_trig(.clk(clk),.rst_n(rst_n),.start(trig_start),.angle_q16(trig_angle),.busy(trig_busy),.done(trig_done),.cos_q16(cosine),.sin_q16(sine));
 nav_math scalar(.clk(clk),.rst_n(rst_n),.start(math_start),.op(math_op),.a(math_a),.b(math_b),.busy(math_busy),.done(math_done),.error(math_error),.result(math_result));
 wire heading_safe=heading<=17157 && heading>=-17157;
 wire feedback_still=velocities==0;
 wire global_permit=running && armed && calibrated && map_fresh && imu_fresh && !odom_overflow && !hist_overflow &&
  heading_safe && plan_valid && map_epoch==command_epoch && now_ms-command_ms<100;
 mc_nav_direction_gate final_gate(.clk(clk),.rst_n(rst_n),.global_permit(global_permit),.danger_mask(dangers),
  .forward_flow_risk(active_flow_risk),.rotation_requested(nav_z!=0),.target_x(body_vx[15:0]),.target_y(body_vy[15:0]),
  .actual_x(actual_x),.actual_y(actual_y),.permit(nav_permit),.blocked_mask());
 function automatic signed [31:0] wrap(input signed [31:0] a);if(a>205887)wrap=a-411775;else if(a< -205887)wrap=a+411775;else wrap=a;endfunction
 function automatic signed [15:0] limit_counts(input signed [63:0] v);if(v>32767)limit_counts=32767;else if(v< -32767)limit_counts=-32767;else limit_counts=v;endfunction
 task automatic divide(input signed [63:0] numerator,input [31:0] scale,input [5:0] next_state);
  begin math_op<=1;math_a<=numerator;math_b<=(scale<<12)+(scale<<11)+(scale<<8);math_start<=1;state<=next_state;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=0;path_start<=0;planner_start<=0;vfh_start<=0;trig_start<=0;trig_angle<=0;cs<=65536;sn<=0;body_cs<=65536;body_sn<=0;
   math_start<=0;math_op<=0;math_a<=0;math_b<=1;p0<=0;p1<=0;p2<=0;p3<=0;local_x<=0;local_y<=0;local_tx<=0;local_ty<=0;relative_heading<=0;
   body_vx<=0;body_vy<=0;omega<=0;angular_mm_s<=0;nav_x<=0;nav_y<=0;nav_z<=0;previous_vx<=0;previous_vy<=0;measured_map_vx<=0;measured_map_vy<=0;
   command_ms<=0;plan_epoch<=0;command_epoch<=0;plan_valid<=0;goal_checked<=0;goal_reached<=0;progress<=0;best_distance<=65535;desired_heading<=0;hist_probe_index<=0;
   path_occluded<=0;occlusion_seen<=0;clear_plans<=3;rejoin_ms<=0;
   previous_waypoint<=0;
   coarse_min<=0;coarse_known<=0;target_probe_index<=0;
   path_cs<=65536;path_sn<=0;rejoin_weight<=256;
  end else begin
   path_start<=0;planner_start<=0;vfh_start<=0;trig_start<=0;math_start<=0;goal_checked<=0;goal_reached<=0;progress<=0;
   case(state)
    0:if(plan_tick && running && map_fresh && imu_fresh && !planner_busy && !path_busy && !math_busy)begin path_start<=1;plan_epoch<=map_epoch;state<=1;end
    1:if(path_done)begin
     goal_checked<=1;goal_reached<=goal_distance<=100 && heading<5719 && heading> -5719 && (waypoint_index+1<point_count || feedback_still);
     if(goal_distance+10<best_distance)begin best_distance<=goal_distance;progress<=1;end
     if((goal_distance<=100 && heading<5719 && heading> -5719) || path_error)begin nav_x<=0;nav_y<=0;nav_z<=0;plan_valid<=0;state<=0;end
     else begin trig_angle<=map_heading;trig_start<=1;state<=2;end
    end
    2:if(trig_done)begin cs<=cosine;sn<=sine;state<=3;end
    3:begin p0<=($signed(px_q16[31:16])-map_x)*cs;state<=4;end
    4:begin p1<=($signed(py_q16[31:16])-map_y)*sn;state<=5;end
    5:begin p2<=($signed(px_q16[31:16])-map_x)*sn;state<=6;end
    6:begin p3<=($signed(py_q16[31:16])-map_y)*cs;state<=7;end
    7:begin local_x<=(p0+p1)>>>16;local_y<=(p3-p2)>>>16;state<=8;end
    8:begin p0<=(target_x-map_x)*cs;state<=9;end
    9:begin p1<=(target_y-map_y)*sn;state<=10;end
    10:begin p2<=(target_x-map_x)*sn;state<=11;end
    11:begin p3<=(target_y-map_y)*cs;state<=12;end
    12:begin local_tx<=(p0+p1)>>>16;local_ty<=(p3-p2)>>>16;relative_heading<=wrap(heading-map_heading);trig_angle<=wrap(heading-map_heading);trig_start<=1;state<=13;end
    13:if(trig_done)begin body_cs<=cosine;body_sn<=sine;
     omega<=dangers!=0 || active_flow_risk?0:heading>11438?-22876:heading< -11438?22876:-($signed(heading)<<<1);state<=30;end
    30:begin p0<=$signed(actual_x)*body_cs;state<=31;end
    31:begin p1<=$signed(actual_y)*body_sn;state<=32;end
    32:begin p2<=$signed(actual_x)*body_sn;state<=33;end
    33:begin p3<=$signed(actual_y)*body_cs;state<=34;end
    34:begin measured_map_vx<=(p0-p1)>>>16;measured_map_vy<=(p2+p3)>>>16;
     math_op<=3;math_a<=local_tx-local_x;math_b<=local_ty-local_y;math_start<=1;state<=14;end
    14:if(math_done)begin
     math_op<=4;math_a<=math_result;math_b<=0;math_start<=1;state<=43;end
    43:if(math_done)begin path_cs<=math_result[63:32];path_sn<=math_result[31:0];
     trig_angle<=wrap(relative_heading+laser_yaw);trig_start<=1;state<=44;end
    // Histogram bins are native laser angles, while path costs use map axes.
    // Transform the probe into the sensor frame including its mounting yaw.
    44:if(trig_done)begin cs<=cosine;sn<=sine;state<=45;end
    45:begin p0<=(local_tx-local_x)*cs;state<=46;end
    46:begin p1<=(local_ty-local_y)*sn;state<=47;end
    47:begin p2<=(local_tx-local_x)*sn;state<=48;end
    48:begin p3<=(local_ty-local_y)*cs;state<=49;end
    49:begin math_op<=3;math_a<=(p0+p1)>>>16;math_b<=(p3-p2)>>>16;math_start<=1;state<=50;end
    50:if(math_done)begin desired_heading<=math_result;target_probe_index<=(math_result<0?math_result+36000:math_result)/500;
     vfh_start<=1;state<=15;end
    15:if(vfh_done)begin hist_probe_index<=0;coarse_min<=65535;coarse_known<=1;state<=40;end
    40:state<=41;
    41:state<=42;
    42:begin
     if(!hist_known)coarse_known<=0;else if(hist_distance<coarse_min)coarse_min<=hist_distance;
     if(hist_probe_index==71)begin hist_probe_index<=target_probe_index;state<=16;end
     else begin hist_probe_index<=hist_probe_index+1'b1;state<=40;end
    end
    16:state<=17;
    17:begin occlusion_seen<=!hist_known || blocked[target_probe_index] || hist_distance<radius+400;state<=51;end
    51:begin
     if(occlusion_seen)begin path_occluded<=1;clear_plans<=0;end
     else if(path_occluded)begin if(clear_plans==2)begin path_occluded<=0;clear_plans<=3;rejoin_ms<=now_ms;end else clear_plans<=clear_plans+1'b1;end
     planner_start<=1;state<=18;
    end
    18:if(planner_done)begin
     plan_valid<=0;
     if(!planner_found || planner_timeout)begin nav_x<=0;nav_y<=0;nav_z<=0;plan_valid<=0;previous_vx<=0;previous_vy<=0;state<=0;end
     else begin previous_vx<=planned_x;previous_vy<=planned_y;state<=19;end
    end
    19:begin p0<=$signed(planned_x)*body_cs;state<=20;end
    20:begin p1<=$signed(planned_y)*body_sn;state<=21;end
    21:begin p2<=$signed(planned_x)*body_sn;state<=22;end
    22:begin p3<=$signed(planned_y)*body_cs;state<=23;end
    23:begin body_vx<=(p0+p1)>>>16;body_vy<=(p3-p2)>>>16;state<=24;end
    24:begin p0<=$signed(omega)*$signed({1'b0,rotation_lever});state<=25;end
    25:begin angular_mm_s<=p0>>>16;divide($signed({{32{body_vx[31]}},body_vx})<<<24,forward_scale,26);end
    26:if(math_done)begin nav_x<=limit_counts(math_result);divide($signed({{32{body_vy[31]}},body_vy})<<<24,lateral_scale,27);end
    27:if(math_done)begin nav_y<=limit_counts(math_result);divide($signed({{32{angular_mm_s[31]}},angular_mm_s})<<<24,forward_scale,28);end
    28:if(math_done)begin nav_z<=limit_counts(math_result);command_ms<=now_ms;command_epoch<=plan_epoch;plan_valid<=1;state<=0;end
    default:begin plan_valid<=0;nav_x<=0;nav_y<=0;nav_z<=0;state<=0;end
   endcase
   if(origin)begin best_distance<=65535;plan_valid<=0;previous_vx<=0;previous_vy<=0;end
   if(path_occluded || clear_plans<3)rejoin_weight<=0;
   else if(now_ms-rejoin_ms<100)rejoin_weight<=0;
   else if(now_ms-rejoin_ms<200)rejoin_weight<=51;
   else if(now_ms-rejoin_ms<300)rejoin_weight<=102;
   else if(now_ms-rejoin_ms<400)rejoin_weight<=153;
   else if(now_ms-rejoin_ms<500)rejoin_weight<=204;
   else rejoin_weight<=256;
   if(waypoint_index!=previous_waypoint)begin previous_waypoint<=waypoint_index;best_distance<=65535;end
   if(!running || !imu_fresh || !map_fresh || odom_overflow || (state>1 && map_epoch!=plan_epoch) || math_error)begin
    nav_x<=0;nav_y<=0;nav_z<=0;plan_valid<=0;if(!planner_busy && !path_busy && !trig_busy && !math_busy)state<=0;
   end
  end
 end
 reg [367:0] status_body;reg capture;
 wire [383:0] checked_frame;wire checked_valid;
 mc_frame_crc status_crc(.clk(clk),.reset_n(rst_n),.capture(capture),.body_in(status_body),.frame(checked_frame),.valid(checked_valid));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin status_body<=0;capture<=0;frame_valid<=0;frame<=0;end
  else begin
   capture<=0;if(frame_valid && frame_ready)frame_valid<=0;
   if(checked_valid)begin frame<=checked_frame;frame_valid<=1;end
   if(status_tick && !frame_valid)begin
    status_body<=0;status_body[15:0]<=16'h344e;status_body[23:16]<=1;
    status_body[31:24]<={3'd0,map_fresh,flow_degraded,nav_permit,running,nav_active};
    status_body[47:32]<=mission_id;status_body[55:48]<=waypoint_index;status_body[63:56]<=task_status;
    status_body[71:64]<=ack_seq;status_body[79:72]<=ack_error;status_body[87:80]<=reason;status_body[95:88]<={7'd0,calibrated};
    status_body[127:96]<=px_q16;status_body[159:128]<=py_q16;status_body[191:160]<=heading;
    status_body[207:192]<=goal_x;status_body[223:208]<=goal_y;status_body[239:224]<=body_vx;status_body[255:240]<=body_vy;
    status_body[287:256]<=now_ms;status_body[303:288]<=map_epoch[15:0];status_body[319:304]<=ray_overflows[15:0];
    status_body[351:320]<=planner_cycles;status_body[359:352]<=dangers;status_body[367:360]<={5'd0,planner_timeout,search_limited,unknown_motion};capture<=1;
   end
  end
 end
endmodule
