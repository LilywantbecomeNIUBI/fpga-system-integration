module mecanum_core #(
 parameter CLK_HZ=50000000,CONTROL_HZ=100,PWM_HZ=10000,
 WATCHDOG_MS=250,STALL_TICKS=50,REVERSE_TICKS=10,FILTER_CYCLES=8,CONTROL_SHIFT=0,LATERAL_START_PWM=0,LATERAL_TAPER_COUNTS=160,LATERAL_REVERSE=0,
 parameter [3:0] BASE_ENCODER_REVERSE=4'b0000,BASE_MOTOR_REVERSE=4'b0000
)(
 input wire clk,reset_n,input wire [3:0] enc_a,enc_b,
 input wire command_valid,input wire [175:0] command_body,
 input wire yaw_ok,input wire signed [31:0] yaw_q16,
 output wire [3:0] in1,in2,output reg armed,output reg [15:0] faults,
 output reg [7:0] last_sequence,output reg [31:0] rejected,
 output wire [127:0] positions,velocities,output wire [63:0] applied,targets,
 output reg [1:0] control_mode,output wire arrived,output reg imu_recalibrate,
 output wire signed [31:0] pose_x,pose_y,pose_heading
);
 localparam WINDOW=CLK_HZ/CONTROL_HZ,WD=(CLK_HZ/1000)*WATCHDOG_MS;
 integer divider,watchdog;
 reg sample_tick,control_tick;
 reg [15:0] kp,ki;
 reg signed [15:0] requests[0:3];
 wire [7:0] op=command_body[31:24],seq=command_body[39:32];
 wire [7:0] delta=seq-last_sequence;
 wire fresh=delta!=0 && delta<128;
 wire signed [15:0] x=command_body[79:64],y=command_body[95:80],z=command_body[111:96];
 wire [15:0] cmd_kp=command_body[127:112],cmd_ki=command_body[143:128];
 wire zero=x==0 && y==0 && z==0;
 wire config_ok=command_body[55:40]==0 && cmd_kp>0 && cmd_kp<=4096 && cmd_ki<=1024;
 wire config_same=command_body[55:40]==0 && kp==cmd_kp && ki==cmd_ki;
 wire enable=reset_n && armed && faults==0;
 reg new_goal;
 reg signed [15:0] goal_x,goal_y,goal_angle;
 reg [15:0] outer_speed,outer_tolerance;
 wire signed [15:0] outer_x,outer_y,outer_z;
 wire outer_valid,pose_overflow,pose_arrived;
 // Invalidate the previous result immediately when a new goal is accepted,
 // before the pose pipeline observes its reset pulse on the next clock.
 assign arrived=pose_arrived && control_mode==2 && !new_goal && enable;
 wire signed [15:0] outer_mix[0:3];
 wire outer_options=command_body[159:144]>=1 && command_body[159:144]<=128 && command_body[175:160]>=1 && command_body[175:160]<=1024;
 wire pose_bounds=x>=-16000 && x<=16000 && y>=-16000 && y<=16000 && z>=-18000 && z<=18000;
 wire [127:0] control_positions,control_velocities;
 wire [15:0] lateral_compensation;
 lateral_pwm_compensation #(.PWM_PERMILLE(LATERAL_START_PWM),.TAPER_COUNTS(LATERAL_TAPER_COUNTS)) lateral_assist(
  .clk(clk),.reset_n(reset_n),.active(enable),.arrived(arrived),.mode(control_mode),.goal_x(goal_x),.goal_y(goal_y),.goal_angle(goal_angle),
  .position_y(pose_y),.compensation(lateral_compensation));
 mecanum_pose_control #(.LATERAL_REVERSE(LATERAL_REVERSE)) pose_loop(.clk(clk),.reset_n(reset_n),.tick(control_tick),.active(enable && control_mode!=0),.new_goal(new_goal),
  .mode(control_mode),.yaw_ok(yaw_ok),.yaw_q16(yaw_q16),.wheel_positions(control_positions),.wheel_velocities(control_velocities),
  .goal_x(goal_x),.goal_y(goal_y),.goal_centideg(goal_angle),.speed_limit(outer_speed),.position_tolerance(outer_tolerance),
  .command_x(outer_x),.command_y(outer_y),.command_z(outer_z),.command_valid(outer_valid),.arrived(pose_arrived),.overflow(pose_overflow),
  .x_q16(pose_x),.y_q16(pose_y),.heading_q16(pose_heading));
 wire signed [15:0] chassis_outer_y=LATERAL_REVERSE ? -outer_y : outer_y;
 mecanum_mixer outer_mixer(.x(outer_x),.y(chassis_outer_y),.z(outer_z),.fl(outer_mix[0]),.fr(outer_mix[1]),.rl(outer_mix[2]),.rr(outer_mix[3]));
 integer stalls[0:3];integer i;
 genvar n;
 generate for(n=0;n<4;n=n+1)begin:wheels
  wire signed [31:0] position,velocity;
  wire signed [31:0] control_position,control_velocity;
  wire signed [15:0] pwm;
  encoder_ab #(.FILTER_CYCLES(FILTER_CYCLES)) encoder(.clk(clk),.reset_n(reset_n),.a(enc_a[n]),.b(enc_b[n]),
   .sample_tick(sample_tick),.reverse(BASE_ENCODER_REVERSE[n]),.position(position),.velocity(velocity),
   .invalid_count(),.diagnostics(),.diagnostic_levels(),.transition_trace(),.transition_count());
  encoder_control_scale #(.SHIFT(CONTROL_SHIFT)) scale(.clk(clk),.reset_n(reset_n),.sample_tick(sample_tick),
   .raw_position(position),.raw_velocity(velocity),.control_position(control_position),.control_velocity(control_velocity));
  wheel_control #(.PWM_LIMIT(300),.REVERSE_TICKS(REVERSE_TICKS)) controller(.clk(clk),.reset_n(reset_n),.tick(control_tick),
   .enable(enable),.motor_reverse(BASE_MOTOR_REVERSE[n]),.target(requests[n]),.velocity(control_velocity),.kp(kp),.ki(ki),.feedforward(lateral_compensation),.applied(pwm));
  at8236_pwm #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ)) driver(.clk(clk),.reset_n(reset_n),.enable(enable),.command(pwm),.in1(in1[n]),.in2(in2[n]));
  assign positions[n*32+:32]=position;assign velocities[n*32+:32]=velocity;
  assign control_positions[n*32+:32]=control_position;assign control_velocities[n*32+:32]=control_velocity;
  assign applied[n*16+:16]=pwm;assign targets[n*16+:16]=requests[n];
 end endgenerate
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin divider<=0;sample_tick<=0;control_tick<=0;end
  else begin
   sample_tick<=0;control_tick<=sample_tick;
   if(divider==WINDOW-1)begin divider<=0;sample_tick<=1;end else divider<=divider+1;
  end
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   armed<=0;faults<=1;last_sequence<=0;rejected<=0;kp<=256;ki<=0;watchdog<=0;imu_recalibrate<=0;
   control_mode<=0;new_goal<=0;goal_x<=0;goal_y<=0;goal_angle<=0;outer_speed<=40;outer_tolerance<=4;
   for(i=0;i<4;i=i+1)begin requests[i]<=0;stalls[i]<=0;end
  end else begin
   new_goal<=0;imu_recalibrate<=0;
   if(outer_valid && enable && control_mode!=0 && !new_goal)
    for(i=0;i<4;i=i+1)requests[i]<=outer_mix[i];
   if(armed && watchdog<WD)watchdog<=watchdog+1;
   if(!armed)for(i=0;i<4;i=i+1)stalls[i]<=0;
   if(command_valid)begin
    if(op==0)begin armed<=0;control_mode<=0;for(i=0;i<4;i=i+1)requests[i]<=0;end
    else if(op==3 && !armed && zero)begin faults<=0;watchdog<=0;for(i=0;i<4;i=i+1)requests[i]<=0;end
    else if(op==8 && !armed && zero && fresh && config_ok && command_body[175:144]==0)begin
     imu_recalibrate<=1;last_sequence<=seq;control_mode<=0;watchdog<=0;
     for(i=0;i<4;i=i+1)requests[i]<=0;
    end
    else if(op==1 && !armed && faults==0 && zero && config_ok)begin
     armed<=1;kp<=cmd_kp;ki<=cmd_ki;
     last_sequence<=seq;watchdog<=0;for(i=0;i<4;i=i+1)requests[i]<=0;
     control_mode<=0;
    end else if(op==6 && enable && fresh && config_same && yaw_ok && outer_options && pose_bounds)begin
     last_sequence<=seq;watchdog<=0;control_mode<=2;new_goal<=1;
     goal_x<=x;goal_y<=y;goal_angle<=z;outer_speed<=command_body[159:144];outer_tolerance<=command_body[175:160];
     for(i=0;i<4;i=i+1)requests[i]<=0;
    end else if(op==7 && enable && fresh && config_same && zero && command_body[175:144]==0 && control_mode!=0)begin
     last_sequence<=seq;watchdog<=0;
    end else rejected<=rejected+1;
   end
   // Faults have priority over commands and stop every wheel together.
   if(armed && watchdog>=WD-1)begin faults[1]<=1;armed<=0;end
   if(enable && control_mode!=0 && !yaw_ok)begin faults[8]<=1;armed<=0;end
   if(enable && control_mode!=0 && pose_overflow)begin faults[9]<=1;armed<=0;end
   if(control_tick && armed)for(i=0;i<4;i=i+1)begin
    if($signed(velocities[i*32+:32])>4096 || $signed(velocities[i*32+:32])< -4096)begin faults[3]<=1;armed<=0;end
    if(($signed(applied[i*16+:16])>=100 || $signed(applied[i*16+:16])<=-100) && $signed(velocities[i*32+:32])==0)begin
     stalls[i]<=stalls[i]+1;
     if(stalls[i]>=STALL_TICKS-1)begin faults[4+i]<=1;armed<=0;end
    end else stalls[i]<=0;
   end
  end
 end
endmodule
