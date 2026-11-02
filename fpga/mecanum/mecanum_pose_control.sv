// Relative local frame established by each new goal. Translation units are
// wheel encoder counts, independent of the yet-to-be-measured counts/revolution.
module mc_mecanum_pose_control #(parameter LATERAL_REVERSE=0)(
 input wire clk,reset_n,tick,active,new_goal,
 input wire [1:0] mode, // 2: local position + heading; all other modes inactive
 input wire yaw_ok,input wire signed [31:0] yaw_q16,
 input wire [127:0] wheel_positions,wheel_velocities,
 input wire signed [15:0] goal_x,goal_y,goal_centideg,
 input wire [15:0] speed_limit,position_tolerance,
 output reg signed [15:0] command_x,command_y,command_z,
 output reg command_valid,arrived,overflow,
 output reg signed [31:0] x_q16,y_q16,heading_q16
);
 localparam signed [31:0] PI=205887,TWO_PI=411775,ANGLE_TOL=1144;
 reg [3:0] state;
 reg signed [31:0] previous[0:3],origin_yaw;
 wire signed [31:0] difference[0:3];
 genvar n;
 generate for(n=0;n<4;n=n+1)begin:deltas
  assign difference[n]=$signed(wheel_positions[n*32+:32])-previous[n];
 end endgenerate
 wire signed [33:0] d0={{2{difference[0][31]}},difference[0]},d1={{2{difference[1][31]}},difference[1]},
  d2={{2{difference[2][31]}},difference[2]},d3={{2{difference[3][31]}},difference[3]};
 reg signed [33:0] forward_sum,lateral_sum;
 reg trig_start;wire trig_done,trig_busy;
 wire signed [31:0] cosine,sine;
 reg signed [31:0] cos_hold,sin_hold,angle_hold,error_x,error_y,error_angle;
 reg signed [63:0] dx1,dx2,dy1,dy2,ex1,ex2,ey1,ey2,next_x,next_y;
 reg [5:0] settled;
 reg position_captured;
 wire signed [33:0] lateral_delta=-d0+d1+d2-d3;
 integer i;
 function automatic signed [31:0] wrap(input signed [31:0] a);
  if(a>PI)wrap=a-TWO_PI;else if(a< -PI)wrap=a+TWO_PI;else wrap=a;
 endfunction
 function automatic signed [15:0] limited(input signed [63:0] value,input [15:0] limit_value);
  if(value>$signed({1'b0,limit_value}))limited=$signed(limit_value);
  else if(value< -$signed({1'b0,limit_value}))limited=-$signed(limit_value);
  else limited=value[15:0];
 endfunction
 function automatic signed [15:0] position_speed(input signed [63:0] error_q32,input [15:0] limit_value,input [15:0] tol);
  reg signed [63:0] tol_q32,scaled;
  begin
   // Suppress sub-quarter-count CORDIC residue on an otherwise zero axis.
   tol_q32=$signed({1'b0,tol})*64'sd4294967296+64'sd1073741824;
   scaled=error_q32>>>38; // proportional gain 1/64 per 10ms
   if(error_q32<=tol_q32 && error_q32>=-tol_q32)position_speed=0;
   else if(scaled==0)position_speed=error_q32<0 ? -16'sd1 : 16'sd1;
   else position_speed=limited(scaled,limit_value);
  end
 endfunction
 wire signed [63:0] target_angle_product=$signed(goal_centideg)*64'sd749613;
 wire signed [31:0] target_angle=target_angle_product>>>16;
 wire signed [63:0] tolerance_q16=$signed({1'b0,position_tolerance})*64'sd65536;
 wire near_position=error_x<=tolerance_q16 && error_x>=-tolerance_q16 && error_y<=tolerance_q16 && error_y>=-tolerance_q16;
 wire near_angle=error_angle<=ANGLE_TOL && error_angle>=-ANGLE_TOL;
 wire wheels_still=($signed(wheel_velocities[31:0])<=1 && $signed(wheel_velocities[31:0])>=-1 &&
  $signed(wheel_velocities[63:32])<=1 && $signed(wheel_velocities[63:32])>=-1 &&
  $signed(wheel_velocities[95:64])<=1 && $signed(wheel_velocities[95:64])>=-1 &&
  $signed(wheel_velocities[127:96])<=1 && $signed(wheel_velocities[127:96])>=-1);
 mc_cordic_sincos trig(.clk(clk),.rst_n(reset_n),.start(trig_start),.angle_q16(angle_hold),
  .busy(trig_busy),.done(trig_done),.cos_q16(cosine),.sin_q16(sine));
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   state<=0;trig_start<=0;command_valid<=0;arrived<=0;overflow<=0;settled<=0;position_captured<=0;
   x_q16<=0;y_q16<=0;heading_q16<=0;origin_yaw<=0;forward_sum<=0;lateral_sum<=0;
   cos_hold<=0;sin_hold<=0;angle_hold<=0;error_x<=0;error_y<=0;error_angle<=0;
   dx1<=0;dx2<=0;dy1<=0;dy2<=0;ex1<=0;ex2<=0;ey1<=0;ey2<=0;next_x<=0;next_y<=0;
   command_x<=0;command_y<=0;command_z<=0;for(i=0;i<4;i=i+1)previous[i]<=0;
  end else begin
   trig_start<=0;command_valid<=0;
   if(new_goal)begin
    state<=0;x_q16<=0;y_q16<=0;heading_q16<=0;origin_yaw<=yaw_q16;overflow<=0;
    command_x<=0;command_y<=0;command_z<=0;arrived<=0;settled<=0;position_captured<=0;
    for(i=0;i<4;i=i+1)previous[i]<=wheel_positions[i*32+:32];
   end else if(!active || mode!=2 || !yaw_ok || overflow)begin state<=0;command_x<=0;command_y<=0;command_z<=0;settled<=0;end
   else case(state)
    0:if(tick && !trig_busy)begin
     for(i=0;i<4;i=i+1)previous[i]<=wheel_positions[i*32+:32];
     forward_sum<=d0+d1+d2+d3;
     lateral_sum<=LATERAL_REVERSE ? -lateral_delta : lateral_delta;
     angle_hold<=wrap(yaw_q16-origin_yaw);heading_q16<=wrap(yaw_q16-origin_yaw);state<=1;
    end
    1:begin trig_start<=1;state<=2;end
    2:if(trig_done)begin cos_hold<=cosine;sin_hold<=sine;state<=3;end
    3:begin dx1<=forward_sum*cos_hold;dx2<=lateral_sum*sin_hold;dy1<=forward_sum*sin_hold;dy2<=lateral_sum*cos_hold;state<=4;end
    4:begin next_x<=$signed(x_q16)+((dx1-dx2)>>>2);next_y<=$signed(y_q16)+((dy1+dy2)>>>2);state<=5;end
    5:begin
     if(next_x>1048576000 || next_x< -64'sd1048576000 || next_y>1048576000 || next_y< -64'sd1048576000)overflow<=1;
     else begin x_q16<=next_x[31:0];y_q16<=next_y[31:0];end
     state<=6;
    end
    6:begin
     error_x<=($signed(goal_x)*32'sd65536)-x_q16;error_y<=($signed(goal_y)*32'sd65536)-y_q16;
     error_angle<=wrap(target_angle-heading_q16);state<=7;
    end
    7:begin ex1<=error_x*cos_hold;ex2<=error_y*sin_hold;ey1<=error_y*cos_hold;ey2<=error_x*sin_hold;state<=8;end
    8:begin
     // Capture completion once; coasting must not restart this relative move.
     if(mode==2 && near_position && near_angle)position_captured<=1;
     command_x<=(position_captured || (near_position && near_angle)) ? 16'sd0 : position_speed(ex1+ex2,speed_limit,16'd0);
     command_y<=(position_captured || (near_position && near_angle)) ? 16'sd0 : position_speed(ey1-ey2,speed_limit,16'd0);
     command_z<=(mode==2 && position_captured) || near_angle ? 16'sd0 : (error_angle>0 && error_angle<2048) ? 16'sd1 : limited($signed(error_angle)>>>11,speed_limit);
     command_valid<=!overflow;state<=0;
     if(mode==2 && (position_captured || (near_position && near_angle)) && wheels_still)begin
      if(settled<20)settled<=settled+1'b1;
      if(settled>=19)arrived<=1;
     end else begin settled<=0;arrived<=0;end
    end
    default:state<=0;
   endcase
  end
 end
endmodule
