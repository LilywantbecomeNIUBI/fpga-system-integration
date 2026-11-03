// Bounded full-direction dynamic window; eight best candidates per period.
// Failed candidates rotate across periods. Circular clearance is advisory:
// no circular false-positive may veto a rectangularly safe translation.
module mc_nav_planner #(parameter MAX_CYCLES=2500000)(
 input wire clk,rst_n,start,abort,input wire [31:0] map_epoch,
 input wire signed [31:0] x_mm,y_mm,theta_q16,target_x_mm,target_y_mm,
 input wire signed [15:0] current_vx,current_vy,omega_step_q16,
 input wire signed [31:0] direction_cos_q16,direction_sin_q16,
 input wire signed [31:0] path_cos_q16,path_sin_q16,input wire [8:0] path_weight_q8,
 input wire [15:0] speed_mm_s,accel_mm_s2,front_mm,back_mm,half_width_mm,radius_mm,pad_mm,
 input wire [15:0] coarse_clearance_mm,
 input wire [7:0] danger_mask,input wire flow_risk,flow_degraded,path_occluded,
 input wire allow_zero_translation,
 input wire signed [15:0] suggested_heading_q100,
 output reg busy,done,found,search_limited,timed_out,unknown_motion,
 output reg signed [15:0] vx_mm_s,vy_mm_s,output reg [31:0] cycles,
 output wire planner_lock,read_valid,output wire [13:0] read_address,
 input wire read_ready,read_done,input wire [1:0] read_code
);
 localparam IDLE=0,WINDOW=1,CANDIDATE=2,DIFF=3,FLAGS=4,SCORE=5,INSERT=6,
 NEXT=7,TEST_START=8,TEST_WAIT=9,TEST_SCORE=10,FINISH=11,CANDIDATE_Y=12,BCX=13,BSY=14,BSUMX=15,BSX=16,BCY=17,BSUMY=18,RANK_COMPARE=19,RANK_COMMIT=20,PATH_X=21,PATH_Y=22,PATH_SUM=23,PATH_WEIGHT=24,
 WINDOW_BOUNDS=25,STEP_X=26,STEP_Y=27,STEP_COMMIT=28,TEST_COMMIT=29,SCORE_SUM=30,SCORE_FINAL=31;
 reg [4:0] state;reg [31:0] captured_epoch;
 reg [6:0] candidate;reg [3:0] nx,ny,insert_index,test_index;
 reg [80:0] tested;reg [7:0] failed_count;
 reg signed [15:0] lo_x,hi_x,lo_y,hi_y,cvx,cvy,speed,accel;
 reg signed [31:0] ex,ey,path_error,change_cost,raw_score;
 reg signed [31:0] change_x,change_y;
 reg candidate_ok,circle_clear,rank_less;reg signed [63:0] score;
 reg signed [63:0] ranks[0:7];reg signed [15:0] rank_x[0:7],rank_y[0:7];reg [6:0] rank_id[0:7];
 reg [7:0] rank_valid;reg signed [63:0] best_score;
 reg sweep_start;wire sweep_busy,sweep_done,sweep_safe,sweep_timeout,sweep_unknown;
 wire [31:0] sweep_cost,sweep_cycles;
 reg signed [15:0] test_x,test_y;
 reg signed [63:0] bp0,bp1;reg signed [31:0] body_x,body_y;
 reg signed [63:0] path_p0,path_p1,weighted_error;
 reg window_ok;reg [31:0] candidate_speed,feedback_speed;
 reg [31:0] accel_product;
 reg signed [15:0] step_x_q,step_y_q;
 reg [16:0] step_abs_x,step_abs_y;reg [34:0] step_product_x,step_product_y;
 reg signed [63:0] trial_total_score,score_progress,score_path,score_change,score_pair0,score_pair1;
 reg trial_eligible,trial_unknown;
 function automatic signed [31:0] abs32(input signed [31:0] a);abs32=a<0?-a:a;endfunction
 function automatic signed [15:0] clamp(input signed [31:0] a,input signed [15:0] limit_v);
  if(a>limit_v)clamp=limit_v;else if(a< -limit_v)clamp=-limit_v;else clamp=a;
 endfunction
 assign planner_lock=busy;
 mc_nav_sweep #(.MAX_CYCLES(MAX_CYCLES)) sweep(.clk(clk),.rst_n(rst_n),.start(sweep_start),.abort(abort || timed_out),.map_epoch(map_epoch),
  .x_mm(x_mm),.y_mm(y_mm),.theta_q16(theta_q16),.step_x(step_x_q),.step_y(step_y_q),
  .step_theta({{16{omega_step_q16[15]}},omega_step_q16}),.steps(6'd20),.front_mm(front_mm),.back_mm(back_mm),.half_width_mm(half_width_mm),
  // Integer-step error <29mm; half-sample translation+rotation <22mm
  // at <=150mm/s, radius<=800mm and |omega|<=0.35rad/s. Add 64mm
  // separately from measured braking +36mm cell-center margin.
  .radius_mm(radius_mm),.pad_mm(pad_mm+16'd64),.busy(sweep_busy),.done(sweep_done),.safe(sweep_safe),.timed_out(sweep_timeout),
  .unknown_seen(sweep_unknown),.unknown_cost(sweep_cost),.cycles(sweep_cycles),.read_valid(read_valid),.read_address(read_address),
  .read_ready(read_ready),.read_done(read_done),.read_code(read_code));
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;busy<=0;done<=0;found<=0;search_limited<=0;timed_out<=0;unknown_motion<=0;cycles<=0;
   captured_epoch<=0;candidate<=0;nx<=0;ny<=0;insert_index<=0;test_index<=0;tested<=0;failed_count<=0;
   lo_x<=0;hi_x<=0;lo_y<=0;hi_y<=0;cvx<=0;cvy<=0;speed<=0;accel<=0;ex<=0;ey<=0;path_error<=0;change_cost<=0;raw_score<=0;
   candidate_ok<=0;score<=0;rank_valid<=0;best_score<=64'h7fffffffffffffff;sweep_start<=0;test_x<=0;test_y<=0;vx_mm_s<=0;vy_mm_s<=0;
   change_x<=0;change_y<=0;
   for(i=0;i<8;i=i+1)begin ranks[i]<=64'h7fffffffffffffff;rank_x[i]<=0;rank_y[i]<=0;rank_id[i]<=0;end
   bp0<=0;bp1<=0;body_x<=0;body_y<=0;
   circle_clear<=0;rank_less<=0;
   path_p0<=0;path_p1<=0;weighted_error<=0;
   window_ok<=0;candidate_speed<=0;feedback_speed<=0;accel_product<=0;
   step_x_q<=0;step_y_q<=0;step_abs_x<=0;step_abs_y<=0;step_product_x<=0;step_product_y<=0;
   trial_total_score<=0;trial_eligible<=0;trial_unknown<=0;
   score_progress<=0;score_path<=0;score_change<=0;score_pair0<=0;score_pair1<=0;
  end else begin
   done<=0;sweep_start<=0;if(busy)cycles<=cycles+1'b1;
   case(state)
    IDLE:if(start && !sweep_busy)begin
     busy<=1;found<=0;search_limited<=0;timed_out<=0;unknown_motion<=0;cycles<=0;captured_epoch<=map_epoch;
     candidate<=0;nx<=0;ny<=0;rank_valid<=0;test_index<=0;vx_mm_s<=0;vy_mm_s<=0;best_score<=64'h7fffffffffffffff;
     speed<=flow_degraded && speed_mm_s>50?50:speed_mm_s;
     // Exact for the full unsigned16 interface, unlike26215>>19 above43698.
     accel_product<=accel_mm_s2*16'd52429;feedback_speed<=abs32(current_vx)+abs32(current_vy);state<=WINDOW;
     if(failed_count>=81)begin tested<=0;failed_count<=0;end
     for(i=0;i<8;i=i+1)ranks[i]<=64'h7fffffffffffffff;
    end
    WINDOW:begin
     accel<=accel_mm_s2<20?1:accel_product>>20;
     window_ok<=speed_mm_s<=150 && feedback_speed<=speed_mm_s && omega_step_q16<=2294 && omega_step_q16>=-2294;
     state<=WINDOW_BOUNDS;
    end
    WINDOW_BOUNDS:begin lo_x<=clamp($signed(current_vx)-accel,speed);hi_x<=clamp($signed(current_vx)+accel,speed);
     lo_y<=clamp($signed(current_vy)-accel,speed);hi_y<=clamp($signed(current_vy)+accel,speed);state<=CANDIDATE;end
    CANDIDATE:begin
     cvx<=lo_x+((($signed(hi_x)-$signed(lo_x))*$signed({1'b0,nx}))>>>3);state<=CANDIDATE_Y;
    end
    CANDIDATE_Y:begin cvy<=lo_y+((($signed(hi_y)-$signed(lo_y))*$signed({1'b0,ny}))>>>3);state<=BCX;end
    BCX:begin bp0<=$signed(cvx)*direction_cos_q16;state<=BSY;end
    BSY:begin bp1<=$signed(cvy)*direction_sin_q16;state<=BSUMX;end
    BSUMX:begin body_x<=(bp0+bp1)>>>16;state<=BSX;end
    BSX:begin bp0<=$signed(cvx)*direction_sin_q16;state<=BCY;end
    BCY:begin bp1<=$signed(cvy)*direction_cos_q16;state<=BSUMY;end
    BSUMY:begin body_y<=(bp1-bp0)>>>16;state<=DIFF;end
    DIFF:begin ex<=target_x_mm-x_mm-($signed(cvx)<<<1);ey<=target_y_mm-y_mm-($signed(cvy)<<<1);
     change_x<=$signed(cvx)-$signed(current_vx);change_y<=$signed(cvy)-$signed(current_vy);
     change_cost<=abs32($signed(cvx)-$signed(current_vx))+abs32($signed(cvy)-$signed(current_vy));state<=PATH_X;end
    PATH_X:begin path_p0<=ex*path_sin_q16;candidate_speed<=abs32(cvx)+abs32(cvy);state<=PATH_Y;end
    PATH_Y:begin path_p1<=ey*path_cos_q16;state<=PATH_SUM;end
    PATH_SUM:begin path_error<=path_occluded?0:abs32((path_p0-path_p1)>>>16);state<=PATH_WEIGHT;end
    PATH_WEIGHT:begin weighted_error<=path_error*$signed({1'b0,path_weight_q8});state<=FLAGS;end
    FLAGS:begin
     candidate_ok<=!tested[candidate] && window_ok && change_x<=accel && change_x>=-accel && change_y<=accel && change_y>=-accel && candidate_speed<=speed && !(danger_mask[0] && body_x>0) && !(danger_mask[1] && body_x<0) &&
       !(danger_mask[2] && body_y>0) && !(danger_mask[3] && body_y<0) && !(flow_risk && body_x>0);
     circle_clear<=coarse_clearance_mm>radius_mm+pad_mm+(candidate_speed<<1);
     state<=SCORE;
    end
    SCORE:begin
     score_progress<=((abs32(ex)+abs32(ey))<<<10);score_path<=(weighted_error>>>8)<<<3;
     score_change<=($signed(change_cost)<<<1);state<=SCORE_SUM;
    end
    SCORE_SUM:begin
     score_pair0<=score_progress+score_path;
     score_pair1<=score_change+(((suggested_heading_q100>4500 && cvy<0) || (suggested_heading_q100< -4500 && cvy>0))?64:0)+(circle_clear?0:128);
     state<=SCORE_FINAL;
    end
    SCORE_FINAL:begin score<=score_pair0+score_pair1;insert_index<=0;state<=RANK_COMPARE;end
    RANK_COMPARE:begin rank_less<=score<ranks[insert_index];state<=RANK_COMMIT;end
    RANK_COMMIT:if(!candidate_ok)state<=NEXT;
     else if(rank_less)begin
      for(i=7;i>0;i=i-1)if(i>insert_index)begin ranks[i]<=ranks[i-1];rank_x[i]<=rank_x[i-1];rank_y[i]<=rank_y[i-1];rank_id[i]<=rank_id[i-1];rank_valid[i]<=rank_valid[i-1];end
      ranks[insert_index]<=score;rank_x[insert_index]<=cvx;rank_y[insert_index]<=cvy;rank_id[insert_index]<=candidate;rank_valid[insert_index]<=1;state<=NEXT;
     end else if(insert_index==7)state<=NEXT;else begin insert_index<=insert_index+1'b1;state<=RANK_COMPARE;end
    NEXT:if(candidate==80)begin test_index<=0;state<=TEST_START;end
     else begin candidate<=candidate+1'b1;if(nx==8)begin nx<=0;ny<=ny+1'b1;end else nx<=nx+1'b1;state<=CANDIDATE;end
    TEST_START:if(!rank_valid[test_index])state<=FINISH;
     else if(!sweep_busy)begin test_x<=rank_x[test_index];test_y<=rank_y[test_index];state<=TEST_SCORE;end
    TEST_SCORE:begin
     step_abs_x<=test_x[15]?-$signed({test_x[15],test_x}):$signed({test_x[15],test_x});
     step_abs_y<=test_y[15]?-$signed({test_y[15],test_y}):$signed({test_y[15],test_y});state<=STEP_X;
    end
    STEP_X:begin step_product_x<=step_abs_x*18'd26215;state<=STEP_Y;end
    STEP_Y:begin step_product_y<=step_abs_y*18'd26215;state<=STEP_COMMIT;end
    STEP_COMMIT:begin
     step_x_q<=test_x[15]?-$signed(step_product_x>>18):$signed(step_product_x>>18);
     step_y_q<=test_y[15]?-$signed(step_product_y>>18):$signed(step_product_y>>18);
     sweep_start<=1;state<=TEST_WAIT;
    end
    TEST_WAIT:if(sweep_done)begin
     tested[rank_id[test_index]]<=1;failed_count<=failed_count+1'b1;
     trial_total_score<=ranks[test_index]+({32'd0,sweep_cost}<<6);
     trial_eligible<=sweep_safe && (!sweep_unknown || step_abs_x+step_abs_y<=50) && (allow_zero_translation || test_x!=0 || test_y!=0);
     trial_unknown<=sweep_unknown;state<=TEST_COMMIT;
    end
    TEST_COMMIT:begin
     if(trial_eligible && trial_total_score<best_score)begin
      found<=1;best_score<=trial_total_score;vx_mm_s<=test_x;vy_mm_s<=test_y;unknown_motion<=trial_unknown;
     end
     if(test_index==7)state<=FINISH;else begin test_index<=test_index+1'b1;state<=TEST_START;end
    end
    FINISH:begin
     if(found || rank_valid==0)begin tested<=0;failed_count<=0;end
     if(!found)begin search_limited<=1;vx_mm_s<=0;vy_mm_s<=0;end
     busy<=0;done<=1;state<=IDLE;
    end
    default:begin timed_out<=1;found<=0;vx_mm_s<=0;vy_mm_s<=0;state<=FINISH;end
   endcase
   if(busy && state!=FINISH && (abort || map_epoch!=captured_epoch || cycles>=MAX_CYCLES-1))begin
    timed_out<=1;found<=0;vx_mm_s<=0;vy_mm_s<=0;state<=FINISH;
   end
  end
 end
endmodule
