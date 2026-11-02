// Translation-only multi-point tracker on the existing tagged feature history.
// Shared fixed-point LK, 16 ROI points, 3 forward/backward iterations and median.
// Not full MedianFlow: no pyramid, NCC or scale estimation in this first version.
module vision_tracker #(
 parameter GRID_W=80,GRID_H=45,PATCH=8,SEARCH=4,SCALE=16,
 parameter HISTORY=16,MAX_SEMANTIC_FRAMES=14,RED_PERMILLE=250,
 parameter MAX_SAD=1024,MIN_MARGIN=32,MIN_TEXTURE=16,
 parameter MIN_POINTS=4,FB_LIMIT=96,CONSENSUS_LIMIT=128,ITERATIONS=3
)(
 input clk,rst_n,invalidate,frame_start,frame_end,frame_good,input [15:0] frame_id,
 input feature_valid,input [15:0] feature_address,feature,
 input seed_valid,input [256:0] seed_data,
 output reg status_valid,output reg [255:0] status_payload
);
 localparam CELLS=GRID_W*GRID_H,AB=$clog2(CELLS),HB=$clog2(HISTORY);
 localparam IMAGE_W=GRID_W*SCALE,IMAGE_H=GRID_H*SCALE;
 reg [15:0] memory[0:(1<<(AB+HB))-1];
 reg [15:0] tags[0:HISTORY-1];reg [HISTORY-1:0] sealed;
 reg [15:0] write_fid,feature_count,latest_fid;reg have_frame,latest_good;
 reg [15:0] read_word;wire [AB+HB-1:0] read_address;
 always @(posedge clk)begin
  if(feature_valid && feature_address<CELLS)memory[{write_fid[HB-1:0],feature_address[AB-1:0]}]<=feature;
  read_word<=memory[read_address];
 end
 integer k;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sealed<=0;write_fid<=0;feature_count<=0;latest_fid<=0;have_frame<=0;latest_good<=0;for(k=0;k<HISTORY;k=k+1)tags[k]<=0;end
  else begin
   if(frame_start)begin write_fid<=frame_id;feature_count<=0;sealed[frame_id[HB-1:0]]<=0;end
   if(feature_valid && feature_address<CELLS)feature_count<=feature_count+1'b1;
   if(frame_end)begin
    latest_fid<=write_fid;have_frame<=1;latest_good<=frame_good && feature_count==CELLS;
    if(frame_good && feature_count==CELLS)begin tags[write_fid[HB-1:0]]<=write_fid;sealed[write_fid[HB-1:0]]<=1;end
   end
   if(invalidate)begin sealed<=0;have_frame<=0;latest_good<=0;feature_count<=0;end
  end
 end
 function present;input [15:0] f;begin present=sealed[f[HB-1:0]] && tags[f[HB-1:0]]==f;end endfunction
 function [16:0] abs17;input signed [16:0] v;begin abs17=v<0 ? -v : v;end endfunction
 localparam IDLE=0,SEED_INIT=1,PAIR_INIT=2,POINT_INIT=3,POINT_START=4,POINT_WAIT=5,POINT_NEXT=6,
  SORT_INIT=7,SORT_STEP=8,MEDIAN=9,COUNT_INIT=10,COUNT_STEP=11,TRANSLATE=12,APPLY=13,
  ROI_INIT=14,ROI_REQ=15,ROI_WAIT=16,ROI_TAKE=17,DIV_START=18,DIV_WAIT=19,PUBLISH=20,INVALID=21,POINT_CHECK=22,FB_CHECK=23,SORT_READ=24,SORT_COMPARE=25;
 reg [4:0] state /* synthesis syn_preserve=1 syn_maxfan=100000 */;reg pending_seed;reg [256:0] held_seed;
 reg active,tracking_allowed,track_ok,observation_ok,force_yolo,history_miss,scene_change;
 reg have_report;reg [15:0] reported_fid,semantic_fid,job_fid,tracked_fid;
 reg [15:0] class_id,confidence,x1,y1,x2,y2,track_confidence,red_ratio;reg [7:0] reason;
 reg [3:0] point_index;reg [4:0] good_count,sort_i,sort_j,inlier_count,count_index;
 reg [15:0] point_x,point_y,last_point_x,last_row_y;
 reg reverse,patch_start;reg fb_good;reg [1:0] iteration;
 reg signed [15:0] seed_u,seed_v,forward_u,forward_v,median_u,median_v;
 reg signed [15:0] sort_u[0:15],sort_v[0:15],values_u[0:15],values_v[0:15];
 reg signed [15:0] sort_au,sort_bu,sort_av,sort_bv;reg swap_u,swap_v;
 reg signed [31:0] remainder_x,remainder_y,shift_x,shift_y;
 wire [15:0] previous_fid=job_fid-16'd1;
 wire [15:0] candidate_x=(x1+(((point_index[1:0]*2+1)*(x2-x1))>>3))/SCALE;
 wire [15:0] candidate_y=(y1+(((point_index[3:2]*2+1)*(y2-y1))>>3))/SCALE;
 wire signed [16:0] reverse_x=$signed({1'b0,point_x})+(($signed(forward_u)+17'sd64)>>>7);
 wire signed [16:0] reverse_y=$signed({1'b0,point_y})+(($signed(forward_v)+17'sd64)>>>7);
 wire [15:0] center_x=reverse ? reverse_x[15:0] : point_x,center_y=reverse ? reverse_y[15:0] : point_y;
 reg [15:0] patch_center_x,patch_center_y;
 wire patch_busy,patch_done,patch_good,lk_read,lk_current;wire signed [15:0] pu,pv;wire [15:0] lk_address;
 flow_lk_patch #(.WIDTH(GRID_W),.HEIGHT(GRID_H)) lk(.clk(clk),.rst_n(rst_n),.start(patch_start),.coarse(1'b0),
  .center_x(patch_center_x),.center_y(patch_center_y),.initial_u(seed_u),.initial_v(seed_v),.busy(patch_busy),.done(patch_done),
  .valid(patch_good),.u(pu),.v(pv),.read_enable(lk_read),.read_current(lk_current),.read_address(lk_address),.read_data(read_word[7:0]));
 wire [15:0] selected_fid=(lk_current ^ reverse) ? job_fid : previous_fid;
 reg [AB+HB-1:0] roi_address;
 assign read_address=(state==POINT_START || state==POINT_WAIT) ? {selected_fid[HB-1:0],lk_address[AB-1:0]} : roi_address;
 wire signed [16:0] fb_u=$signed({forward_u[15],forward_u})+$signed({pu[15],pu});
 wire signed [16:0] fb_v=$signed({forward_v[15],forward_v})+$signed({pv[15],pv});
 wire signed [16:0] residual_u=$signed(values_u[count_index[3:0]])-$signed(median_u);
 wire signed [16:0] residual_v=$signed(values_v[count_index[3:0]])-$signed(median_v);
 wire signed [31:0] next_x1=$signed({1'b0,x1})+shift_x,next_x2=$signed({1'b0,x2})+shift_x;
 wire signed [31:0] next_y1=$signed({1'b0,y1})+shift_y,next_y2=$signed({1'b0,y2})+shift_y;
 reg [15:0] rx0,ry0,rxmax,rymax,rx,ry,red_sum,roi_cells;
 reg divide_start;reg [31:0] numerator,denominator;wire divide_done;wire [31:0] quotient;
 vision_divider divide(.clk(clk),.rst_n(rst_n),.start(divide_start),.numerator(numerator),.denominator(denominator),
  .busy(),.done(divide_done),.quotient(quotient));
 wire [15:0] semantic_age=job_fid-semantic_fid,latest_age=latest_fid-semantic_fid;
 wire semantic_live=active && !semantic_age[15] && semantic_age<=MAX_SEMANTIC_FRAMES;
 wire color_red=red_ratio>=RED_PERMILLE,fused=semantic_live && observation_ok && color_red;
 wire [7:0] flags={~latest_good,scene_change,history_miss,force_yolo,color_red,semantic_live,track_ok,fused};
 wire [16:0] xsum={1'b0,x1}+{1'b0,x2};wire [18:0] xtriple={2'd0,xsum}+{1'd0,xsum,1'b0};
 wire [7:0] direction=!fused ? 0 : xtriple<IMAGE_W*2 ? 1 : xtriple>=IMAGE_W*4 ? 3 : 2;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;pending_seed<=0;held_seed<=0;status_valid<=0;status_payload<=0;
   active<=0;tracking_allowed<=0;track_ok<=0;observation_ok<=0;force_yolo<=1;history_miss<=0;scene_change<=0;
   have_report<=0;reported_fid<=0;semantic_fid<=0;job_fid<=0;tracked_fid<=0;
   class_id<=0;confidence<=0;x1<=0;y1<=0;x2<=0;y2<=0;track_confidence<=0;red_ratio<=0;reason<=1;
   point_index<=0;good_count<=0;sort_i<=0;sort_j<=0;inlier_count<=0;count_index<=0;
   point_x<=0;point_y<=0;last_point_x<=0;last_row_y<=0;reverse<=0;patch_start<=0;fb_good<=0;iteration<=0;
   patch_center_x<=0;patch_center_y<=0;
   sort_au<=0;sort_bu<=0;sort_av<=0;sort_bv<=0;swap_u<=0;swap_v<=0;
   seed_u<=0;seed_v<=0;forward_u<=0;forward_v<=0;median_u<=0;median_v<=0;
   remainder_x<=0;remainder_y<=0;shift_x<=0;shift_y<=0;roi_address<=0;
   rx0<=0;ry0<=0;rxmax<=0;rymax<=0;rx<=0;ry<=0;red_sum<=0;roi_cells<=0;
   divide_start<=0;numerator<=0;denominator<=0;
  end else begin
   status_valid<=0;divide_start<=0;patch_start<=0;
   if(seed_valid)begin held_seed<=seed_data;pending_seed<=1;end
   case(state)
    IDLE:if(pending_seed && !patch_busy)begin
     pending_seed<=seed_valid;semantic_fid<=held_seed[207:192];job_fid<=held_seed[207:192];
     class_id<=held_seed[159:144];confidence<=held_seed[143:128];
     x1<=held_seed[127:112];y1<=held_seed[111:96];x2<=held_seed[95:80];y2<=held_seed[79:64];
     active<=held_seed[256];tracking_allowed<=0;track_ok<=0;observation_ok<=0;
     force_yolo<=0;history_miss<=0;scene_change<=0;red_ratio<=0;track_confidence<=0;
     remainder_x<=0;remainder_y<=0;state<=SEED_INIT;
    end else if(!patch_busy && have_frame && (!have_report || latest_fid!=reported_fid))begin
     if(active && tracking_allowed && !latest_age[15] && latest_age<=MAX_SEMANTIC_FRAMES)begin job_fid<=tracked_fid+1'b1;state<=PAIR_INIT;end
     else begin
      job_fid<=latest_fid;observation_ok<=0;track_ok<=0;force_yolo<=1;
      if(active && latest_age>MAX_SEMANTIC_FRAMES)begin active<=0;reason<=6;end
      state<=PUBLISH;
     end
    end
    SEED_INIT:begin
     if(!active)begin reason<=2;job_fid<=latest_fid;force_yolo<=1;state<=PUBLISH;end
     else if(!present(semantic_fid) || !have_frame || latest_age[15] || latest_age>=HISTORY-1 || x1>=x2 || y1>=y2 || x2>IMAGE_W || y2>IMAGE_H)begin
      active<=0;history_miss<=1;force_yolo<=1;reason<=5;state<=PUBLISH;
     end else begin tracked_fid<=job_fid;observation_ok<=1;tracking_allowed<=1;track_confidence<=1000;reason<=0;state<=ROI_INIT;end
    end
    PAIR_INIT:begin
     if(!present(job_fid) || !present(previous_fid))begin history_miss<=1;reason<=5;state<=INVALID;end
     else begin point_index<=0;good_count<=0;last_point_x<=0;last_row_y<=0;state<=POINT_INIT;end
    end
    POINT_INIT:begin
     point_x<=candidate_x;point_y<=candidate_y;reverse<=0;iteration<=0;seed_u<=0;seed_v<=0;
     state<=POINT_CHECK;
    end
    POINT_CHECK:begin
     if(point_x<3 || point_y<3 || point_x>=GRID_W-3 || point_y>=GRID_H-3 ||
       (point_index[1:0]!=0 && point_x==last_point_x) || (point_index[3:2]!=0 && point_y==last_row_y))state<=POINT_NEXT;
     else state<=POINT_START;
     last_point_x<=point_x;
    end
    POINT_START:if(!patch_busy)begin
     patch_center_x<=center_x;patch_center_y<=center_y;
     patch_start<=1;state<=POINT_WAIT;
    end
    POINT_WAIT:if(patch_done)begin
     if(!patch_good)state<=POINT_NEXT;
     else if(iteration!=ITERATIONS-1)begin seed_u<=pu;seed_v<=pv;iteration<=iteration+1'b1;state<=POINT_START;end
     else if(!reverse)begin forward_u<=pu;forward_v<=pv;reverse<=1;iteration<=0;seed_u<=-pu;seed_v<=-pv;state<=POINT_START;end
     else begin
      // Separate tensor result arithmetic from the wide point-cache write enable.
      fb_good<=abs17(fb_u)<=FB_LIMIT && abs17(fb_v)<=FB_LIMIT;state<=FB_CHECK;
     end
    end
    FB_CHECK:begin
     if(fb_good)begin
      sort_u[good_count[3:0]]<=forward_u;values_u[good_count[3:0]]<=forward_u;
      sort_v[good_count[3:0]]<=forward_v;values_v[good_count[3:0]]<=forward_v;good_count<=good_count+1'b1;
     end
     state<=POINT_NEXT;
    end
    POINT_NEXT:begin
     if(point_index[1:0]==3)last_row_y<=point_y;
     if(point_index==15)begin if(good_count<MIN_POINTS)begin reason<=8;state<=INVALID;end else state<=SORT_INIT;end
     else begin point_index<=point_index+1'b1;state<=POINT_INIT;end
    end
    SORT_INIT:begin sort_i<=0;sort_j<=0;state<=SORT_READ;end
    SORT_READ:begin
     sort_au<=sort_u[sort_j];sort_bu<=sort_u[sort_j+1'b1];
     sort_av<=sort_v[sort_j];sort_bv<=sort_v[sort_j+1'b1];state<=SORT_COMPARE;
    end
    SORT_COMPARE:begin swap_u<=sort_au>sort_bu;swap_v<=sort_av>sort_bv;state<=SORT_STEP;end
    SORT_STEP:begin
     if(swap_u)begin sort_u[sort_j]<=sort_bu;sort_u[sort_j+1'b1]<=sort_au;end
     if(swap_v)begin sort_v[sort_j]<=sort_bv;sort_v[sort_j+1'b1]<=sort_av;end
     state<=SORT_READ;
     if(sort_j==good_count-sort_i-2)begin sort_j<=0;if(sort_i==good_count-2)state<=MEDIAN;else sort_i<=sort_i+1'b1;end
     else sort_j<=sort_j+1'b1;
    end
    MEDIAN:begin median_u<=sort_u[good_count>>1];median_v<=sort_v[good_count>>1];state<=COUNT_INIT;end
    COUNT_INIT:begin inlier_count<=0;count_index<=0;state<=COUNT_STEP;end
    COUNT_STEP:begin
     if(abs17(residual_u)<=CONSENSUS_LIMIT && abs17(residual_v)<=CONSENSUS_LIMIT)inlier_count<=inlier_count+1'b1;
     if(count_index==good_count-1)state<=TRANSLATE;else count_index<=count_index+1'b1;
    end
    TRANSLATE:begin
     if(inlier_count<MIN_POINTS || ({1'b0,inlier_count}<<1)<good_count)begin reason<=4;state<=INVALID;end
     else begin
      shift_x<=($signed(median_u)*SCALE+remainder_x+32'sd64)>>>7;
      shift_y<=($signed(median_v)*SCALE+remainder_y+32'sd64)>>>7;state<=APPLY;
     end
    end
    APPLY:begin
     if(next_x1<0 || next_y1<0 || next_x2>IMAGE_W || next_y2>IMAGE_H)begin reason<=7;state<=INVALID;end
     else begin
      x1<=next_x1[15:0];x2<=next_x2[15:0];y1<=next_y1[15:0];y2<=next_y2[15:0];
      remainder_x<=$signed(median_u)*SCALE+remainder_x-(shift_x<<<7);remainder_y<=$signed(median_v)*SCALE+remainder_y-(shift_y<<<7);
      tracked_fid<=job_fid;track_ok<=1;observation_ok<=1;track_confidence<=inlier_count*1000/16;
      reason<=0;force_yolo<=inlier_count<8;state<=ROI_INIT;
     end
    end
    ROI_INIT:begin
     rx0<=x1/SCALE;ry0<=y1/SCALE;rx<=x1/SCALE;ry<=y1/SCALE;rxmax<=(x2+SCALE-1)/SCALE;rymax<=(y2+SCALE-1)/SCALE;
     red_sum<=0;roi_cells<=0;state<=ROI_REQ;
    end
    ROI_REQ:begin roi_address<={job_fid[HB-1:0],{AB{1'b0}}}+ry*GRID_W+rx;state<=ROI_WAIT;end
    ROI_WAIT:state<=ROI_TAKE;
    ROI_TAKE:begin
     red_sum<=red_sum+{11'd0,read_word[12:8]};roi_cells<=roi_cells+1'b1;
     if(rx==rxmax-1)begin rx<=rx0;if(ry==rymax-1)state<=DIV_START;else begin ry<=ry+1'b1;state<=ROI_REQ;end end
     else begin rx<=rx+1'b1;state<=ROI_REQ;end
    end
    DIV_START:begin numerator<=red_sum*1000;denominator<={12'd0,roi_cells,4'd0};divide_start<=1;state<=DIV_WAIT;end
    DIV_WAIT:if(divide_done)begin red_ratio<=quotient[15:0];state<=PUBLISH;end
    INVALID:begin track_ok<=0;observation_ok<=0;tracking_allowed<=0;force_yolo<=1;track_confidence<=0;red_ratio<=0;state<=PUBLISH;end
    PUBLISH:begin
     status_payload<={32'h46535441,8'd2,flags,reason,direction,job_fid,semantic_fid,class_id,confidence,x1,y1,x2,y2,red_ratio,track_confidence,semantic_age,16'd0};
     status_valid<=1;have_report<=1;reported_fid<=job_fid;state<=IDLE;
    end
   endcase
   if(state!=IDLE && state!=SEED_INIT && state!=INVALID && state!=PUBLISH &&
     (!present(job_fid) || (((state>=PAIR_INIT && state<=APPLY) || state==POINT_CHECK || state==FB_CHECK || state==SORT_READ || state==SORT_COMPARE) && !present(previous_fid))))begin history_miss<=1;reason<=5;state<=INVALID;end
   if(invalidate)begin
   active<=0;tracking_allowed<=0;track_ok<=0;observation_ok<=0;pending_seed<=0;
   history_miss<=1;force_yolo<=1;reason<=5;status_valid<=0;patch_start<=0;state<=INVALID;
   end
  end
 end
endmodule
