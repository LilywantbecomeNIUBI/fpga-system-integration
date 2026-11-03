// Full native-point fanout -> packed double-buffer grid. No host dependency.
// Frame coordinates are anchored at the body pose of the first accepted point.
module mc_nav_raymap(input wire clk,rst_n,enable,input wire [31:0] now_ms,
 input wire point_valid,input wire [15:0] angle_q100,distance_mm,input wire [7:0] intensity,
 input wire [31:0] error_count,input wire signed [31:0] pose_x_q16,pose_y_q16,heading_q16,
 input wire signed [15:0] laser_x_mm,laser_y_mm,input wire signed [31:0] laser_yaw_q16,
 input wire [15:0] front_mm,back_mm,half_width_mm,stop_margin_mm,
 input wire [31:0] self_start,self_end,self_distance,
 input wire grid_ready,cell_done,input wire [31:0] grid_epoch,input wire initializing,
 output reg scan_begin,scan_abort,publish,output wire cell_valid,
 output wire [13:0] cell_address,output wire [1:0] cell_code,
 output reg signed [31:0] anchor_x_mm,anchor_y_mm,anchor_heading,
 output reg signed [31:0] published_x_mm,published_y_mm,published_heading,
 output reg [31:0] metadata_epoch,
 output reg [31:0] overflow_count,output wire [7:0] danger_mask,output reg [31:0] last_point_ms
);
 localparam IDLE=0,FIFO_WAIT=1,POINT=2,BEGIN=3,CLEAR_WAIT=4,SELF=5,SELF_WAIT=6,
 ANGLE=7,ANGLE_SUM=8,TRIG_START=9,TRIG_WAIT=10,PX=11,PY=12,SUM=13,
 INDEX_START=14,INDEX_WAIT=15,RAY_INIT=16,CELL=17,CELL_WAIT=18,RAY_FLAGS=19,RAY_STEP=20,
 SEALED=21,SEAL_BEGIN=22,CLASSIFY=23,HAZARD=24,LXC=25,LYS=26,LXS=27,LYC=28,
 TXC=29,TYS=30,TXS=31,TYC=32,ORIGIN_SUM=33,RAY_TRIG=34,BODY_PX=35,BODY_PY=36,BODY_SUM=37,SELF_MASK_DECIDE=38,RAY_BOUNDS=39,RAY_COMMIT=40,HAZARD_FLAGS=41,HAZARD_COMMIT=42;
 reg [5:0] state;
 (* syn_ramstyle="block_ram" *) reg [167:0] fifo[0:31];
 reg [4:0] wp,rp;reg [5:0] fifo_count;reg [167:0] fifo_q,record;
 reg [7:0] danger_work,danger_published;
 assign danger_mask=danger_work | danger_published;
 reg [31:0] loss;wire push=point_valid && enable && fifo_count<32;
 wire pop=state==IDLE && enable && fifo_count!=0;
 wire [15:0] pa=record[15:0],pd=record[31:16];wire [7:0] pi=record[39:32];
 wire signed [15:0] ppx=record[55:40],ppy=record[71:56];
 wire signed [31:0] ph=record[103:72];wire [31:0] pe=record[135:104],pl=record[167:136];
 reg collecting,have_previous,scan_bad;reg [15:0] previous_angle;
 reg [31:0] base_error,base_loss;reg [31:0] sealed_epoch;
 reg signed [63:0] angle_product,product_x,product_y;
 reg signed [63:0] lxc,lys,lxs,lyc,txc,tys,txs,tyc;
 reg signed [31:0] anchor_cs,anchor_sn;reg [1:0] trig_mode;
 reg signed [31:0] hazard_x,hazard_y;
 reg self_hit;
 reg interior_hit,danger_band;reg [3:0] normal_flags;
 function automatic in_sector(input [15:0] a,lo,hi);
  if(lo<=hi)in_sector=a>=lo && a<=hi;else in_sector=a>=lo || a<=hi;
 endfunction
 reg signed [31:0] trig_angle,ray_angle,ox,oy,ex,ey,cs,sn;
 reg trig_start;wire trig_busy,trig_done;wire signed [31:0] cosine,sine;
 reg index_start;wire index_busy,index_done,index_outside;wire [6:0] index_result;
 wire signed [12:0] extended_index;
 reg signed [31:0] index_mm;reg [2:0] coordinate;
 reg [6:0] x0,y0,init_x,init_y;
 reg signed [12:0] x1,y1,cx,cy,next_cx,next_cy;
 reg endpoint_inside,ray_in_bounds;reg signed [14:0] dx,dy,err;reg signed [15:0] twice_err;
 reg signed [1:0] step_x,step_y;reg move_x,move_y,at_end;
 reg [6:0] self_x0,self_x1,self_y0,self_y1;
 wire wrap_angle=have_previous && pa<previous_angle && previous_angle-pa>18000;
 wire [16:0] angle_delta=pa>=previous_angle?pa-previous_angle:17'd36000+pa-previous_angle;
 function automatic signed [31:0] wrap(input signed [31:0] a);
  if(a>205887)wrap=a-411775;else if(a< -205887)wrap=a+411775;else wrap=a;
 endfunction
 mc_cordic_sincos trig(.clk(clk),.rst_n(rst_n),.start(trig_start),.angle_q16(trig_angle),.busy(trig_busy),.done(trig_done),.cos_q16(cosine),.sin_q16(sine));
 mc_nav_cell_index convert(.clk(clk),.rst_n(rst_n),.start(index_start),.mm(index_mm),.busy(index_busy),.done(index_done),.outside(index_outside),.index(index_result),.extended_index(extended_index));
 assign cell_valid=enable && (state==CELL || state==SELF);
 assign cell_address=state==SELF?{init_y,init_x}:{cy[6:0],cx[6:0]};
 assign cell_code=state==CELL && cx==x1 && cy==y1 && endpoint_inside?2'd2:2'd1;
 always @(posedge clk)begin
  if(push)fifo[wp]<={loss,error_count,heading_q16,pose_y_q16[31:16],pose_x_q16[31:16],intensity,distance_mm,angle_q100};
  if(pop)fifo_q<=fifo[rp];
 end
 reg signed [31:0] mul_a,mul_b;
 wire signed [63:0] mul_result=mul_a*mul_b;
 always @* begin
  mul_a=0;mul_b=0;
  case(state)
   ANGLE:begin mul_a=(pa>18000?$signed({1'b0,pa})-32'sd36000:$signed({1'b0,pa}));mul_b=32'sd11993645;end
   LXC:begin mul_a=$signed(laser_x_mm);mul_b=cs;end
   LYS:begin mul_a=$signed(laser_y_mm);mul_b=sn;end
   LXS:begin mul_a=$signed(laser_x_mm);mul_b=sn;end
   LYC:begin mul_a=$signed(laser_y_mm);mul_b=cs;end
   TXC:begin mul_a=($signed(ppx)-anchor_x_mm);mul_b=anchor_cs;end
   TYS:begin mul_a=($signed(ppy)-anchor_y_mm);mul_b=anchor_sn;end
   TXS:begin mul_a=($signed(ppx)-anchor_x_mm);mul_b=anchor_sn;end
   TYC:begin mul_a=($signed(ppy)-anchor_y_mm);mul_b=anchor_cs;end
   PX:begin mul_a=$signed({1'b0,pd});mul_b=cs;end
   PY:begin mul_a=$signed({1'b0,pd});mul_b=sn;end
   BODY_PX:begin mul_a=$signed({1'b0,pd});mul_b=cs;end
   BODY_PY:begin mul_a=$signed({1'b0,pd});mul_b=sn;end
   default:;
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;wp<=0;rp<=0;fifo_count<=0;loss<=0;overflow_count<=0;record<=0;scan_begin<=0;scan_abort<=0;publish<=0;
   collecting<=0;have_previous<=0;scan_bad<=1;previous_angle<=0;base_error<=0;base_loss<=0;sealed_epoch<=0;
   anchor_x_mm<=0;anchor_y_mm<=0;anchor_heading<=0;angle_product<=0;product_x<=0;product_y<=0;trig_angle<=0;ray_angle<=0;
   lxc<=0;lys<=0;lxs<=0;lyc<=0;txc<=0;tys<=0;txs<=0;tyc<=0;anchor_cs<=65536;anchor_sn<=0;trig_mode<=0;
   published_x_mm<=0;published_y_mm<=0;published_heading<=0;metadata_epoch<=0;
   hazard_x<=0;hazard_y<=0;
   self_hit<=0;
   interior_hit<=0;danger_band<=0;normal_flags<=0;
   ox<=0;oy<=0;ex<=0;ey<=0;cs<=0;sn<=0;trig_start<=0;index_start<=0;index_mm<=0;coordinate<=0;
   x0<=0;y0<=0;x1<=0;y1<=0;cx<=0;cy<=0;init_x<=0;init_y<=0;endpoint_inside<=0;
   dx<=0;dy<=0;err<=0;twice_err<=0;step_x<=0;step_y<=0;move_x<=0;move_y<=0;at_end<=0;
   self_x0<=0;self_x1<=0;self_y0<=0;self_y1<=0;danger_work<=0;last_point_ms<=0;danger_published<=0;
   next_cx<=0;next_cy<=0;ray_in_bounds<=0;
  end else begin
   scan_begin<=0;scan_abort<=0;publish<=0;trig_start<=0;index_start<=0;
   if(point_valid && enable)begin last_point_ms<=now_ms;if(!push)begin loss<=loss+1'b1;overflow_count<=overflow_count+1'b1;end end
   case({push,pop})2'b10:fifo_count<=fifo_count+1'b1;2'b01:fifo_count<=fifo_count-1'b1;default:;endcase
   if(push)wp<=wp+1'b1;if(pop)rp<=rp+1'b1;
   case(state)
    IDLE:if(pop)state<=FIFO_WAIT;
    FIFO_WAIT:begin record<=fifo_q;state<=POINT;end
    POINT:begin
     previous_angle<=pa;have_previous<=1;
     if(!collecting)begin if(wrap_angle)state<=BEGIN;else state<=IDLE;end
     else if(wrap_angle)begin
      if(!scan_bad && pe==base_error && pl==base_loss)begin publish<=1;sealed_epoch<=grid_epoch;state<=SEALED;end
      else begin scan_abort<=1;state<=BEGIN;end
     end else if(pe!=base_error || pl!=base_loss || angle_delta>2000)begin scan_bad<=1;scan_abort<=1;state<=IDLE;end
     else if(pd==0 || pi==0)state<=IDLE;
     else state<=ANGLE;
    end
    SEALED:if(grid_epoch!=sealed_epoch)begin published_x_mm<=anchor_x_mm;published_y_mm<=anchor_y_mm;published_heading<=anchor_heading;metadata_epoch<=grid_epoch;danger_published<=danger_work;state<=BEGIN;end
    BEGIN:begin
     scan_begin<=1;collecting<=1;scan_bad<=0;base_error<=pe;base_loss<=pl;danger_work<=0;
     anchor_x_mm<=$signed(ppx);anchor_y_mm<=$signed(ppy);anchor_heading<=ph;
     coordinate<=0;index_mm<=-$signed({1'b0,back_mm});state<=SEAL_BEGIN;
    end
    SEAL_BEGIN:state<=CLEAR_WAIT;
    CLEAR_WAIT:if(!initializing && grid_ready)begin trig_angle<=anchor_heading;trig_mode<=2;state<=TRIG_START;end
    // coordinates 4..7 initialize the current body rectangle before obstacle writes.
    INDEX_START:begin index_start<=1;state<=INDEX_WAIT;end
    INDEX_WAIT:if(index_done)begin
     case(coordinate)
      0:begin x0<=index_result;coordinate<=1;index_mm<=oy;if(index_outside)begin scan_bad<=1;scan_abort<=1;state<=IDLE;end else state<=INDEX_START;end
      1:begin y0<=index_result;if(index_outside)begin scan_bad<=1;scan_abort<=1;state<=IDLE;end else begin coordinate<=2;index_mm<=ex;state<=INDEX_START;end end
      2:begin x1<=extended_index;endpoint_inside<=!index_outside;coordinate<=3;index_mm<=ey;state<=INDEX_START;end
      3:begin y1<=extended_index;endpoint_inside<=endpoint_inside && !index_outside;state<=RAY_INIT;end
      4:begin self_x0<=index_result;coordinate<=5;index_mm<=$signed({1'b0,front_mm});state<=INDEX_START;end
      5:begin self_x1<=index_result;coordinate<=6;index_mm<=-$signed({1'b0,half_width_mm});state<=INDEX_START;end
      6:begin self_y0<=index_result;coordinate<=7;index_mm<=$signed({1'b0,half_width_mm});state<=INDEX_START;end
      7:begin self_y1<=index_result;init_x<=self_x0;init_y<=self_y0;state<=SELF;end
     endcase
    end
    SELF:if(grid_ready)state<=SELF_WAIT;
    SELF_WAIT:if(cell_done)begin
     if(init_x==self_x1)begin init_x<=self_x0;if(init_y==self_y1)state<=pd==0 || pi==0?IDLE:ANGLE;else begin init_y<=init_y+1'b1;state<=SELF;end end
     else begin init_x<=init_x+1'b1;state<=SELF;end
    end
    ANGLE:begin angle_product<=mul_result;state<=ANGLE_SUM;end
    ANGLE_SUM:begin ray_angle<=wrap(ph-anchor_heading);trig_angle<=wrap(ph-anchor_heading);trig_mode<=0;state<=TRIG_START;end
    TRIG_START:begin trig_start<=1;state<=TRIG_WAIT;end
    TRIG_WAIT:if(trig_done)begin
     cs<=cosine;sn<=sine;
     if(trig_mode==2)begin anchor_cs<=cosine;anchor_sn<=sine;coordinate<=4;index_start<=1;state<=INDEX_WAIT;end
     else if(trig_mode==0)state<=LXC;else if(trig_mode==3)state<=BODY_PX;else state<=PX;
    end
    LXC:begin lxc<=mul_result;state<=LYS;end
    LYS:begin lys<=mul_result;state<=LXS;end
    LXS:begin lxs<=mul_result;state<=LYC;end
    LYC:begin lyc<=mul_result;state<=TXC;end
    TXC:begin txc<=mul_result;state<=TYS;end
    TYS:begin tys<=mul_result;state<=TXS;end
    TXS:begin txs<=mul_result;state<=TYC;end
    TYC:begin tyc<=mul_result;state<=ORIGIN_SUM;end
    ORIGIN_SUM:begin ox<=((txc+tys)>>>16)+((lxc-lys)>>>16);oy<=((tyc-txs)>>>16)+((lxs+lyc)>>>16);state<=RAY_TRIG;end
    RAY_TRIG:begin trig_mode<=1;trig_angle<=wrap(ray_angle+laser_yaw_q16+(angle_product>>>20));state<=TRIG_START;end
    PX:begin product_x<=mul_result;state<=PY;end
    PY:begin product_y<=mul_result;state<=SUM;end
    SUM:begin
     ex<=ox+(product_x>>>16);ey<=oy+(product_y>>>16);
     trig_angle<=wrap(laser_yaw_q16+(angle_product>>>20));trig_mode<=3;state<=TRIG_START;
    end
    BODY_PX:begin product_x<=mul_result;state<=BODY_PY;end
    BODY_PY:begin product_y<=mul_result;state<=BODY_SUM;end
    BODY_SUM:begin hazard_x<=$signed(laser_x_mm)+(product_x>>>16);hazard_y<=$signed(laser_y_mm)+(product_y>>>16);state<=CLASSIFY;end
    CLASSIFY:begin
     self_hit<=hazard_x<$signed({1'b0,front_mm}) && hazard_x> -$signed({1'b0,back_mm}) &&
       hazard_y<$signed({1'b0,half_width_mm}) && hazard_y> -$signed({1'b0,half_width_mm}) &&
       ((self_distance[15:0]!=0 && pd<=self_distance[15:0] && in_sector(pa,self_start[15:0],self_end[15:0])) ||
        (self_distance[31:16]!=0 && pd<=self_distance[31:16] && in_sector(pa,self_start[31:16],self_end[31:16])));
     state<=SELF_MASK_DECIDE;
    end
    SELF_MASK_DECIDE:state<=self_hit?IDLE:HAZARD_FLAGS;
    HAZARD_FLAGS:begin
     interior_hit<=hazard_x<=$signed({1'b0,front_mm}) && hazard_x>=-$signed({1'b0,back_mm}) &&
       hazard_y<=$signed({1'b0,half_width_mm}) && hazard_y>=-$signed({1'b0,half_width_mm});
     danger_band<=hazard_x< $signed({1'b0,front_mm})+$signed({1'b0,stop_margin_mm}) && hazard_x> -$signed({1'b0,back_mm})-$signed({1'b0,stop_margin_mm}) &&
        hazard_y< $signed({1'b0,half_width_mm})+$signed({1'b0,stop_margin_mm}) && hazard_y> -$signed({1'b0,half_width_mm})-$signed({1'b0,stop_margin_mm});
     normal_flags<={hazard_y<=-$signed({1'b0,half_width_mm}),hazard_y>=$signed({1'b0,half_width_mm}),hazard_x<=-$signed({1'b0,back_mm}),hazard_x>=$signed({1'b0,front_mm})};
     state<=HAZARD_COMMIT;
    end
    HAZARD_COMMIT:begin
     // Unmasked returns inside the actual body cannot authorize escape.
     if(interior_hit)danger_work<=255;
     else if(danger_band)danger_work[3:0]<=danger_work[3:0] | normal_flags;
     index_mm<=ox;coordinate<=0;state<=INDEX_START;
    end
    RAY_INIT:begin
     cx<=$signed({1'b0,x0});cy<=$signed({1'b0,y0});dx<=x1>=$signed({1'b0,x0})?x1-$signed({1'b0,x0}):$signed({1'b0,x0})-x1;
     dy<=y1>=$signed({1'b0,y0})?-($signed(y1)-$signed({1'b0,y0})):-($signed({1'b0,y0})-$signed(y1));
     step_x<=x1>=$signed({1'b0,x0})?2'sd1:-2'sd1;step_y<=y1>=$signed({1'b0,y0})?2'sd1:-2'sd1;state<=HAZARD;
    end
    HAZARD:begin err<=dx+dy;state<=CELL;end
    CELL:if(grid_ready)state<=CELL_WAIT;
    CELL_WAIT:if(cell_done)begin twice_err<=$signed(err)<<<1;at_end<=cx==x1 && cy==y1;state<=RAY_FLAGS;end
    RAY_FLAGS:begin move_x<=twice_err>=dy;move_y<=twice_err<=dx;state<=RAY_STEP;end
    RAY_STEP:if(at_end)state<=IDLE;else begin
     next_cx<=cx+(move_x?step_x:2'sd0);next_cy<=cy+(move_y?step_y:2'sd0);
     err<=err+(move_x?dy:15'sd0)+(move_y?dx:15'sd0);state<=RAY_BOUNDS;
    end
    RAY_BOUNDS:begin ray_in_bounds<=next_cx>=0 && next_cx<128 && next_cy>=0 && next_cy<128;state<=RAY_COMMIT;end
    RAY_COMMIT:if(ray_in_bounds)begin cx<=next_cx;cy<=next_cy;state<=CELL;end else state<=IDLE;
    default:begin scan_bad<=1;scan_abort<=1;state<=IDLE;end
   endcase
   if(!enable)begin
    danger_published<=0;collecting<=0;have_previous<=0;state<=IDLE;fifo_count<=0;wp<=0;rp<=0;danger_work<=0;
    publish<=0;scan_begin<=0;scan_abort<=1;metadata_epoch<=0;last_point_ms<=0;
    trig_start<=0;index_start<=0;
   end
  end
 end
endmodule
