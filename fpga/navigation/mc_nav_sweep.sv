// Filled conservative rectangle sweep; one multiplier OR RAM read per state.
// Coordinates: millimetres relative to the immutable map anchor; radians Q16.
// Pad must cover cell half-diagonal, between-step motion and braking uncertainty.
module mc_nav_sweep #(parameter MAX_CYCLES=2500000)(
 input wire clk,rst_n,start,abort,input wire [31:0] map_epoch,
 input wire signed [31:0] x_mm,y_mm,theta_q16,
 input wire signed [15:0] step_x,step_y,input wire signed [31:0] step_theta,
 input wire [5:0] steps,input wire [15:0] front_mm,back_mm,half_width_mm,radius_mm,pad_mm,
 output reg busy,done,safe,timed_out,unknown_seen,output reg [31:0] unknown_cost,cycles,
 output wire read_valid,output wire [13:0] read_address,
 input wire read_ready,read_done,input wire [1:0] read_code
);
 localparam IDLE=0,TRIG_START=1,TRIG_WAIT=2,BOUND=3,INDEX_START=4,INDEX_WAIT=5,
 DIFF=6,XC=7,YS=8,SUMX=9,XS=10,YC=11,SUMY=12,FLAGS=13,SELECT=14,
 READ=15,WAIT_READ=16,COST_FLAGS=17,COST=18,ADVANCE=19,NEXT_STEP=20,FINISH=21,ANGLE_FLAGS=22,ANGLE_COMMIT=23;
 reg [4:0] state;reg [31:0] captured_epoch;
 reg signed [31:0] px,py,angle,sa;
 reg signed [32:0] next_angle;reg angle_high,angle_low;
 reg signed [15:0] sx,sy;
 reg [5:0] count,total;reg [15:0] front,back,width,radius,pad;
 reg trig_start;wire trig_busy,trig_done;wire signed [31:0] cosine,sine;
 reg signed [31:0] cs,sn,dx,dy,local_x,local_y;
 reg signed [63:0] pxc,pys,pxs,pyc;
 reg index_start;wire index_busy,index_done,index_outside;wire [6:0] index_result;
 reg signed [31:0] index_mm;reg [1:0] bound_index;
 reg [6:0] lo_x,hi_x,lo_y,hi_y,cx,cy;
 reg footprint_hit,occupied,unknown_cell,shape_outside;
 wire [15:0] expanded_radius=radius+pad;
 mc_cordic_sincos trig(.clk(clk),.rst_n(rst_n),.start(trig_start),.angle_q16(angle),
  .busy(trig_busy),.done(trig_done),.cos_q16(cosine),.sin_q16(sine));
 mc_nav_cell_index convert(.clk(clk),.rst_n(rst_n),.start(index_start),.mm(index_mm),
  .busy(index_busy),.done(index_done),.outside(index_outside),.index(index_result));
 assign read_valid=state==READ && busy;
 assign read_address={cy,cx};
 reg signed [31:0] mul_a,mul_b;
 wire signed [63:0] mul_result=mul_a*mul_b;
 always @* begin
  mul_a=0;mul_b=0;
  case(state)
   XC:begin mul_a=dx;mul_b=cs;end
   YS:begin mul_a=dy;mul_b=sn;end
   XS:begin mul_a=dx;mul_b=sn;end
   YC:begin mul_a=dy;mul_b=cs;end
   default:;
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;busy<=0;done<=0;safe<=0;timed_out<=0;unknown_seen<=0;unknown_cost<=0;cycles<=0;
   trig_start<=0;index_start<=0;captured_epoch<=0;px<=0;py<=0;angle<=0;sa<=0;sx<=0;sy<=0;
   next_angle<=0;angle_high<=0;angle_low<=0;
   count<=0;total<=0;front<=0;back<=0;width<=0;radius<=0;pad<=0;cs<=0;sn<=0;dx<=0;dy<=0;
   local_x<=0;local_y<=0;pxc<=0;pys<=0;pxs<=0;pyc<=0;index_mm<=0;bound_index<=0;
   lo_x<=0;hi_x<=0;lo_y<=0;hi_y<=0;cx<=0;cy<=0;footprint_hit<=0;occupied<=0;unknown_cell<=0;shape_outside<=0;
  end else begin
   done<=0;trig_start<=0;index_start<=0;
   if(busy)cycles<=cycles+1'b1;
   case(state)
    IDLE:if(start)begin
     captured_epoch<=map_epoch;px<=x_mm;py<=y_mm;angle<=theta_q16;sa<=step_theta;
     sx<=step_x;sy<=step_y;total<=steps;count<=0;front<=front_mm;back<=back_mm;
     width<=half_width_mm;radius<=radius_mm;pad<=pad_mm;busy<=1;safe<=1;timed_out<=0;
     unknown_seen<=0;unknown_cost<=0;cycles<=0;state<=TRIG_START;
     if(steps==0 || steps>32 || radius_mm>800 || pad_mm>464 || front_mm>radius_mm || back_mm>radius_mm || half_width_mm>radius_mm || theta_q16>205887 || theta_q16< -205887 || step_theta>205887 || step_theta< -205887)begin safe<=0;state<=FINISH;end
    end
    TRIG_START:begin trig_start<=1;state<=TRIG_WAIT;end
    TRIG_WAIT:if(trig_done)begin cs<=cosine;sn<=sine;bound_index<=0;state<=BOUND;end
    BOUND:begin
     case(bound_index)
      0:index_mm<=px-$signed({1'b0,expanded_radius});
      1:index_mm<=px+$signed({1'b0,expanded_radius});
      2:index_mm<=py-$signed({1'b0,expanded_radius});
      3:index_mm<=py+$signed({1'b0,expanded_radius});
     endcase
     state<=INDEX_START;
    end
    INDEX_START:begin index_start<=1;state<=INDEX_WAIT;end
    INDEX_WAIT:if(index_done)begin
     if(index_outside)begin safe<=0;state<=FINISH;end
     else begin
      case(bound_index)0:lo_x<=index_result;1:hi_x<=index_result;2:lo_y<=index_result;3:hi_y<=index_result;endcase
      if(bound_index==3)begin cx<=lo_x;cy<=lo_y;state<=DIFF;end
      else begin bound_index<=bound_index+1'b1;state<=BOUND;end
     end
    end
    DIFF:begin
     dx<=(($signed({1'b0,cx})-32'sd64)<<<5)+(($signed({1'b0,cx})-32'sd64)<<<4)+(($signed({1'b0,cx})-32'sd64)<<<1)+25-px;
     dy<=(($signed({1'b0,cy})-32'sd64)<<<5)+(($signed({1'b0,cy})-32'sd64)<<<4)+(($signed({1'b0,cy})-32'sd64)<<<1)+25-py;
     state<=XC;
    end
    XC:begin pxc<=mul_result;state<=YS;end
    YS:begin pys<=mul_result;state<=SUMX;end
    SUMX:begin local_x<=($signed(pxc)+$signed(pys))>>>16;state<=XS;end
    XS:begin pxs<=mul_result;state<=YC;end
    YC:begin pyc<=mul_result;state<=SUMY;end
    SUMY:begin local_y<=($signed(pyc)-$signed(pxs))>>>16;state<=FLAGS;end
    FLAGS:begin
     footprint_hit<=local_x<= $signed({1'b0,front})+$signed({1'b0,pad}) &&
       local_x>= -$signed({1'b0,back})-$signed({1'b0,pad}) &&
       local_y<= $signed({1'b0,width})+$signed({1'b0,pad}) && local_y>= -$signed({1'b0,width})-$signed({1'b0,pad});
     state<=SELECT;
    end
    SELECT:state<=footprint_hit?READ:ADVANCE;
    READ:if(read_ready)state<=WAIT_READ;
    WAIT_READ:if(read_done)begin occupied<=read_code[1];unknown_cell<=read_code==0;state<=COST_FLAGS;end
    COST_FLAGS:begin if(occupied)begin safe<=0;state<=FINISH;end else state<=COST;end
    COST:begin
     if(unknown_cell)begin unknown_seen<=1;if(unknown_cost!=32'hffffffff)unknown_cost<=unknown_cost+1'b1;end
     state<=ADVANCE;
    end
    ADVANCE:if(cx==hi_x)begin cx<=lo_x;if(cy==hi_y)state<=NEXT_STEP;else begin cy<=cy+1'b1;state<=DIFF;end end
            else begin cx<=cx+1'b1;state<=DIFF;end
    NEXT_STEP:if(count+1>=total)state<=FINISH;
     else begin count<=count+1'b1;px<=px+$signed(sx);py<=py+$signed(sy);next_angle<=$signed({angle[31],angle})+$signed({sa[31],sa});state<=ANGLE_FLAGS;end
    ANGLE_FLAGS:begin angle_high<=next_angle>205887;angle_low<=next_angle< -205887;state<=ANGLE_COMMIT;end
    ANGLE_COMMIT:begin angle<=angle_high?next_angle-33'sd411775:angle_low?next_angle+33'sd411775:next_angle;state<=TRIG_START;end
    FINISH:begin done<=1;busy<=0;state<=IDLE;end
    default:begin safe<=0;state<=FINISH;end
   endcase
   if(busy && state!=FINISH && (abort || map_epoch!=captured_epoch || cycles>=MAX_CYCLES-1))begin
    safe<=0;timed_out<=cycles>=MAX_CYCLES-1;state<=FINISH;
   end
  end
 end
endmodule
