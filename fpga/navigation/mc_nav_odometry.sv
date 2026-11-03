// Continuous mission frame, raw wheel counts -> mm Q16; one IMU owner upstream.
module mc_nav_odometry(input wire clk,rst_n,tick,origin,active,
 input wire [127:0] positions,input wire signed [31:0] yaw_q16,
 input wire [31:0] forward_scale_q16,lateral_scale_q16,
 output reg signed [31:0] x_q16,y_q16,heading_q16,
 output reg signed [15:0] velocity_x_mm_s,velocity_y_mm_s,output reg valid,overflow);
 reg [4:0] state;reg signed [31:0] previous[0:3],difference[0:3],yaw_origin;
 reg signed [33:0] forward_sum,lateral_sum;
 reg signed [63:0] forward_product,lateral_product,xc,ys,xs,yc;
 reg signed [31:0] forward_mm,lateral_mm,cs,sn;
 reg trig_start;wire trig_busy,trig_done;wire signed [31:0] cosine,sine;
 reg signed [63:0] next_x,next_y;integer i;
 reg signed [63:0] velocity_x_full,velocity_y_full;reg velocity_bad;
 function automatic signed [31:0] wrap(input signed [31:0] a);
  if(a>205887)wrap=a-411775;else if(a< -205887)wrap=a+411775;else wrap=a;
 endfunction
 mc_cordic_sincos trig(.clk(clk),.rst_n(rst_n),.start(trig_start),.angle_q16(heading_q16),.busy(trig_busy),.done(trig_done),.cos_q16(cosine),.sin_q16(sine));
 reg signed [31:0] mul_a,mul_b;
 wire signed [63:0] mul_result=mul_a*mul_b;
 always @* begin
  mul_a=0;mul_b=0;
  case(state)
   2:begin mul_a=forward_sum;mul_b=$signed({1'b0,forward_scale_q16});end
   3:begin mul_a=lateral_sum;mul_b=$signed({1'b0,lateral_scale_q16});end
   6:begin mul_a=forward_mm;mul_b=cs;end
   7:begin mul_a=lateral_mm;mul_b=sn;end
   8:begin mul_a=forward_mm;mul_b=sn;end
   9:begin mul_a=lateral_mm;mul_b=cs;end
   default:;
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;x_q16<=0;y_q16<=0;heading_q16<=0;valid<=0;overflow<=0;yaw_origin<=0;trig_start<=0;
   forward_sum<=0;lateral_sum<=0;forward_product<=0;lateral_product<=0;forward_mm<=0;lateral_mm<=0;cs<=0;sn<=0;
   xc<=0;ys<=0;xs<=0;yc<=0;next_x<=0;next_y<=0;
   velocity_x_mm_s<=0;velocity_y_mm_s<=0;
   velocity_x_full<=0;velocity_y_full<=0;velocity_bad<=0;
   for(i=0;i<4;i=i+1)begin previous[i]<=0;difference[i]<=0;end
  end else begin
   valid<=0;trig_start<=0;
   if(origin || !active)begin
    for(i=0;i<4;i=i+1)previous[i]<=$signed(positions[i*32+:32]);
    if(origin)begin x_q16<=0;y_q16<=0;yaw_origin<=yaw_q16;heading_q16<=0;overflow<=0;end
    state<=0;
   end else case(state)
    0:if(tick)begin
     for(i=0;i<4;i=i+1)begin difference[i]<=$signed(positions[i*32+:32])-previous[i];previous[i]<=$signed(positions[i*32+:32]);end
     heading_q16<=wrap(yaw_q16-yaw_origin);state<=1;
    end
    1:begin
     for(i=0;i<4;i=i+1)if(difference[i]>2048 || difference[i]< -2048)overflow<=1;
     forward_sum<=$signed({{2{difference[0][31]}},difference[0]})+$signed({{2{difference[1][31]}},difference[1]})+$signed({{2{difference[2][31]}},difference[2]})+$signed({{2{difference[3][31]}},difference[3]});
     lateral_sum<=-$signed({{2{difference[0][31]}},difference[0]})+$signed({{2{difference[1][31]}},difference[1]})+$signed({{2{difference[2][31]}},difference[2]})-$signed({{2{difference[3][31]}},difference[3]});state<=2;end
    2:begin forward_product<=mul_result;state<=3;end
    3:begin lateral_product<=mul_result;state<=4;end
    4:begin forward_mm<=forward_product>>>2;lateral_mm<=lateral_product>>>2;
     velocity_x_full<=((forward_product<<<6)+(forward_product<<<5)+(forward_product<<<2))>>>18;
     velocity_y_full<=((lateral_product<<<6)+(lateral_product<<<5)+(lateral_product<<<2))>>>18;state<=12;end
    12:begin velocity_bad<=velocity_x_full>32767 || velocity_x_full< -32768 || velocity_y_full>32767 || velocity_y_full< -32768;state<=13;end
    13:begin
     if(velocity_bad)begin overflow<=1;velocity_x_mm_s<=0;velocity_y_mm_s<=0;end
     else begin velocity_x_mm_s<=velocity_x_full[15:0];velocity_y_mm_s<=velocity_y_full[15:0];end
     trig_start<=1;state<=5;end
    5:if(trig_done)begin cs<=cosine;sn<=sine;state<=6;end
    6:begin xc<=mul_result;state<=7;end
    7:begin ys<=mul_result;state<=8;end
    8:begin xs<=mul_result;state<=9;end
    9:begin yc<=mul_result;state<=10;end
    10:begin next_x<=$signed(x_q16)+((xc-ys)>>>16);next_y<=$signed(y_q16)+((xs+yc)>>>16);state<=11;end
    11:begin
     if(next_x>1048576000 || next_x< -1048576000 || next_y>1048576000 || next_y< -1048576000)overflow<=1;
     else begin x_q16<=next_x[31:0];y_q16<=next_y[31:0];valid<=1;end
     state<=0;
    end
    default:begin overflow<=1;state<=0;end
   endcase
  end
 end
endmodule
