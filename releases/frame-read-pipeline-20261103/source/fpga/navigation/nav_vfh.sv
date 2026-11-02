// Sequential VFH+ variant: min-range histogram, distance hysteresis, footprint
// enlargement, minimum-radius circle mask, valleys, target/current/previous cost.
module nav_vfh(input wire clk,rst_n,start,input wire [15:0] now_ms,
 input wire signed [15:0] target_heading,previous_heading,
 input wire [15:0] safe_radius,block_low,block_high,stop_mm,slow_mm,turn_radius,
 input wire [7:0] w_target,w_current,w_previous,valley_width,
 output reg [6:0] read_index,input wire [15:0] read_distance,input wire read_known,
 input wire [31:0] hazard_sequence,output reg clear_hazard,output reg [31:0] clear_sequence,
 output reg busy,done,found,front_known,rotation_clear,output reg [15:0] front_min,
 output reg signed [15:0] safe_heading,output reg [71:0] blocked,
 output reg [23:0] best_cost);
 reg [15:0] ranges[0:71];reg [71:0] observed,binary;
 reg [4:0] state;reg [6:0] i,j,spread,step;
 reg [15:0] d;reg [32:0] product;
 reg signed [31:0] px,py;
 reg known_q,hit_q;reg [15:0] input_distance;reg input_known;
 (* syn_preserve=1 *) reg [6:0] candidate_q;
 reg [15:0] delta_t,delta_c,delta_p;
 reg [23:0] cost_t,cost_c,cost_p;
 reg signed [47:0] xx,yy;
 reg [47:0] circle_bound;
 reg signed [15:0] left_limit,right_limit;
 reg [6:0] cursor,first,last,width,candidate,candidate_phase;
 reg in_valley;reg [23:0] trial_cost;
 reg signed [15:0] candidate_heading;
 reg [31:0] captured_sequence;
 function automatic [6:0] distbin(input [6:0] a,b);
  reg [6:0] t;begin t=a>b?a-b:b-a;distbin=t>36?72-t:t;end
 endfunction
 function automatic [15:0] adiff(input signed [15:0] a,b);
  reg signed [31:0] v;begin v=$signed(a)-$signed(b);if(v<0)v=-v;adiff=v>18000?36000-v:v;end
 endfunction
 function automatic [6:0] physical(input [6:0] x);
  physical=x>=18?x-18:x+54;
 endfunction
 function automatic signed [15:0] heading(input [6:0] x);
  heading=$signed({1'b0,x})*500-8750;
 endfunction
 function automatic [16:0] sin5(input [6:0] x);
 case(x)
  0:sin5=17'd0;
  1:sin5=17'd2856;
  2:sin5=17'd5690;
  3:sin5=17'd8481;
  4:sin5=17'd11207;
  5:sin5=17'd13848;
  6:sin5=17'd16384;
  7:sin5=17'd18795;
  8:sin5=17'd21063;
  9:sin5=17'd23170;
  10:sin5=17'd25102;
  11:sin5=17'd26842;
  12:sin5=17'd28378;
  13:sin5=17'd29698;
  14:sin5=17'd30792;
  15:sin5=17'd31651;
  16:sin5=17'd32270;
  17:sin5=17'd32643;
  18:sin5=17'd32768;
  default:sin5=32768;endcase
 endfunction
 function automatic signed [15:0] sintab(input [6:0] x);
 case(x)
  0:sintab=16'sd1429;
  1:sintab=16'sd4277;
  2:sintab=16'sd7092;
  3:sintab=16'sd9854;
  4:sintab=16'sd12540;
  5:sintab=16'sd15131;
  6:sintab=16'sd17606;
  7:sintab=16'sd19948;
  8:sintab=16'sd22138;
  9:sintab=16'sd24159;
  10:sintab=16'sd25997;
  11:sintab=16'sd27636;
  12:sintab=16'sd29066;
  13:sintab=16'sd30274;
  14:sintab=16'sd31251;
  15:sintab=16'sd31991;
  16:sintab=16'sd32488;
  17:sintab=16'sd32737;
  18:sintab=16'sd32737;
  19:sintab=16'sd32488;
  20:sintab=16'sd31991;
  21:sintab=16'sd31251;
  22:sintab=16'sd30274;
  23:sintab=16'sd29066;
  24:sintab=16'sd27636;
  25:sintab=16'sd25997;
  26:sintab=16'sd24159;
  27:sintab=16'sd22138;
  28:sintab=16'sd19948;
  29:sintab=16'sd17606;
  30:sintab=16'sd15131;
  31:sintab=16'sd12540;
  32:sintab=16'sd9854;
  33:sintab=16'sd7092;
  34:sintab=16'sd4277;
  35:sintab=16'sd1429;
  36:sintab=-16'sd1429;
  37:sintab=-16'sd4277;
  38:sintab=-16'sd7092;
  39:sintab=-16'sd9854;
  40:sintab=-16'sd12540;
  41:sintab=-16'sd15131;
  42:sintab=-16'sd17606;
  43:sintab=-16'sd19948;
  44:sintab=-16'sd22138;
  45:sintab=-16'sd24159;
  46:sintab=-16'sd25997;
  47:sintab=-16'sd27636;
  48:sintab=-16'sd29066;
  49:sintab=-16'sd30274;
  50:sintab=-16'sd31251;
  51:sintab=-16'sd31991;
  52:sintab=-16'sd32488;
  53:sintab=-16'sd32737;
  54:sintab=-16'sd32737;
  55:sintab=-16'sd32488;
  56:sintab=-16'sd31991;
  57:sintab=-16'sd31251;
  58:sintab=-16'sd30274;
  59:sintab=-16'sd29066;
  60:sintab=-16'sd27636;
  61:sintab=-16'sd25997;
  62:sintab=-16'sd24159;
  63:sintab=-16'sd22138;
  64:sintab=-16'sd19948;
  65:sintab=-16'sd17606;
  66:sintab=-16'sd15131;
  67:sintab=-16'sd12540;
  68:sintab=-16'sd9854;
  69:sintab=-16'sd7092;
  70:sintab=-16'sd4277;
  71:sintab=-16'sd1429;
  default:sintab=0;endcase
 endfunction
 function automatic signed [15:0] costab(input [6:0] x);
 case(x)
  0:costab=16'sd32737;
  1:costab=16'sd32488;
  2:costab=16'sd31991;
  3:costab=16'sd31251;
  4:costab=16'sd30274;
  5:costab=16'sd29066;
  6:costab=16'sd27636;
  7:costab=16'sd25997;
  8:costab=16'sd24159;
  9:costab=16'sd22138;
  10:costab=16'sd19948;
  11:costab=16'sd17606;
  12:costab=16'sd15131;
  13:costab=16'sd12540;
  14:costab=16'sd9854;
  15:costab=16'sd7092;
  16:costab=16'sd4277;
  17:costab=16'sd1429;
  18:costab=-16'sd1429;
  19:costab=-16'sd4277;
  20:costab=-16'sd7092;
  21:costab=-16'sd9854;
  22:costab=-16'sd12540;
  23:costab=-16'sd15131;
  24:costab=-16'sd17606;
  25:costab=-16'sd19948;
  26:costab=-16'sd22138;
  27:costab=-16'sd24159;
  28:costab=-16'sd25997;
  29:costab=-16'sd27636;
  30:costab=-16'sd29066;
  31:costab=-16'sd30274;
  32:costab=-16'sd31251;
  33:costab=-16'sd31991;
  34:costab=-16'sd32488;
  35:costab=-16'sd32737;
  36:costab=-16'sd32737;
  37:costab=-16'sd32488;
  38:costab=-16'sd31991;
  39:costab=-16'sd31251;
  40:costab=-16'sd30274;
  41:costab=-16'sd29066;
  42:costab=-16'sd27636;
  43:costab=-16'sd25997;
  44:costab=-16'sd24159;
  45:costab=-16'sd22138;
  46:costab=-16'sd19948;
  47:costab=-16'sd17606;
  48:costab=-16'sd15131;
  49:costab=-16'sd12540;
  50:costab=-16'sd9854;
  51:costab=-16'sd7092;
  52:costab=-16'sd4277;
  53:costab=-16'sd1429;
  54:costab=16'sd1429;
  55:costab=16'sd4277;
  56:costab=16'sd7092;
  57:costab=16'sd9854;
  58:costab=16'sd12540;
  59:costab=16'sd15131;
  60:costab=16'sd17606;
  61:costab=16'sd19948;
  62:costab=16'sd22138;
  63:costab=16'sd24159;
  64:costab=16'sd25997;
  65:costab=16'sd27636;
  66:costab=16'sd29066;
  67:costab=16'sd30274;
  68:costab=16'sd31251;
  69:costab=16'sd31991;
  70:costab=16'sd32488;
  71:costab=16'sd32737;
  default:costab=0;endcase
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;busy<=0;done<=0;found<=0;front_known<=0;rotation_clear<=0;
   read_index<=0;clear_hazard<=0;clear_sequence<=0;front_min<=0;safe_heading<=0;
   blocked<={72{1'b1}};observed<=0;binary<={72{1'b1}};best_cost<=24'hffffff;
   i<=0;j<=0;spread<=0;step<=0;d<=0;product<=0;px<=0;py<=0;xx<=0;yy<=0;
   circle_bound<=0;left_limit<=9000;right_limit<=-9000;cursor<=0;first<=0;last<=0;
   width<=0;candidate<=0;candidate_phase<=0;in_valley<=0;trial_cost<=0;candidate_heading<=0;captured_sequence<=0;known_q<=0;hit_q<=0;input_distance<=0;input_known<=0;candidate_q<=0;delta_t<=0;delta_c<=0;delta_p<=0;cost_t<=0;cost_c<=0;cost_p<=0;end
  else begin
   done<=0;clear_hazard<=0;
   case(state)
    0:if(start)begin busy<=1;found<=0;front_known<=1;rotation_clear<=1;front_min<=65535;
     best_cost<=24'hffffff;i<=0;read_index<=0;captured_sequence<=hazard_sequence;
     left_limit<=9000;right_limit<=-9000;blocked<=0;state<=1;end
    1:state<=2;
    2:begin input_distance<=read_distance;input_known<=read_known;state<=23;end
    23:begin
     ranges[i]<=input_distance;observed[i]<=input_known;
     if(!input_known || input_distance<=block_low)binary[i]<=1;
     else if(input_distance>=block_high)binary[i]<=0;
     if(!input_known || input_distance<=safe_radius)rotation_clear<=0;
     if(i<6 || i>=66)begin
      if(!input_known)front_known<=0;
      if(input_known && input_distance<front_min)front_min<=input_distance;
     end
     if(i==71)begin i<=0;state<=3;circle_bound<=(turn_radius+safe_radius)*(turn_radius+safe_radius);end
     else begin i<=i+1'b1;read_index<=i+1'b1;state<=1;end
    end
    3:begin d<=ranges[i];known_q<=observed[i];hit_q<=binary[i];state<=24;end
    24:begin
     if(!known_q)begin blocked[i]<=1;blocked[i==0?71:i-1'b1]<=1;blocked[i==71?0:i+1'b1]<=1;state<=10;end
     else if(hit_q)begin
      if(d<=safe_radius)begin blocked<={72{1'b1}};rotation_clear<=0;state<=10;end
      else begin
       px<=($signed({1'b0,d})*costab(i))>>>15;
       py<=($signed({1'b0,d})*sintab(i))>>>15;state<=4;
      end
     end else state<=10;
    end
    4:begin
     xx<=px*px;
     yy<=(py>=0 ? py-$signed({1'b0,turn_radius}) : py+$signed({1'b0,turn_radius})) *
         (py>=0 ? py-$signed({1'b0,turn_radius}) : py+$signed({1'b0,turn_radius}));
     state<=5;
    end
    5:begin
     if(turn_radius!=0 && xx+yy<circle_bound)begin
      if(py>0 && $signed(i*500+250)<left_limit)left_limit<=i*500+250;
      if(py<0 && $signed(i*500-35750)>right_limit)right_limit<=i*500-35750;
     end
     step<=1;state<=6;
    end
    6:begin product<=d*sin5(step);state<=7;end
    7:if(product>=({17'b0,safe_radius}<<15) || step==18)begin spread<=step+1'b1;j<=0;state<=8;end
      else begin step<=step+1'b1;state<=6;end
    8:begin
     if(distbin(i,j)<=spread)blocked[j]<=1;
     if(j==71)state<=10;else j<=j+1'b1;
    end
    9:begin
     if((j<36 ? $signed(j*500+250) : $signed(j*500-35750))>=left_limit ||
        (j<36 ? $signed(j*500+250) : $signed(j*500-35750))<=right_limit)blocked[j]<=1;
     if(j==71)begin cursor<=0;in_valley<=0;state<=11;end else j<=j+1'b1;
    end
    10:if(i==71)begin j<=0;state<=9;end else begin i<=i+1'b1;state<=3;end
    11:begin
     // Only forward travel; a differential chassis may rotate with v=0.
     if(cursor<36 && !blocked[physical(cursor)] && heading(cursor)>right_limit && heading(cursor)<left_limit)begin
      if(!in_valley)begin first<=cursor;in_valley<=1;end
      cursor<=cursor+1'b1;
     end else if(in_valley)begin
      last<=cursor-1'b1;width<=cursor-first;in_valley<=0;candidate_phase<=0;state<=12;
     end else if(cursor==36)state<=16;
     else cursor<=cursor+1'b1;
    end
    12:begin
     if(candidate_phase>=3)state<=11;
     else if(width<=valley_width)begin candidate<=(first+last)>>1;candidate_phase<=3;state<=13;end
     else if(candidate_phase==0)begin candidate<=first+(valley_width>>1);state<=13;end
     else if(candidate_phase==1)begin candidate<=last-(valley_width>>1);state<=13;end
     else if(candidate_phase==2)begin
      // The exact target is quantized to the sector containing it.
      if(target_heading>=heading(first+(valley_width>>1))-250 && target_heading<=heading(last-(valley_width>>1))+249)begin
       candidate<=first+(valley_width>>1);state<=15;
      end else state<=11;
     end else state<=11;
    end
    13:begin candidate_q<=candidate;state<=20;end
    20:begin candidate_heading<=heading(candidate_q);state<=21;end
    21:begin
     delta_t<=adiff(candidate_heading,target_heading);delta_c<=adiff(candidate_heading,0);delta_p<=adiff(candidate_heading,previous_heading);state<=22;
    end
    22:begin cost_t<=w_target*delta_t;cost_c<=w_current*delta_c;cost_p<=w_previous*delta_p;state<=25;end
    25:begin trial_cost<=cost_t+cost_c+cost_p;state<=14;end
    14:begin
     if(!found || trial_cost<best_cost)begin found<=1;best_cost<=trial_cost;safe_heading<=candidate_heading;end
     candidate_phase<=candidate_phase+1'b1;state<=12;
    end
    15:if(target_heading<heading(candidate)+250 || candidate>=last-(valley_width>>1))state<=13;else candidate<=candidate+1'b1;
    16:begin
     // The histogram re-checks the generation; a newer raw hazard wins.
     if(front_known && front_min>stop_mm)begin clear_hazard<=1;clear_sequence<=captured_sequence;end
     busy<=0;done<=1;state<=0;
    end
    default:begin busy<=0;done<=1;found<=0;state<=0;end
   endcase
  end
 end
endmodule
