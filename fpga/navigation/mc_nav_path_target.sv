// Extracted Task4 segment projection + bounded lookahead, no differential law.
// Uses the existing independently tested serialized nav_math implementation.
module mc_nav_path_target(input wire clk,rst_n,start,
 input wire signed [15:0] from_x,from_y,goal_x,goal_y,pose_x,pose_y,
 input wire [15:0] lookahead_mm,
 output reg busy,done,error,output reg signed [31:0] target_x,target_y,
 output reg [15:0] distance_mm);
 reg [4:0] state;reg math_start;reg [2:0] op;reg signed [63:0] a;
 reg signed [31:0] b,sx,sy,dx,dy,fx,fy,length,along;
 reg signed [63:0] hold;wire math_busy,math_done,math_error;wire signed [63:0] result;
 nav_math #(.OP_MASK(5'b00111)) math(.clk(clk),.rst_n(rst_n),.start(math_start),.op(op),.a(a),.b(b),.busy(math_busy),.done(math_done),.error(math_error),.result(result));
 task automatic calc(input [2:0] kind,input signed [63:0] lhs,input signed [31:0] rhs,input [4:0] next_state);
  begin op<=kind;a<=lhs;b<=rhs;math_start<=1;state<=next_state;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;busy<=0;done<=0;error<=0;target_x<=0;target_y<=0;distance_mm<=0;
   math_start<=0;op<=0;a<=0;b<=0;sx<=0;sy<=0;dx<=0;dy<=0;fx<=0;fy<=0;length<=0;along<=0;hold<=0;
  end else begin
   done<=0;math_start<=0;
   case(state)
    0:if(start)begin busy<=1;error<=0;sx<=$signed(goal_x)-$signed(from_x);sy<=$signed(goal_y)-$signed(from_y);
     dx<=$signed(pose_x)-$signed(from_x);dy<=$signed(pose_y)-$signed(from_y);fx<=from_x;fy<=from_y;state<=1;end
    1:calc(0,sx,sx,2);
    2:if(math_done)begin hold<=result;calc(0,sy,sy,3);end
    3:if(math_done)calc(2,hold+result,0,4);
    4:if(math_done)begin length<=result;if(result==0)begin target_x<=goal_x;target_y<=goal_y;state<=13;end else calc(0,dx,sx,5);end
    5:if(math_done)begin hold<=result;calc(0,dy,sy,6);end
    6:if(math_done)calc(1,hold+result,length,7);
    7:if(math_done)begin along<=result<0?lookahead_mm:result+lookahead_mm>length?length:result+lookahead_mm;state<=8;end
    8:calc(0,sx,along,9);
    9:if(math_done)calc(1,result,length,10);
    10:if(math_done)begin target_x<=fx+result;calc(0,sy,along,11);end
    11:if(math_done)calc(1,result,length,12);
    12:if(math_done)begin target_y<=fy+result;state<=13;end
    13:calc(0,$signed(goal_x)-$signed(pose_x),$signed(goal_x)-$signed(pose_x),14);
    14:if(math_done)begin hold<=result;calc(0,$signed(goal_y)-$signed(pose_y),$signed(goal_y)-$signed(pose_y),15);end
    15:if(math_done)calc(2,hold+result,0,16);
    16:if(math_done)begin distance_mm<=result;busy<=0;done<=1;state<=0;end
    default:begin busy<=0;done<=1;error<=1;state<=0;end
   endcase
   if(math_done && math_error)begin busy<=0;done<=1;error<=1;state<=0;end
  end
 end
endmodule
