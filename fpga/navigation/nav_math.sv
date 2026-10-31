// Serialized fixed-point arithmetic. No combinational /, sqrt, atan or wide multiplier.
// op: 0=signed 32x32, 1=signed 64/positive 32, 2=isqrt(unsigned 64),
//     3=atan2(b,a) in 0.01 degrees, 4=sincos(a in 0.01 degrees), Q16.
module nav_math(input wire clk,rst_n,start,input wire [2:0] op,
 input wire signed [63:0] a,input wire signed [31:0] b,
 output reg busy,done,error,output reg signed [63:0] result);
 reg [2:0] mode; reg [6:0] count; reg negative;
 reg [63:0] acc,n,den,remv,root,rootbit;
 reg [31:0] multiplier;
 reg signed [39:0] cx,cy;
 reg signed [31:0] cz;
 wire [64:0] trial={remv,n[63]};
 function automatic signed [31:0] atab(input [4:0] i);
  case(i)
   0:atab=294912000;1:atab=174096719;2:atab=91987925;3:atab=46694507;
   4:atab=23437865;5:atab=11730358;6:atab=5866610;7:atab=2933484;
   8:atab=1466764;9:atab=733385;10:atab=366693;11:atab=183346;
   12:atab=91673;13:atab=45837;14:atab=22918;15:atab=11459;
   16:atab=5730;17:atab=2865;18:atab=1432;default:atab=716;
  endcase
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin busy<=0;done<=0;error<=0;result<=0;mode<=0;count<=0;
   negative<=0;acc<=0;n<=0;den<=0;remv<=0;root<=0;rootbit<=0;
   multiplier<=0;cx<=0;cy<=0;cz<=0;end
  else begin
   done<=0;
   if(start && !busy) begin
    busy<=1;error<=0;mode<=op;count<=0;acc<=0;
    case(op)
     0:begin n<={32'b0,(a[31] ? (~a[31:0]+32'd1) : a[31:0])};
      multiplier<=b[31] ? -b : b;negative<=a[31]^b[31];end
     1:begin n<=a[63] ? -a : a;den<={32'b0,b};negative<=a[63];remv<=0;
      if(b<=0)begin result<=0;busy<=0;done<=1;error<=1;end end
     2:begin n<=a;root<=0;rootbit<=64'h4000000000000000;end
     3:begin
      // Inputs are mm coordinates, scaled before shifts to preserve precision.
      cx<=$signed(a[31:0])<<<16;cy<=$signed(b)<<<16;
      cz<=0;
      if(a[31])begin cx<=-($signed({{8{a[31]}},a[31:0]})<<<16);
       cy<=-($signed({{8{b[31]}},b})<<<16);cz<=b[31] ? -1179648000 : 1179648000;end
      if(a[31:0]==0 && b==0)begin result<=0;busy<=0;done<=1;end
     end
     4:begin
      // CORDIC gain compensated Q30, followed by quadrant reduction.
      cx<=40'sd652032874;cy<=0;cz<=$signed(a[31:0])<<<16;negative<=0;
      if($signed(a[31:0])>9000)begin cz<=($signed(a[31:0])-18000)<<<16;negative<=1;end
      if($signed(a[31:0])< -9000)begin cz<=($signed(a[31:0])+18000)<<<16;negative<=1;end
     end
     default:begin result<=0;error<=1;busy<=0;done<=1;end
    endcase
   end else if(busy) begin
    case(mode)
     0:begin
      if(multiplier[0])acc<=acc+n;
      n<=n<<1;multiplier<=multiplier>>1;count<=count+1'b1;
      if(count==31)begin result<=negative ? -(acc+(multiplier[0]?n:0)) : acc+(multiplier[0]?n:0);busy<=0;done<=1;end
     end
     1:begin
      n<=n<<1;count<=count+1'b1;
      if(trial>={1'b0,den})begin remv<=trial-den;acc<={acc[62:0],1'b1};end
      else begin remv<=trial;acc<=acc<<1;end
      if(count==63)begin result<=negative ? -$signed({acc[62:0],trial>={1'b0,den}}) : $signed({acc[62:0],trial>={1'b0,den}});busy<=0;done<=1;end
     end
     2:begin
      if(n>=root+rootbit)begin n<=n-root-rootbit;root<=(root>>1)+rootbit;end
      else root<=root>>1;
      rootbit<=rootbit>>2;count<=count+1'b1;
      if(count==31)begin result<=n>=root+rootbit ? (root>>1)+rootbit : root>>1;busy<=0;done<=1;end
     end
     3:begin
      if(count<20)begin
       if(cy>0)begin cx<=cx+(cy>>>count);cy<=cy-(cx>>>count);cz<=cz+atab(count[4:0]);end
       else if(cy<0)begin cx<=cx-(cy>>>count);cy<=cy+(cx>>>count);cz<=cz-atab(count[4:0]);end
       count<=count+1'b1;
      end else begin result<=cz>>>16;busy<=0;done<=1;end
     end
     4:begin
      if(count<20)begin
       if(cz>=0)begin cx<=cx-(cy>>>count);cy<=cy+(cx>>>count);cz<=cz-atab(count[4:0]);end
       else begin cx<=cx+(cy>>>count);cy<=cy-(cx>>>count);cz<=cz+atab(count[4:0]);end
       count<=count+1'b1;
      end else begin
       result<={negative ? -$signed({{6{cx[39]}},cx[39:14]}) : $signed({{6{cx[39]}},cx[39:14]}),negative ? -$signed({{6{cy[39]}},cy[39:14]}) : $signed({{6{cy[39]}},cy[39:14]})};
       busy<=0;done<=1;
      end
     end
    endcase
   end
  end
 end
endmodule
