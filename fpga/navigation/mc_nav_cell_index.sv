// Signed floor((mm+3200)/50), bounded 33-cycle restoring division.
// Rays retain signed endpoints: independent x/y clipping changes ray slope.
module mc_nav_cell_index(input wire clk,rst_n,start,input wire signed [31:0] mm,
 output reg busy,done,outside,output reg [6:0] index,
 output reg signed [12:0] extended_index);
 reg [32:0] numerator,quotient;reg [6:0] remainder;reg [5:0] bits_left;
 reg negative,below,above;
 wire signed [32:0] shifted=$signed({mm[31],mm})+33'sd3200;
 wire [7:0] trial={remainder,numerator[32]};wire take=trial>=50;
 wire [32:0] next_q={quotient[31:0],take};
 wire [7:0] next_r=take?trial-8'd50:trial;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;outside<=0;index<=0;extended_index<=0;numerator<=0;quotient<=0;remainder<=0;bits_left<=0;negative<=0;below<=0;above<=0;end
  else begin
   done<=0;
   if(start && !busy)begin
    outside<=mm< -3200 || mm>=3200;below<=mm< -3200;above<=mm>=3200;
    negative<=shifted<0;numerator<=shifted<0?-$signed(shifted):shifted;
    quotient<=0;remainder<=0;bits_left<=33;busy<=1;
   end else if(busy)begin
    numerator<=numerator<<1;quotient<=next_q;remainder<=next_r;bits_left<=bits_left-1'b1;
    if(bits_left==1)begin
     extended_index<=negative?-$signed({1'b0,next_q[11:0]})-(next_r!=0):$signed(next_q[12:0]);
     index<=below?7'd0:above?7'd127:next_q[6:0];busy<=0;done<=1;
    end
   end
  end
 end
endmodule
