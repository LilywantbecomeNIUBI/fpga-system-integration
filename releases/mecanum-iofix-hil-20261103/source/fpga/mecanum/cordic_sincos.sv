`timescale 1ns/1ps
// 16-cycle rotation CORDIC. Angle is Q16.16 radians in [-pi,pi].
module mc_cordic_sincos(
 input wire clk,rst_n,start,
 input wire signed [31:0] angle_q16,
 output reg busy,done,
 output reg signed [31:0] cos_q16,sin_q16
);
 localparam signed [31:0] HALF_PI=32'sd102944;
 localparam signed [31:0] PI=32'sd205887;
 reg signed [31:0] x,y,z;
 reg invert;
 reg [4:0] i;
 function signed [31:0] lut;
 input [4:0] n;
 begin case(n)
 0:lut=51472;1:lut=30386;2:lut=16055;3:lut=8150;
 4:lut=4091;5:lut=2047;6:lut=1024;7:lut=512;
 8:lut=256;9:lut=128;10:lut=64;11:lut=32;
 12:lut=16;13:lut=8;14:lut=4;default:lut=2;
 endcase end endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   x<=0;y<=0;z<=0;i<=0;invert<=0;busy<=0;done<=0;
   cos_q16<=0;sin_q16<=0;
  end else begin
   done<=0;
   if(start && !busy) begin
    x<=32'sd39797;y<=0;i<=0;busy<=1;
    invert<=angle_q16>HALF_PI || angle_q16< -HALF_PI;
    if(angle_q16>HALF_PI) z<=angle_q16-PI;
    else if(angle_q16< -HALF_PI) z<=angle_q16+PI;
    else z<=angle_q16;
   end else if(busy) begin
    if(z>=0) begin
     x<=x-(y>>>i);y<=y+(x>>>i);z<=z-lut(i);
    end else begin
     x<=x+(y>>>i);y<=y-(x>>>i);z<=z+lut(i);
    end
    if(i==15) begin
     busy<=0;done<=1;
     // Include the final rotation in the reported result.
     if(z>=0) begin
      cos_q16<=invert ? -(x-(y>>>i)) : x-(y>>>i);
      sin_q16<=invert ? -(y+(x>>>i)) : y+(x>>>i);
     end else begin
      cos_q16<=invert ? -(x+(y>>>i)) : x+(y>>>i);
      sin_q16<=invert ? -(y-(x>>>i)) : y-(x>>>i);
     end
    end else i<=i+1'b1;
   end
  end
 end
endmodule
