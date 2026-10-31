`timescale 1ns/1ps
// 50MHz to 921600 baud, 8N1. Integer 54-clock bit period (0.46% error).
module stl27l_uart_rx #(parameter integer CLKS_PER_BIT=54)(input clk,input rst_n,input rx,output reg byte_valid,output reg[7:0] byte_data,output reg framing_error);
 reg rx_meta,rx_sync;reg[2:0] state;reg[7:0] sh;reg[3:0] bitn;reg[15:0] count;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin rx_meta<=1;rx_sync<=1;state<=0;count<=0;bitn<=0;sh<=0;byte_data<=0;byte_valid<=0;framing_error<=0;end else begin rx_meta<=rx;rx_sync<=rx_meta;byte_valid<=0;framing_error<=0;
   case(state)
    0:if(!rx_sync)begin state<=1;count<=CLKS_PER_BIT/2;end
    1:if(count==0)begin if(!rx_sync)begin state<=2;count<=CLKS_PER_BIT-1;bitn<=0;end else state<=0;end else count<=count-1;
    2:if(count==0)begin sh[bitn]<=rx_sync;count<=CLKS_PER_BIT-1;if(bitn==7)state<=3;else bitn<=bitn+1;end else count<=count-1;
    3:if(count==0)begin state<=0;if(rx_sync)begin byte_data<=sh;byte_valid<=1;end else framing_error<=1;end else count<=count-1;
    default:state<=0;
   endcase
  end
 end
endmodule
