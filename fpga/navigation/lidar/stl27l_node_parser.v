`timescale 1ns/1ps
// Rolling 46-byte window preserves embedded next headers after corruption.
module stl27l_node_parser #(
 parameter integer BYTE_TIMEOUT_CYCLES=100000,
 parameter integer MAX_SPAN_Q100=2000
)(
 input clk,input rst_n,input byte_valid,input [7:0] byte_in,
 output reg packet_valid,output reg point_valid,output reg packet_error,
 output reg [3:0] point_index,output reg [15:0] distance_mm,output reg [7:0] intensity,
 output reg [15:0] speed_dps,start_angle_q100,end_angle_q100,timestamp_ms,
 output reg [31:0] packet_count,error_count,
 input stream_error,
 output reg [15:0] angle_q100,
 output reg packet_timeout,output reg [31:0] timeout_count,crc_error_count,format_error_count
);
 reg [7:0] window[0:45];reg [5:0] fill;reg [7:0] crc;
 reg [31:0] idle_cycles;reg [15:0] pd[0:11];reg [7:0] pi[0:11];
 reg streaming;reg [3:0] emit_index;
 reg [15:0] emit_angle,step_angle;reg [3:0] remainder,error_acc;integer i;
 // One restoring-division bit per clock avoids a combinational /11 and %11.
 // A new 47-byte packet cannot arrive before these 17 clocks + 12 point clocks,
 // even when the byte interface supplies one byte on every clock.
 reg divide_busy;reg [4:0] divide_left;
 reg [16:0] divide_shift;reg [3:0] divide_remainder;
 wire [4:0] divide_trial={divide_remainder,divide_shift[16]};
 wire divide_take=(divide_trial>=5'd11);
 wire [4:0] divide_next_remainder=divide_take ? divide_trial-5'd11 : divide_trial;
 wire [15:0] candidate_start={window[5],window[4]};
 wire [15:0] candidate_end={window[43],window[42]};
 wire [16:0] span=(candidate_end>=candidate_start) ?
     {1'b0,candidate_end}-{1'b0,candidate_start} :
     17'd36000+{1'b0,candidate_end}-{1'b0,candidate_start};
 wire [4:0] next_error={1'b0,error_acc}+{1'b0,remainder};
 wire [16:0] next_angle={1'b0,emit_angle}+{1'b0,step_angle}+(next_error>=11);
 function [7:0] crc8;
  input [7:0] d;integer k;reg [7:0] c;
  begin c=d;for(k=0;k<8;k=k+1)c=c[7]?(c<<1)^8'h4d:c<<1;crc8=c;end
 endfunction
 // CRC contribution of the oldest byte followed by 45 zero bytes.
 function [7:0] remove_old;
  input [7:0] d;
  begin
   remove_old=({8{d[0]}}&8'hf9)^({8{d[1]}}&8'hbf)^
              ({8{d[2]}}&8'h33)^({8{d[3]}}&8'h66)^
              ({8{d[4]}}&8'hcc)^({8{d[5]}}&8'hd5)^
              ({8{d[6]}}&8'he7)^({8{d[7]}}&8'h83);
  end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   fill<=0;crc<=0;idle_cycles<=0;streaming<=0;emit_index<=0;
   packet_valid<=0;point_valid<=0;packet_error<=0;packet_timeout<=0;
   point_index<=0;distance_mm<=0;intensity<=0;angle_q100<=0;
   speed_dps<=0;start_angle_q100<=0;end_angle_q100<=0;timestamp_ms<=0;
   packet_count<=0;error_count<=0;timeout_count<=0;crc_error_count<=0;format_error_count<=0;
   emit_angle<=0;step_angle<=0;remainder<=0;error_acc<=0;
   divide_busy<=0;divide_left<=0;divide_shift<=0;divide_remainder<=0;
   for(i=0;i<46;i=i+1)window[i]<=0;
   for(i=0;i<12;i=i+1)begin pd[i]<=0;pi[i]<=0;end
  end else begin
   packet_valid<=0;point_valid<=0;packet_error<=0;packet_timeout<=0;
   if(divide_busy)begin
    divide_shift<={divide_shift[15:0],divide_take};
    divide_remainder<=divide_next_remainder[3:0];
    divide_left<=divide_left-1'b1;
    if(divide_left==1)begin
     // The /11 quotient fits in 16 bits for every 17-bit dividend.
     step_angle<={divide_shift[14:0],divide_take};
     remainder<=divide_next_remainder[3:0];error_acc<=5;
     divide_busy<=0;streaming<=1;
    end
   end
   if(streaming)begin
    point_valid<=1;point_index<=emit_index;distance_mm<=pd[emit_index];
    intensity<=pi[emit_index];angle_q100<=emit_angle;
    // Error accumulator gives round(i*span/11), no 12 separate dividers.
    emit_angle<=(next_angle>=36000)?next_angle-36000:next_angle;
    error_acc<=(next_error>=11)?next_error-11:next_error;
    if(emit_index==11)streaming<=0;else emit_index<=emit_index+1'b1;
   end
   if(stream_error)begin
    fill<=0;crc<=0;idle_cycles<=0;packet_error<=1;error_count<=error_count+1'b1;
   end else if(byte_valid)begin
    idle_cycles<=0;
    for(i=0;i<45;i=i+1)window[i]<=window[i+1];
    window[45]<=byte_in;
    if(fill<46)begin fill<=fill+1'b1;crc<=crc8(crc^byte_in);end
    else begin
     crc<=crc8(crc^remove_old(window[0])^byte_in);
     if(window[0]==8'h54 && window[1]==8'h2c)begin
      if(crc!=byte_in)begin
       packet_error<=1;error_count<=error_count+1'b1;crc_error_count<=crc_error_count+1'b1;
      end else begin
       fill<=0;crc<=0;
       if(candidate_start>=36000 || candidate_end>=36000 || span>MAX_SPAN_Q100)begin
        packet_error<=1;error_count<=error_count+1'b1;format_error_count<=format_error_count+1'b1;
       end else begin
        packet_valid<=1;packet_count<=packet_count+1'b1;
        speed_dps<={window[3],window[2]};start_angle_q100<=candidate_start;
        end_angle_q100<=candidate_end;timestamp_ms<={window[45],window[44]};
        for(i=0;i<12;i=i+1)begin pd[i]<={window[7+i*3],window[6+i*3]};pi[i]<=window[8+i*3];end
        // Keep receiving into window while dividing this accepted snapshot.
        // A subsequent UART error only resets reception, not accepted points.
        divide_busy<=1;divide_left<=17;divide_shift<=span;divide_remainder<=0;
        emit_index<=0;emit_angle<=candidate_start;
       end
      end
     end
    end
   end else if(fill!=0)begin
    if(idle_cycles>=BYTE_TIMEOUT_CYCLES-1)begin
     fill<=0;crc<=0;idle_cycles<=0;packet_error<=1;packet_timeout<=1;
     error_count<=error_count+1'b1;timeout_count<=timeout_count+1'b1;
    end else idle_cycles<=idle_cycles+1'b1;
   end
  end
 end
endmodule


