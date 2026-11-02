// 48-byte C3 3C 04 frame; little-endian CRC16. Layout is in tools/task4/protocol.py.
module nav_status_source #(parameter CLK_HZ=50000000)(input wire clk,rst_n,
 input wire running,mission_done,permit,input wire [4:0] waypoint_index,input wire [15:0] faults,
 input wire signed [31:0] x_q16,y_q16,theta_q16,input wire signed [15:0] v,w,left_target,right_target,
 input wire [15:0] front_min,input wire [31:0] plans,packets,errors,rejected,
 output wire [383:0] frame,output wire valid);
 reg [31:0] divider;reg [367:0] body;reg capture;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin divider<=0;body<=0;capture<=0;end else begin
   capture<=0;
   if(divider==CLK_HZ/10-1)begin
    divider<=0;capture<=1;
    body<={rejected,errors,packets,plans,front_min,right_target,left_target,w,v,theta_q16,y_q16,x_q16,
      faults,{3'b0,waypoint_index},{5'b0,permit,mission_done,running},8'h01,8'h04,16'h3cc3};
   end else divider<=divider+1'b1;
  end
 end
 frame_crc crc(.clk(clk),.reset_n(rst_n),.capture(capture),.body_in(body),.frame(frame),.valid(valid));
endmodule
