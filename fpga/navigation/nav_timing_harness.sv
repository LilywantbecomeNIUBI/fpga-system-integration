// STA HARNESS ONLY. Both virtual streams share J21 so no unconfirmed LiDAR pin
// is assigned. Encoder/IMU snapshots are injected with opcode 0x50. This is not
// the robot firmware, does not drive motors and must not be used for navigation.
module nav_timing_harness(input wire clk50m,reset_n,uart_rx,output wire uart_tx);
 reg r0,r1;
 always @(posedge clk50m or negedge reset_n)if(!reset_n)begin r0<=0;r1<=0;end else begin r0<=1;r1<=r0;end
 wire bv,fe,cv;wire [7:0] byte_data,opcode,sequence_id,flags;
 wire [15:0] a0,a1,a2,a3;wire [31:0] bad_frames;
 uart_rx_byte rx(.clk(clk50m),.reset_n(r1),.rx(uart_rx),.data(byte_data),.valid(bv),.framing_error(fe));
 motor_protocol protocol(.clk(clk50m),.reset_n(r1),.byte_valid(bv),.framing_error(fe),.byte_data(byte_data),
  .command_valid(cv),.opcode(opcode),.sequence_id(sequence_id),.flags(flags),.left_target(a0),.right_target(a1),.kp(a2),.ki(a3),.bad_frames(bad_frames));
 reg armed;reg signed [31:0] lp,rp;reg signed [15:0] gx,gy,gz;reg [15:0] div;
 reg iv;
 always @(posedge clk50m or negedge r1)begin
  if(!r1)begin armed<=0;lp<=0;rp<=0;gx<=0;gy<=0;gz<=0;div<=0;iv<=0;end
  else begin
   iv<=0;if(div==49999)begin div<=0;iv<=1;end else div<=div+1'b1;
   if(cv)case(opcode)
    0:armed<=0;1:armed<=1;
    8'h50:begin lp<=lp+$signed(a0);rp<=rp+$signed(a1);gx<=a2;gy<=a3;gz<=a0;end
   endcase
  end
 end
 wire lbv,lerr,pv;wire [7:0] lbyte,lint;wire [15:0] lang,ldist;wire [31:0] packets,errors;
 stl27l_uart_rx lidar_uart(.clk(clk50m),.rst_n(r1),.rx(uart_rx),.byte_valid(lbv),.byte_data(lbyte),.framing_error(lerr));
 stl27l_node_parser parser(.clk(clk50m),.rst_n(r1),.byte_valid(lbv),.byte_in(lbyte),.stream_error(lerr),
  .point_valid(pv),.distance_mm(ldist),.intensity(lint),.angle_q100(lang),.packet_count(packets),.error_count(errors));
 wire running,finished,permit,update;wire [15:0] faults,front;wire signed [15:0] lt,rt,v,w;
 wire signed [31:0] x,y,theta;wire [4:0] index;wire [31:0] rejected,plans;
 nav_core navigation(.clk(clk50m),.rst_n(r1),.left_position(lp),.right_position(rp),.armed(armed),
  .imu_valid(iv),.imu_bad(1'b0),.gx(gx),.gy(gy),.gz(gz),.point_valid(pv),.point_angle(lang),.point_distance(ldist),.point_intensity(lint),
  .command_valid(cv),.opcode(opcode),.sequence_id(sequence_id),.arg0(a0),.arg1(a1),.arg2(a2),.arg3(a3),
  .running(running),.mission_done(finished),.permit(permit),.faults(faults),.update(update),.left_target(lt),.right_target(rt),
  .x_q16(x),.y_q16(y),.theta_q16(theta),.v_mm_s(v),.w_mrad_s(w),.waypoint_index(index),.front_min(front),.rejected(rejected),.plan_count(plans));
 wire [383:0] frame;wire valid;
 nav_status_source status(.clk(clk50m),.rst_n(r1),.running(running),.mission_done(finished),.permit(permit),.waypoint_index(index),.faults(faults),
  .x_q16(x),.y_q16(y),.theta_q16(theta),.v(v),.w(w),.left_target(lt),.right_target(rt),.front_min(front),.plans(plans),.packets(packets),.errors(errors),.rejected(rejected),.frame(frame),.valid(valid));
 frame_uart_mux sender(.clk(clk50m),.reset_n(r1),.motor_valid(valid),.motor_frame(frame),.imu_valid(1'b0),.imu_frame(384'b0),.aux_valid(1'b0),.aux_frame(384'b0),.tx(uart_tx),.imu_drops(),.aux_drops());
endmodule
