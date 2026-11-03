`timescale 1ns/1ps
module tb_pipeline_pose;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,new_goal=0,active=1;reg signed [31:0] yaw=0;
 reg [127:0] positions=0;wire va,vb,aa,ab,oa,ob;
 wire signed [15:0] ax,ay,az,bx,by,bz;wire signed [31:0] apx,apy,ah,bpx,bpy,bh;
 mc_mecanum_pose_control #(.EXTENDED_TURNS(1),.LATERAL_REVERSE(1)) a(.clk(clk),.reset_n(rst),.tick(tick),.active(active),.new_goal(new_goal),
 .mode(2'd2),.yaw_ok(1'b1),.yaw_q16(yaw),.wheel_positions(positions),.wheel_velocities(128'd0),.goal_x(16'sd10000),.goal_y(-16'sd10000),
 .goal_centideg(32'sd36000),.continuous_turn(1'b1),.speed_limit(16'd6),.position_tolerance(16'd4),
 .command_x(ax),.command_y(ay),.command_z(az),.command_valid(va),.arrived(aa),.overflow(oa),.x_q16(apx),.y_q16(apy),.heading_q16(ah));
 reference_pose_control #(.EXTENDED_TURNS(1),.LATERAL_REVERSE(1)) b(.clk(clk),.reset_n(rst),.tick(tick),.active(active),.new_goal(new_goal),
 .mode(2'd2),.yaw_ok(1'b1),.yaw_q16(yaw),.wheel_positions(positions),.wheel_velocities(128'd0),.goal_x(16'sd10000),.goal_y(-16'sd10000),
 .goal_centideg(32'sd36000),.continuous_turn(1'b1),.speed_limit(16'd6),.position_tolerance(16'd4),
 .command_x(bx),.command_y(by),.command_z(bz),.command_valid(vb),.arrived(ab),.overflow(ob),.x_q16(bpx),.y_q16(bpy),.heading_q16(bh));
 reg seen_a=0,seen_b=0;integer i,j,n,seed=32'h57262311,checks=0;
 always @(posedge clk)begin #1;if(va)seen_a=1;if(vb)seen_b=1;end
 initial begin
  repeat(4)@(negedge clk);rst=1;
  positions={4{32'h7ffffff0}};new_goal=1;@(negedge clk);new_goal=0;
  repeat(3)@(negedge clk);
  for(i=0;i<600;i=i+1)begin
   @(negedge clk);seen_a=0;seen_b=0;
   for(j=0;j<4;j=j+1)positions[j*32+:32]=positions[j*32+:32]+(($random(seed)&63)-32);
   yaw=(i%400)*900-180000;tick=1;
   @(negedge clk);tick=0;
   // Inputs keep moving after the tick: arithmetic must use its snapshot.
   for(j=0;j<4;j=j+1)positions[j*32+:32]=positions[j*32+:32]+j+1;
   n=0;while((!seen_a || !seen_b) && n<100)begin @(negedge clk);n=n+1;end
   if(n==100)$fatal(1,"pose pipeline did not complete");
   if({ax,ay,az,aa,oa,apx,apy,ah}!=={bx,by,bz,ab,ob,bpx,bpy,bh})
    $fatal(1,"pose sample mismatch %0d xy=%0d/%0d expected=%0d/%0d",i,apx,apy,bpx,bpy);
   checks=checks+1;
   if(i%101==0)begin new_goal=1;@(negedge clk);new_goal=0;repeat(4)@(negedge clk);end
  end
  // Cancel immediately after the snapshot, before pair sums can publish.
  @(negedge clk);tick=1;@(negedge clk);tick=0;active=0;
  repeat(5)@(negedge clk);if(va || ax || ay || az)$fatal(1,"inactive pose leaked command");
  $display("PASS pose pipeline: %0d bit-exact tick snapshots, changing live positions, signed/wrap/yaw/reversed-lateral/new-goal and cancellation",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"pose pipeline timeout");end
endmodule
