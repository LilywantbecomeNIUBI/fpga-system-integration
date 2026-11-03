`timescale 1ns/1ps
module tb_fast_core;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,cv=0;reg [175:0] body=0;reg [1:0] phase=0;integer edge_div=0;
 reg [3:0] a=0,b=0;wire [3:0] in1,in2;wire armed;wire [15:0] faults;wire [7:0] seq;
 wire [127:0] positions,velocities;wire [63:0] applied,targets;
 mc_mecanum_core #(.CLK_HZ(1000000),.CONTROL_HZ(100),.CONTROL_SHIFT(6),.FAST_SPEED_LOOP(1),.FILTER_CYCLES(1)) dut(
 .clk(clk),.reset_n(rst),.enc_a(a),.enc_b(b),.command_valid(cv),.command_body(body),.yaw_ok(1'b1),.yaw_q16(32'sd0),
 .in1(in1),.in2(in2),.armed(armed),.faults(faults),.last_sequence(seq),.rejected(),.positions(positions),.velocities(velocities),.applied(applied),.targets(targets),
 .control_mode(),.arrived(),.imu_recalibrate(),.pose_x(),.pose_y(),.pose_heading());
 always @(negedge clk)if(rst)begin
  if(edge_div==49)begin edge_div=0;phase=phase+1;
   case(phase)0:begin a=0;b=0;end 1:begin a=0;b=15;end 2:begin a=15;b=15;end 3:begin a=15;b=0;end endcase
  end else edge_div=edge_div+1;
 end
 integer fast_count=0,outer_count=0,pi0=0,pi1=0,pi2=0,pi3=0;
 always @(posedge clk)if(rst)begin
 if(dut.fast_sample_tick)fast_count=fast_count+1;if(dut.control_tick)outer_count=outer_count+1;
 if(dut.wheels[0].fast.controller.state==7)pi0=pi0+1;
 if(dut.wheels[1].fast.controller.state==7)pi1=pi1+1;
 if(dut.wheels[2].fast.controller.state==7)pi2=pi2+1;
 if(dut.wheels[3].fast.controller.state==7)pi3=pi3+1;
 end
 task command(input [7:0] op,s,input signed [15:0] x);begin
 @(negedge clk);body=0;body[31:24]=op;body[39:32]=s;body[79:64]=x;body[127:112]=2048;body[143:128]=128;
 if(op==6 || op==9)begin body[159:144]=3;body[175:160]=4;end
 cv=1;@(negedge clk);cv=0;end endtask
 initial begin
 repeat(5)@(negedge clk);rst=1;command(3,1,0);command(1,2,0);command(6,3,10000);
 repeat(120000)@(negedge clk);
 if(faults || !armed || dut.speed_fault)$fatal(1,"unexpected fast-loop fault %h",faults);
 if(fast_count!=120 || outer_count!=12 || pi0<119 || pi0!=pi1 || pi0!=pi2 || pi0!=pi3)$fatal(1,"loop cadence mismatch fast=%d outer=%d PI=%d/%d/%d/%d",fast_count,outer_count,pi0,pi1,pi2,pi3);
 if(dut.wheels[0].fast.raw_velocity_q8<50560 || dut.wheels[0].fast.raw_velocity_q8>51840)$fatal(1,"M/T raw velocity units wrong %d",dut.wheels[0].fast.raw_velocity_q8);
 if($signed(velocities[31:0])<196 || $signed(velocities[31:0])>204)$fatal(1,"legacy raw telemetry units changed");
 command(0,4,0);repeat(3)@(negedge clk);if(armed || in1 || in2 || applied)$fatal(1,"STOP did not override 1kHz loops");
 $display("PASS four parallel 1kHz M/T+PI loops, 100Hz outer loop, raw telemetry retained, observer health and STOP");$finish;
 end
 initial begin #3000000;$fatal(1,"fast core timeout");end
endmodule
