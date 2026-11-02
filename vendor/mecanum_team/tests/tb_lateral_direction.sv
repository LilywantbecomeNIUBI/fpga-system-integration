`timescale 1ns/1ps
module tb_lateral_direction;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0;reg [175:0] body=0;reg [3:0] a=0,b=0;
 wire armed;wire [15:0] faults;wire [63:0] target;wire signed [31:0] px,py;
 mecanum_core #(.CLK_HZ(1000000),.CONTROL_HZ(1000),.CONTROL_SHIFT(4),.LATERAL_START_PWM(150),.LATERAL_REVERSE(0)) dut(
 .clk(clk),.reset_n(reset_n),.enc_a(a),.enc_b(b),.command_valid(valid),.command_body(body),
 .yaw_ok(1'b1),.yaw_q16(32'd0),.armed(armed),.faults(faults),.targets(target),.pose_x(px),.pose_y(py));
 task command(input [7:0] op,seq,input signed [15:0] y);
 begin
  @(negedge clk);body=0;body[31:24]=op;body[39:32]=seq;body[95:80]=y;body[127:112]=256;
  if(op==5 || op==6)begin body[159:144]=20;body[175:160]=4;end
  valid=1;@(negedge clk);valid=0;
 end endtask
 task left_encoder_cycle;
 begin
  a=9;b=6;repeat(40)@(negedge clk);a=15;b=15;
  repeat(40)@(negedge clk);a=6;b=9;
  repeat(40)@(negedge clk);a=0;b=0;repeat(40)@(negedge clk);
 end endtask
 initial begin
  repeat(5)@(negedge clk);reset_n=1;
  command(3,0,0);command(1,1,0);command(6,2,320);
  repeat(20000)@(negedge clk);
  if($signed(target[0+:16])>=0 || $signed(target[16+:16])<=0 ||
     $signed(target[32+:16])<=0 || $signed(target[48+:16])>=0)$fatal(1,"+Y physical wheel mapping");
  repeat(20)left_encoder_cycle();repeat(3000)@(negedge clk);
  if(py<=0 || px>65536 || px< -65536 || !armed || faults)$fatal(1,"left feedback must increase logical Y without forward drift");
  command(0,3,0);command(1,10,0);command(6,11,-320);repeat(15000)@(negedge clk);
  if($signed(target[0+:16])<=0 || $signed(target[16+:16])>=0 ||
     $signed(target[32+:16])>=0 || $signed(target[48+:16])<=0)$fatal(1,"-Y physical wheel mapping");
  command(0,13,0);
  $display("PASS physical lateral calibration: +Y left targets and odometry, -Y right targets");$finish;
 end
endmodule
