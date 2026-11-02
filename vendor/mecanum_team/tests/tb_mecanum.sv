`timescale 1ns/1ps
module tb_mecanum;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0;
 reg [175:0] body=0;
 reg [3:0] a=0,b=0;
 wire [3:0] in1,in2;wire armed;wire [15:0] faults;
 wire [7:0] seq;wire [31:0] rejected;
 wire [127:0] pos,vel;wire [63:0] pwm,targets;
 // 100 clocks/window, 25 windows/watchdog; accelerated simulation.
 mecanum_core #(.CLK_HZ(100000),.CONTROL_HZ(1000),.PWM_HZ(1000),.STALL_TICKS(5),.REVERSE_TICKS(1),.FILTER_CYCLES(1)) dut(
  .clk(clk),.reset_n(reset_n),.enc_a(a),.enc_b(b),.command_valid(valid),.command_body(body),
  .in1(in1),.in2(in2),.armed(armed),.faults(faults),.last_sequence(seq),.rejected(rejected),
  .yaw_ok(1'b1),.yaw_q16(32'd0),.positions(pos),.velocities(vel),.applied(pwm),.targets(targets));
 task command(input [7:0] op,s,input signed [15:0] x,y,z);
 begin
  @(negedge clk);body=0;body[31:24]=op;body[39:32]=s;body[79:64]=x;body[95:80]=y;body[111:96]=z;body[127:112]=4096;if(op==5 || op==6)begin body[159:144]=128;body[175:160]=4;end
  valid=1;@(negedge clk);valid=0;repeat(2)@(negedge clk);
 end endtask
 task expect_targets(input signed [15:0] fl,fr,rl,rr);
 begin if(targets!=={rr,rl,fr,fl})$fatal(1,"wheel targets %h expected %h",targets,{rr,rl,fr,fl});end endtask
 task start_session;
 begin command(0,0,0,0,0);command(3,0,0,0,0);command(1,250,0,0,0);if(!armed || faults)$fatal(1,"arm");end endtask
 integer i,j;reg [31:0] old_rejected;
 initial begin
  #1;reset_n=0;repeat(4)@(negedge clk);reset_n=1;repeat(10)@(negedge clk);
  if(armed || faults!=1 || in1 || in2)$fatal(1,"boot");
  command(1,1,0,0,0);if(armed)$fatal(1,"armed with boot fault");
  command(3,0,0,0,0);
  @(negedge clk);body=0;body[31:24]=1;body[39:32]=1;body[127:112]=4096;body[40]=1;valid=1;
  @(negedge clk);valid=0;repeat(3)@(negedge clk);
  if(armed)$fatal(1,"runtime motor direction override accepted");
  @(negedge clk);body[40]=0;body[48]=1;valid=1;
  @(negedge clk);valid=0;repeat(3)@(negedge clk);
  if(armed)$fatal(1,"runtime encoder direction override accepted");
  start_session();
  old_rejected=rejected;
  command(2,251,100,0,0);
  for(i=0;i<4;i=i+1)command(4,251,100,i,0);
  command(5,251,20,0,0);
  expect_targets(0,0,0,0);
  if(rejected!=old_rejected+6 || seq!=250)$fatal(1,"removed modes accepted");
  command(6,251,6400,0,0);repeat(300)@(negedge clk);expect_targets(100,100,100,100);
  old_rejected=rejected;command(6,251,1,0,0);
  if(rejected!=old_rejected+1)$fatal(1,"duplicate accepted");
  command(6,123,1,0,0);if(rejected!=old_rejected+2)$fatal(1,"half-range replay accepted");
  command(6,252,16001,0,0);if(rejected!=old_rejected+3)$fatal(1,"position bound accepted");
  repeat(2600)@(negedge clk);
  if(armed || faults[7:4]!=15 || in1 || in2)$fatal(1,"four-wheel position stall/global stop %h",faults);
  command(0,0,0,0,0);if(armed || in1 || in2)$fatal(1,"stop");
  start_session();repeat(25100)@(negedge clk);
  if(armed || !faults[1] || in1 || in2)$fatal(1,"watchdog");
  // STOP does not clear a fault or auto-rearm.
  command(0,0,0,0,0);command(1,3,0,0,0);if(armed)$fatal(1,"fault auto restart");
  $display("PASS mecanum core: removed mode rejection, supported position, replay/bounds, four-wheel stalls, global stop, watchdog");$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
