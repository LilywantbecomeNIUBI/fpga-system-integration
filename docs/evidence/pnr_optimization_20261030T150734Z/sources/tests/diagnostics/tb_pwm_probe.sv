`timescale 1ns/1ps
module tb_pwm_probe;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,valid=0;reg [7:0] op=0,seq=0,flags=34;
 reg signed [15:0] lt=0,rt=0;wire armed;wire [15:0] faults;wire [7:0] last_seq;
 wire signed [15:0] la,ra;wire [31:0] rejected;wire l1,l2,r1,r2;
 pwm_probe_core #(.CLK_HZ(1000000)) core(.clk(clk),.reset_n(reset_n),.command_valid(valid),.opcode(op),.sequence_id(seq),.command_flags(flags),.left_target(lt),.right_target(rt),.kp(16'd0),.ki(16'd0),.armed(armed),.faults(faults),.last_sequence(last_seq),.left_applied(la),.right_applied(ra),.rejected_commands(rejected));
 pwm_probe_output #(.CLK_HZ(1000000),.PWM_HZ(1000)) lp(.clk(clk),.reset_n(reset_n),.enable(armed),.duty_permille(la),.in1(l1),.in2(l2));
 pwm_probe_output #(.CLK_HZ(1000000),.PWM_HZ(1000)) rp(.clk(clk),.reset_n(reset_n),.enable(armed),.duty_permille(ra),.in1(r1),.in2(r2));
 task cycles(input integer n);begin repeat(n)@(negedge clk);end endtask
 task command(input [7:0] opcode,input signed [15:0] l,r);begin @(negedge clk);op=opcode;lt=l;rt=r;seq=seq+1;valid=1;@(negedge clk);valid=0;end endtask
 integer i,highs,start_at,deadline;integer clocks=0;
 always @(posedge clk)begin clocks<=clocks+1;if(armed&&((la!=0&&!l2)||(ra!=0&&!r2)))$fatal(1,"IN2 not held high");if(!armed&&(l1||l2||r1||r2))$fatal(1,"unarmed output");end
 initial begin
  #1;cycles(3);reset_n=1;cycles(3);command(1,0,0);if(armed)$fatal(1,"boot lock bypass");
  command(3,0,0);flags=1;command(1,0,0);if(armed)$fatal(1,"normal host could arm diagnostic");flags=34;command(1,0,0);if(!armed)$fatal(1,"diagnostic ARM");
  command(2,0,201);if(ra!=0)$fatal(1,"limit bypass");command(2,100,100);if(la||ra)$fatal(1,"two-wheel command accepted");
  command(2,0,200);start_at=clocks;
  cycles(500);if(!r1||!r2)$fatal(1,"wake not brake");cycles(1500);highs=0;repeat(1000)begin @(negedge clk);if(!r1&&r2)highs=highs+1;if(l1)$fatal(1,"left moved");end
  if(highs!=200)$fatal(1,"PWM duty %d",highs);
  for(i=0;i<104;i=i+1)begin cycles(48000);command(2,0,200);end
  while(armed)@(negedge clk);deadline=clocks-start_at;if(deadline<4999990||deadline>5000010)$fatal(1,"extended one-shot %d",deadline);
  if(la||ra||l1||r1||faults)$fatal(1,"one-shot STOP");command(2,0,200);if(armed||ra)$fatal(1,"automatic rearm");
  command(1,0,0);command(2,200,0);cycles(2000);command(0,0,0);if(armed||l1||r1)$fatal(1,"STOP did not kill PWM");
  command(1,0,0);command(2,0,200);cycles(250010);if(armed||!faults[1]||l1||r1)$fatal(1,"watchdog");
  $display("PASS raw PWM: no feedback ports, normal-host rejection, slow decay 20 percent exact drive duty, one wheel only, 5000ms immutable one-shot, immediate STOP, watchdog, no auto rearm");$finish;
 end
 initial begin #70000000;$fatal(1,"timeout");end
endmodule
