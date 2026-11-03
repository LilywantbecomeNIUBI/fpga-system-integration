`timescale 1ns/1ps
module tb_pwm_width_equivalence;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,enable=0;reg signed [15:0] command=0;
 wire [3:0] old_pins,new_pins;
 mc_at8236_pwm #(.CLK_HZ(50000000),.PWM_HZ(10000)) n0(clk,rst,enable,command,new_pins[0],new_pins[1]);
 mc_at8236_pwm_before_width #(.CLK_HZ(50000000),.PWM_HZ(10000)) b0(clk,rst,enable,command,old_pins[0],old_pins[1]);
 mc_at8236_pwm #(.CLK_HZ(100000),.PWM_HZ(1000)) n1(clk,rst,enable,command,new_pins[2],new_pins[3]);
 mc_at8236_pwm_before_width #(.CLK_HZ(100000),.PWM_HZ(1000)) b1(clk,rst,enable,command,old_pins[2],old_pins[3]);
 integer cycles=0;reg [31:0] seed=32'h50484d53;
 always @(negedge clk)if(rst)begin
  if(new_pins!==old_pins || n0.phase!==b0.phase || n0.duty!==b0.duty || n0.wake_count!==b0.wake_count || n1.phase!==b1.phase || n1.duty!==b1.duty || n1.wake_count!==b1.wake_count)
   $fatal(1,"PWM cycle/phase/wake mismatch cycle%0d",cycles);
  cycles=cycles+1;
 end
 initial begin
  repeat(4)@(negedge clk);rst=1;enable=1;command=300;
  repeat(70000)@(negedge clk);
  command=-300;repeat(70000)@(negedge clk);
  command=-32768;repeat(60000)@(negedge clk);
  command=32767;repeat(60000)@(negedge clk);
  for(integer i=0;i<1000;i=i+1)begin
   @(posedge clk);#1;seed=seed^(seed<<13);seed=seed^(seed>>17);seed=seed^(seed<<5);
   command=seed;enable=seed[31];#1;
   if(new_pins!==old_pins)$fatal(1,"immediate gate changed");
   if(!enable && new_pins!==0)$fatal(1,"disable delayed");
   repeat(17)@(negedge clk);
  end
  #1;rst=0;#1;if(new_pins!==0 || old_pins!==0)$fatal(1,"async reset delayed");
  $display("PASS PWM width optimization %0d cycle-exact checks at real50MHz/10kHz and fast test clock; immediate disable/reset unchanged",cycles);
  $finish;
 end
 initial begin #10000000;$fatal(1,"PWM width watchdog");end
endmodule
