`timescale 1ns/1ps
module tb_pwm_pad;
 reg clk=0;always #5 clk=~clk;reg reset_n=0,valid=0;reg [7:0] op=0,seq=0;reg signed [15:0] lt=0,rt=0;
 wire l1,l2,r1,r2,tx;wire [7:0] led;
 pwm_probe_top #(.CLK_HZ(10000000)) dut(.clk50m(clk),.reset_n(reset_n),.uart_rx(1'b1),.left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2),.uart_tx(tx),.led(led));
 task command(input [7:0] operation,input signed [15:0] target);begin @(negedge clk);op=operation;rt=target;seq=seq+1;valid=1;@(negedge clk);valid=0;repeat(5)@(negedge clk);end endtask
 reg [31:0] before_edges,before_logic,before_high;
 initial begin
  force dut.valid=valid;force dut.opcode=op;force dut.seq=seq;force dut.flags=8'h22;force dut.target_l=lt;force dut.target_r=rt;force dut.kp=16'd0;force dut.ki=16'd0;
  #1;repeat(5)@(negedge clk);reset_n=1;repeat(10)@(negedge clk);
  command(3,0);command(1,0);command(2,200);repeat(21000)@(negedge clk);
  if(dut.pad_edges_r<9||dut.pad_edges_r>11||dut.pad_high_r<1990||dut.pad_high_r>2010)$fatal(1,"pad readback duty");
  if(dut.pad_edges_l||dut.pad_high_l||l1||l2||!r2)$fatal(1,"unexpected other output");
  force r1=1;repeat(10)@(negedge clk);before_edges=dut.pad_edges_r;before_logic=dut.pulses_r;before_high=dut.pad_high_r;repeat(5000)@(negedge clk);
  if(dut.pad_edges_r!=before_edges||dut.pad_high_r!=before_high||dut.pulses_r<=before_logic||dut.applied_r!=200)$fatal(1,"pad observation fed back to PWM or not sensing pad");
  release r1;repeat(3000)@(negedge clk);if(dut.pad_edges_r<=before_edges)$fatal(1,"pad sense did not recover");
  command(0,0);repeat(10)@(negedge clk);if(l1||r1||dut.armed)$fatal(1,"STOP output");
  $display("PASS pad readback: real IOBUF.O net, 20 percent drive duty, externally clamped simulated pad does not change raw PWM, STOP");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
