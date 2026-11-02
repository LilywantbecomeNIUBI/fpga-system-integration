`timescale 1ns/1ps
module tb_chassis_routing;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,rx=1;wire [3:0] in1,in2;
 mecanum_top #(.CLK_HZ(1000000),.BAUD(100000),.CONTROL_HZ(1000),.IMU_ENABLED(0)) dut(
 .clk50m(clk),.reset_n(reset_n),.uart_rx(rx),.enc_a(4'b0),.enc_b(4'b0),.in1(in1),.in2(in2));
 function automatic [15:0] crc(input [175:0] bytes);
 reg [15:0] c;integer i,j;
 begin c=16'hffff;for(i=0;i<22;i=i+1)begin c=c^{bytes[i*8+:8],8'b0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;end crc=c;end
 endfunction
 task command(input [7:0] op,seq,input [15:0] speed,wheel);
 reg [191:0] p;integer i,j;
 begin
  p=0;p[15:0]=16'h5aa5;p[23:16]=3;p[31:24]=op;p[39:32]=seq;
  p[79:64]=speed;p[95:80]=wheel;p[127:112]=256;p[191:176]=crc(p[175:0]);
  for(i=0;i<24;i=i+1)begin
   @(negedge clk);rx=0;repeat(10)@(negedge clk);
   for(j=0;j<8;j=j+1)begin rx=p[i*8+j];repeat(10)@(negedge clk);end
   rx=1;repeat(10)@(negedge clk);
  end
  repeat(50)@(negedge clk);
 end endtask
 integer wheel;reg [31:0] old_rejected;
 initial begin
  repeat(10)@(negedge clk);reset_n=1;repeat(100)@(negedge clk);
  command(3,0,0,0);command(1,1,0,0);
  old_rejected=dut.rejected;
  command(2,2,20,0);command(5,2,20,0);
  for(wheel=0;wheel<4;wheel=wheel+1)command(4,2,20,wheel);
  repeat(22000)@(negedge clk);
  if(dut.rejected!=old_rejected+6 || dut.target!=0 || in1 || in2 || dut.core.last_sequence!=1)
   $fatal(1,"legacy UART drive/single-wheel command was accepted");
  command(0,3,0,0);if(in1 || in2)$fatal(1,"STOP outputs");
  $display("PASS UART rejects removed drive, heading and all single-wheel selectors without moving or advancing sequence");$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
