`timescale 1ns/1ps
module tb_i2c_filter;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0,scl_noise=0,sda_noise=0;tri1 scl,sda;wire [1:0] levels;
 assign scl=scl_noise ? 1'b0 : 1'bz;
 assign sda=sda_noise ? 1'b0 : 1'bz;
 imu_fast_i2c_master #(.CLK_HZ(50000000)) dut(
 .clk(clk),.rst_n(reset_n),.req(1'b0),.read_not_write(1'b1),.device_addr(7'h68),
 .reg_addr(8'h75),.write_data(8'd0),.read_length(6'd1),.scl(scl),.sda(sda),.line_levels(levels));
 initial begin
  repeat(10)@(negedge clk);reset_n=1;repeat(10)@(negedge clk);
  scl_noise=1;sda_noise=1;#40;scl_noise=0;sda_noise=0;
  repeat(20)begin @(negedge clk);if(levels!=3)$fatal(1,"accepted <=50ns noise spike");end
  scl_noise=1;sda_noise=1;#250;
  if(levels!=0)$fatal(1,"filtered legitimate low level");
  scl_noise=0;sda_noise=0;#250;
  if(levels!=3)$fatal(1,"did not recover high level");
  $display("PASS I2C filter: rejects 40ns glitches, accepts stable high/low bus levels at 50MHz");$finish;
 end
endmodule
