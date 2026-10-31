`timescale 1ns/1ps
module tb_lk_products;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0;reg signed [8:0] a,b,c;
 integer i,j,aa,bb,cc;
 flow_lk_patch dut(.clk(clk),.rst_n(rst_n),.start(1'b0),.coarse(1'b0),
  .center_x(16'd8),.center_y(16'd8),.initial_u(16'd0),.initial_v(16'd0),.read_data(8'd0));
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  force dut.gx=a;force dut.gy=b;force dut.gt=c;
  for(i=-256;i<256;i=i+1)for(j=-256;j<256;j=j+1)begin
   @(negedge clk);a=i;b=j;c=a^b;aa=a;bb=b;cc=c;
   force dut.data_phase=32'd1<<dut.PRODUCTS;
   @(negedge clk);force dut.data_phase=32'd1<<dut.PRODUCT_SUM;
   @(posedge clk);#1;
   if($signed(dut.xx)!==(aa*aa) || $signed(dut.yy)!==(bb*bb) ||
      $signed(dut.xy)!==(aa*bb) || $signed(dut.xt)!==(aa*cc) || $signed(dut.yt)!==(bb*cc))
    $fatal(1,"Partial product mismatch a=%0d b=%0d c=%0d",aa,bb,cc);
  end
  $display("PASS all 262144 signed input pairs for all five pipelined LK products");$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
