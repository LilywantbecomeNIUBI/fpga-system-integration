`timescale 1ns/1ps
module tb_mc_nav_cell;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,start=0;reg signed [31:0] mm=0;wire busy,done,outside;wire [6:0] index;wire signed [12:0] full_index;
 mc_nav_cell_index convert(.clk(clk),.rst_n(rst),.start(start),.mm(mm),.busy(busy),.done(done),.outside(outside),.index(index),.extended_index(full_index));
 integer i,n,expected,clipped;
 initial begin
  repeat(3)@(negedge clk);rst=1;
  for(i=-66001;i<=66001;i=i+137)begin
   @(negedge clk);mm=i;start=1;@(negedge clk);start=0;n=0;
   while(!done)begin @(negedge clk);n=n+1;if(n>35)$fatal(1,"divider not bounded");end
   expected=$floor(($itor(i)+3200.0)/50.0);clipped=expected<0?0:expected>127?127:expected;
   if(full_index!=expected || index!=clipped || outside!==(i< -3200 || i>=3200))$fatal(1,"signed floor %d got %d/%d expected %d/%d",i,full_index,index,expected,clipped);
  end
  $display("PASS 964 signed floor/clipped/extended conversion vectors and 33-cycle bound");$finish;
 end
endmodule
