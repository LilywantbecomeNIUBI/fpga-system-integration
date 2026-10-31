`timescale 1ns/1ps
module tb_math;
 reg clk=0;always #10 clk=~clk;reg rst=0,start=0;reg[2:0] op;reg signed[63:0] a;reg signed[31:0] b;
 wire busy,done,error;wire signed[63:0] result;
 nav_math dut(clk,rst,start,op,a,b,busy,done,error,result);
 integer f,o,n,r;reg[1023:0] path;
 initial begin f=$fopen("build/task4/math_input.txt","r");o=$fopen("build/task4/math_output.txt","w");repeat(3)@(negedge clk);rst=1;
 while(!$feof(f))begin r=$fscanf(f,"%d %h %h\n",op,a,b);if(r==3)begin
 @(negedge clk);start=1;@(negedge clk);start=0;n=0;
 while(!done && n<100)begin @(negedge clk);n=n+1;end
 if(!done)$fatal(1,"math timeout");$fwrite(o,"%h %d\n",result,error);
 end end $fclose(o);$display("PASS math vectors");$finish;end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
