`timescale 1ns/1ps
module tb_vfh;
 reg clk=0;always #10 clk=~clk;reg rst=0,start=0;
 reg signed[15:0] target,previous;reg[15:0] rad,lo=1200,hi=1400,stop=450,slow=1000,turn;
 wire[6:0] index;reg[15:0] distancev;reg known;reg[15:0] ranges[0:71];reg[71:0] observed;
 wire busy,done,found,fk,rc,clear;wire[31:0] seq;wire[15:0] fm;wire signed[15:0] safe;wire[71:0] blocked;wire[23:0] cost;
 nav_vfh dut(clk,rst,start,16'd0,target,previous,rad,lo,hi,stop,slow,turn,8'd5,8'd1,8'd3,8'd8,index,distancev,known,32'd0,clear,seq,busy,done,found,fk,rc,fm,safe,blocked,cost);
 always @(posedge clk)begin distancev<=ranges[index];known<=observed[index];end
 integer f,o,r,j,n,id;
 initial begin f=$fopen("build/task4/vfh_input.txt","r");o=$fopen("build/task4/vfh_output.txt","w");repeat(3)@(negedge clk);rst=1;
 while(!$feof(f))begin
 r=$fscanf(f,"%d %d %d %d %h\n",target,previous,rad,turn,observed);
 if(r==5)begin
 for(j=0;j<72;j=j+1)r=$fscanf(f,"%d",ranges[j]);
 @(negedge clk);start=1;@(negedge clk);start=0;n=0;
 while(!done && n<20000)begin @(negedge clk);n=n+1;end
 if(!done)$fatal(1,"VFH timeout");$fwrite(o,"%d %d %h %d %d %d %d %d\n",found,safe,blocked,cost,fk,rc,fm,n);
 end end $fclose(o);$display("PASS VFH fixtures");$finish;end
 initial begin #1000000000;$fatal(1,"timeout");end
endmodule
