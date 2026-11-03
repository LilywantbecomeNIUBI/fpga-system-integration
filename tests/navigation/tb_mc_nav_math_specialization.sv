`timescale 1ns/1ps
// Cycle-exact comparison to the immutable pre-specialization implementation.
module tb_mc_nav_math_specialization;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,start=0;reg [2:0] op=0;
 reg signed [63:0] a=0;reg signed [31:0] b=0;
 wire [3:0] busy,done,error;wire signed [63:0] result[0:3];
 nav_math_full_reference oracle(clk,rst_n,start,op,a,b,busy[0],done[0],error[0],result[0]);
 nav_math full(clk,rst_n,start,op,a,b,busy[1],done[1],error[1],result[1]);
 nav_math #(.OP_MASK(5'b00111)) path_math(clk,rst_n,start,op,a,b,busy[2],done[2],error[2],result[2]);
 nav_math #(.OP_MASK(5'b11010)) scalar_math(clk,rst_n,start,op,a,b,busy[3],done[3],error[3],result[3]);
 integer file,parsed,n,cases=0,cycles=0,j,seed=32'h414d4154;
 reg [3:0] enabled;
 task automatic check;
  begin
   for(j=1;j<4;j=j+1)if(enabled[j])begin
    if({busy[j],done[j],error[j]}!=={busy[0],done[0],error[0]})
     $fatal(1,"math handshake mismatch case=%0d op=%0d unit=%0d",cases,op,j);
    if(done[0] && result[j]!==result[0])
     $fatal(1,"math value mismatch case=%0d op=%0d unit=%0d",cases,op,j);
   end
   cycles=cycles+1;
  end
 endtask
 task automatic execute(input [2:0] kind,input signed [63:0] lhs,input signed [31:0] rhs);
  begin
   @(negedge clk);op=kind;a=lhs;b=rhs;start=1;
   enabled=4'b0011;
   if(kind<=2)enabled[2]=1;
   if(kind==1 || kind==3 || kind==4)enabled[3]=1;
   @(negedge clk);start=0;check();
   if(!enabled[2] && {busy[2],done[2],error[2],result[2]}!=={3'b011,64'd0})$fatal(1,"path disabled operation must fail immediately");
   if(!enabled[3] && {busy[3],done[3],error[3],result[3]}!=={3'b011,64'd0})$fatal(1,"scalar disabled operation must fail immediately");
   n=0;
   while(!done[0] && n<100)begin @(negedge clk);check();n=n+1;end
   if(!done[0])$fatal(1,"math timeout");
   @(negedge clk);check();
   cases=cases+1;
  end
 endtask
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  file=$fopen("build/task4/math_input.txt","r");
  if(!file)$fatal(1,"Run Task4 vector generation first");
  while(!$feof(file))begin
   parsed=$fscanf(file,"%d %h %h\n",op,a,b);
   if(parsed==3)execute(op,a,b);
  end
  $fclose(file);
  execute(1,64'sh8000000000000000,1);
  execute(1,64'sh7fffffffffffffff,32'sh7fffffff);
  execute(1,-1,0);execute(1,1,-1);
  execute(0,64'sh80000000,32'sh80000000);
  execute(2,64'hffffffffffffffff,0);
  execute(5,0,0);execute(6,0,0);execute(7,0,0);
  for(integer k=0;k<128;k=k+1)begin
   execute(0,$random(seed),$random(seed));
   execute(1,{$random(seed),$random(seed)},($random(seed)&32'h7fffffff)|1);
   execute(2,{$random(seed),$random(seed)},0);
   execute(3,$random(seed)%10000,$random(seed)%10000);
   execute(4,$random(seed)%18001,0);
  end
  // Reset while busy must abandon both specialized and full calculations.
  @(negedge clk);op=1;a=-123456;b=7;start=1;
  @(negedge clk);start=0;
  repeat(5)@(negedge clk);
  rst_n=0;#1;
  if(busy!==0 || done!==0 || error!==0)$fatal(1,"math reset");
  @(negedge clk);rst_n=1;execute(1,-99,7);
  @(negedge clk);op=1;a=1000;b=3;start=1;
  @(negedge clk);start=0;
  repeat(3)@(negedge clk);
  force full.mode=3'd7;force path_math.mode=3'd7;force scalar_math.mode=3'd7;
  @(negedge clk);
  if(busy[3:1]!==0 || done[3:1]!==3'b111 || error[3:1]!==3'b111)$fatal(1,"invalid busy mode deadlocked");
  release full.mode;release path_math.mode;release scalar_math.mode;
  rst_n=0;@(negedge clk);rst_n=1;execute(2,123456789,0);
  $display("PASS math specialization: %0d cases, %0d checked cycles, legacy/default/path/scalar, disabled ops/reset/invalid-state exit",cases,cycles);
  $finish;
 end
 initial begin #100000000;$fatal(1,"math specialization watchdog");end
endmodule
