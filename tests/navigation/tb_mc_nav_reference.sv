`timescale 1ns/1ps
module tb_mc_nav_reference;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,start=0;wire busy,done,safe,timeout,unknown_seen,rv;
 reg signed [31:0] x=0,y=0,a=0,sa=0;reg signed [15:0] sx=0,sy=0;
 reg [5:0] steps=1;wire [13:0] ra;reg rd=0;reg [1:0] code=1;
 wire [31:0] cost,cycles;integer fd,n,cx,cy,hit,tests=0,false_positive=0,max_cycles=0;
 mc_nav_sweep dut(.clk(clk),.rst_n(rst),.start(start),.abort(1'b0),.map_epoch(32'd1),
 .x_mm(x),.y_mm(y),.theta_q16(a),.step_x(sx),.step_y(sy),.step_theta(sa),.steps(steps),
 .front_mm(16'd100),.back_mm(16'd100),.half_width_mm(16'd50),.radius_mm(16'd120),.pad_mm(16'd68),
 .busy(busy),.done(done),.safe(safe),.timed_out(timeout),.unknown_seen(unknown_seen),.unknown_cost(cost),.cycles(cycles),
 .read_valid(rv),.read_address(ra),.read_ready(1'b1),.read_done(rd),.read_code(code));
 always @(posedge clk)begin rd<=rv;if(rv)code<=ra=={cy[6:0],cx[6:0]}?2:1;end
 initial begin
  repeat(3)@(negedge clk);rst=1;
  fd=$fopen("build/mecanum_navigation/sweep_vectors.txt","r");if(!fd)$fatal(1,"missing oracle fixture");
  while(!$feof(fd))begin
   n=$fscanf(fd,"%d %d %d %d %d %d %d %d %d %d\n",x,y,a,sx,sy,sa,steps,cx,cy,hit);
   if(n==10)begin
    start=1;@(negedge clk);start=0;while(!done)@(negedge clk);
    if(timeout || (hit && safe))$fatal(1,"oracle missed collision case=%d at %d,%d",tests,cx,cy);
    if(!hit && !safe)false_positive=false_positive+1;
    if(cycles>max_cycles)max_cycles=cycles;tests=tests+1;@(negedge clk);
   end
  end
  if(tests!=100 || false_positive>30)$fatal(1,"fixture count/conservatism regression %d %d",tests,false_positive);
  $display("PASS independent SAT/dense sweep oracle cases=%d conservative_rejections=%d max_cycles=%d",tests,false_positive,max_cycles);$finish;
 end
 initial begin #1000000000;$fatal(1,"oracle test timeout");end
endmodule
