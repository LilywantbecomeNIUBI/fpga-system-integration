`timescale 1ns/1ps
module tb_navigation;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,armed=0,imu_valid=0,imu_bad=0,pv=0,cv=0;
 reg [15:0] angle=0,distancev=4000;reg [7:0] intensity=100,opcode=0,seq=0;
 reg [15:0] a0=0,a1=0,a2=0,a3=0;
 reg signed [31:0] lp=0,rp=0;
 wire running,finished,permit,update;wire[15:0] faults,front;wire signed[15:0] lt,rt,v,w;
 wire signed[31:0] x,y,theta;wire[4:0] wi;wire[71:0] blocked;wire[31:0] rejected,plans;
 nav_core #(.CLK_HZ(1000000)) dut(clk,rst,lp,rp,armed,imu_valid,imu_bad,16'sd0,16'sd0,16'sd0,
  pv,angle,distancev,intensity,cv,opcode,seq,a0,a1,a2,a3,running,finished,permit,faults,update,lt,rt,x,y,theta,v,w,wi,front,blocked,rejected,plans);
 task cmd(input[7:0] op,input[15:0] p0,p1,p2,p3);
 begin @(negedge clk);cv=1;opcode=op;seq=seq+1;a0=p0;a1=p1;a2=p2;a3=p3;@(negedge clk);cv=0;end endtask
 task cfg(input[4:0] addr,input[31:0] value);begin cmd(8'h40,{11'b0,addr},value[15:0],value[31:16],0);end endtask
 integer i,j,cycles=0,updates=0;reg feed_imu=1,feed_lidar=1,drive=0,heart=0,hazard=0;
 // Production inputs are independent streams; deliberately asynchronous phases.
 initial forever begin repeat(911)@(negedge clk);imu_valid=feed_imu;@(negedge clk);imu_valid=0;end
 initial begin wait(rst);forever begin
  for(j=0;j<72;j=j+1)begin
   repeat(100)@(negedge clk);pv=feed_lidar;angle=j*500+100;distancev=hazard && j==0 ? 180:4000;
   @(negedge clk);pv=0;repeat(100)@(negedge clk);pv=feed_lidar;angle=j*500+200;
   @(negedge clk);pv=0;
  end
 end end
 always @(negedge clk)begin
  cycles=cycles+1;
  if(update)updates=updates+1;
  if(drive && dut.tick)begin lp=lp+lt;rp=rp+rt;end
 end
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(2000)@(negedge clk);armed=1;
  cmd(8'h42,0,0,0,0);if(running || faults==0)$fatal(1,"unconfigured navigation started");armed=0;
  cfg(1,6554);cfg(2,6554);cfg(3,280);cfg(4,180);cfg(5,100);cfg(18,6554);cfg(19,6554);cfg(20,2);
  cfg(27,1);cfg(0,15);cmd(8'h41,0,500,0,120);
  repeat(20000)@(negedge clk);armed=1;cmd(8'h42,0,0,0,0);if(!running)$fatal(1,"calibrated start refused %h",faults);
  drive=1;
  // Heartbeat is separate from navigation computation; it never sets wheel targets.
  fork
   begin while(running)begin repeat(50000)@(negedge clk);if(running)cmd(8'h43,0,0,0,0);end end
   begin wait(permit);repeat(100)@(negedge clk);hazard=1;wait(dut.histogram.hazard);repeat(2)@(negedge clk);
    if(lt!=0 || rt!=0 || permit)$fatal(1,"raw hazard did not stop");hazard=0;end
   begin wait(finished);end
  join
  if(faults || !finished || running || x<380*65536 || x>510*65536 || y>3*65536 || y< -3*65536)
   $fatal(1,"straight mission failed fault=%h done=%d x=%d y=%d plans=%d",faults,finished,x,y,plans);
  $display("PASS straight RPP/odom/waypoint completion x_q16=%d plans=%d",x,plans);
  drive=0;cmd(8'h42,0,0,0,0);repeat(50000)@(negedge clk);
  // Duplicate heartbeat must not extend the lease.
  @(negedge clk);cv=1;opcode=8'h43;@(negedge clk);cv=0;
  repeat(160000)@(negedge clk);if(running || !faults[5] || lt || rt)$fatal(1,"lease watchdog/duplicate failed %h",faults);
  cmd(8'h42,0,0,0,0);if(!running)$fatal(1,"restart failed");
  feed_imu=0;repeat(23000)@(negedge clk);if(running || !faults[1] || lt || rt)$fatal(1,"IMU timeout failed");
  feed_imu=1;repeat(2000)@(negedge clk);cmd(8'h42,0,0,0,0);cmd(0,0,0,0,0);
  if(running || lt || rt)$fatal(1,"STOP did not dominate");
  $display("PASS raw hazard, calibration gate, duplicate heartbeat, lease, IMU stale, STOP; cycles=%d",cycles);$finish;
 end
 initial begin #200000000;$fatal(1,"navigation timeout state=%d x=%d faults=%h plans=%d permit=%d",dut.state,x,faults,plans,permit);end
endmodule
