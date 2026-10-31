`timescale 1ns/1ps
module tb_pose_fusion;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,armed=0,imu_valid=0,imu_bad=0,pv=0,cv=0;
 reg [15:0] angle=0,distancev=4000;reg [7:0] intensity=100,opcode=0,seq=0;
 reg [15:0] a0=0,a1=0,a2=0,a3=0;
 reg signed [31:0] lp=0,rp=0;
 wire running,finished,permit,update;wire[15:0] faults,front;wire signed[15:0] lt,rt,v,w;
 wire signed[31:0] x,y,theta;wire[4:0] wi;wire[71:0] blocked;wire[31:0] rejected,plans;
 nav_core #(.CLK_HZ(1000000)) dut(clk,rst,lp,rp,armed,imu_valid,imu_bad,16'sd0,16'sd0,gyro_raw,
  pv,angle,distancev,intensity,cv,opcode,seq,a0,a1,a2,a3,running,finished,permit,faults,update,lt,rt,x,y,theta,v,w,wi,front,blocked,rejected,plans);
 task cmd(input[7:0] op,input[15:0] p0,p1,p2,p3);
 begin @(negedge clk);cv=1;opcode=op;seq=seq+1;a0=p0;a1=p1;a2=p2;a3=p3;@(negedge clk);cv=0;end endtask
 task cfg(input[4:0] addr,input[31:0] value);begin cmd(8'h40,{11'b0,addr},value[15:0],value[31:16],0);end endtask
 integer i,j,cycles=0,updates=0;reg [71:0] obstacle_mask=0;reg feed_imu=1,feed_lidar=1,drive=0,heart=0,hazard=0;
 // Production inputs are independent streams; deliberately asynchronous phases.
 initial forever begin repeat(911)@(negedge clk);imu_valid=feed_imu;@(negedge clk);imu_valid=0;end
 initial begin wait(rst);forever begin
  for(j=0;j<72;j=j+1)begin
   repeat(100)@(negedge clk);pv=feed_lidar;angle=j*500+100;distancev=obstacle_mask[j] ? (case_index==11 ? 550:700) : 4000;
   @(negedge clk);pv=0;repeat(100)@(negedge clk);pv=feed_lidar;angle=j*500+200;
   @(negedge clk);pv=0;
  end
 end end
 always @(negedge clk)begin
  cycles=cycles+1;
  if(update)updates=updates+1;
  if(drive && dut.tick)begin lp=lp+lt;rp=rp+rt;end
 end
 reg signed[15:0] gyro_raw=0;integer fd,case_index;reg signed[15:0] goals_x[0:11],goals_y[0:11];
 initial begin
 fd=$fopen("build/task4/pose_output.txt","w");
 for(case_index=0;case_index<4;case_index=case_index+1)begin
  rst=0;armed=0;drive=0;obstacle_mask=0;lp=0;rp=0;gyro_raw=0;
  if(case_index==1)gyro_raw=1341;if(case_index==2)gyro_raw=-1341;if(case_index==3)gyro_raw=670;
  repeat(4)@(negedge clk);rst=1;repeat(2000)@(negedge clk);
  cfg(1,6554);cfg(2,case_index==3 ? 13107:6554);cfg(3,280);cfg(4,180);cfg(5,100);
  cfg(18,6554);cfg(19,case_index==3 ? 3277:6554);cfg(20,case_index==2?6:2);
  cfg(27,1);cfg(0,15);cmd(8'h41,0,8000,0,120);
  repeat(20000)@(negedge clk);armed=1;cmd(8'h42,0,0,0,0);
  if(case_index==0)begin lp=100;rp=100;end
  else if(case_index==1 || case_index==2)begin lp=-10;rp=10;end
  else begin lp=10;rp=10;end
  wait(plans>0 || faults!=0);@(negedge clk);
  if(faults)$fatal(1,"fusion fault %h",faults);
  $fwrite(fd,"%d %d %d\n",x,y,theta);cmd(0,0,0,0,0);
 end
 $fclose(fd);$display("PASS encoder/IMU fusion + unequal wheel scale + gyro axis sign fixtures");$finish;
 end
 initial begin #200000000;$fatal(1,"navigation timeout state=%d x=%d faults=%h plans=%d permit=%d",dut.state,x,faults,plans,permit);end
endmodule
