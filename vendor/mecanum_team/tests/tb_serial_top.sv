`timescale 1ns/1ps
module tb_serial_top;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,rx=1;reg [3:0] a=0,b=0;
 wire tx;wire [3:0] in1,in2;wire [7:0] led;
 wire old1,old2;wire [5:0] oldp1,oldp2;wire [1:0] oldcal;
 mecanum_top #(.CLK_HZ(1000000),.BAUD(100000),.CONTROL_HZ(1000),.IMU_ENABLED(0),.CONTROL_SHIFT(0),.BASE_ENCODER_REVERSE(0),.BASE_MOTOR_REVERSE(0)) dut(
 .clk50m(clk),.reset_n(reset_n),.uart_rx(rx),.enc_a(a),.enc_b(b),.uart_tx(tx),.in1(in1),.in2(in2),.led(led),
 .legacy_m1_disable(old1),.legacy_m2_disable(old2),.legacy_m1_pwm_disabled(oldp1),.legacy_m2_pwm_disabled(oldp2),.legacy_cal_disabled(oldcal));
 wire [7:0] received;wire received_valid,received_error;
 uart_rx_byte #(.CLK_HZ(1000000),.BAUD(100000)) monitor(.clk(clk),.reset_n(reset_n),.rx(tx),.data(received),.valid(received_valid),.framing_error(received_error));
 reg [383:0] frame=0,motor_frame=0;integer count=0,frames=0;integer k;reg plant=0;
 function automatic [15:0] crc(input [367:0] bytes,input integer length);
 reg [15:0] c;integer i,j;
 begin c=16'hffff;for(i=0;i<length;i=i+1)begin c=c^{bytes[i*8+:8],8'b0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;end crc=c;end
 endfunction
 always @(negedge clk)begin
  if(received_error)$fatal(1,"telemetry UART framing");
  if(received_valid)begin
   frame[count*8+:8]=received;
   if(count==47)begin
    count=0;
    if((frame[15:0]!=16'h344d && frame[15:0]!=16'h3450) || frame[23:16]!=2 || frame[383:368]!=crc(frame[367:0],46))$fatal(1,"telemetry format/CRC");
    if(frame[15:0]==16'h344d)begin frames=frames+1;motor_frame=frame;end
   end else count=count+1;
  end
 end
 // Four independent physical inputs follow the same forward quadrature sequence.
 initial forever begin
  wait(plant);
  repeat(250)@(negedge clk);b=15;
  repeat(250)@(negedge clk);a=15;
  repeat(250)@(negedge clk);b=0;
  repeat(250)@(negedge clk);a=0;
 end
 task byte_send(input [7:0] value);
 integer i;
 begin
  @(negedge clk);rx=0;repeat(10)@(negedge clk);
  for(i=0;i<8;i=i+1)begin rx=value[i];repeat(10)@(negedge clk);end
  rx=1;repeat(10)@(negedge clk);
 end endtask
 task command(input [7:0] op,seq,input signed [15:0] speed,input corrupt);
 reg [367:0] packet;integer i;
 begin
  packet=0;packet[15:0]=16'h5aa5;packet[23:16]=2;packet[31:24]=op;packet[39:32]=seq;packet[79:64]=speed*64;packet[127:112]=256;
  if(op==6)begin packet[159:144]=40;packet[175:160]=4;end
  packet[191:176]=crc(packet,22);if(corrupt)packet[190]=!packet[190];
  for(i=0;i<24;i=i+1)byte_send(packet[i*8+:8]);repeat(20)@(negedge clk);
 end endtask
 initial begin
  force dut.yaw_ok=1'b1;force dut.yaw=32'd0;
  #1;repeat(5)@(negedge clk);reset_n=1;repeat(100)@(negedge clk);
  command(3,0,0,0);command(1,1,0,0);if(!dut.armed)$fatal(1,"serial arm");
  command(6,2,20,1);if(dut.target!=0 || dut.bad!=1)$fatal(1,"bad CRC drove wheels");
  command(6,2,20,0);plant=1;
  repeat(60000)@(negedge clk);
  if(frames<1 || !motor_frame[24] || motor_frame[47:32]!=0)$fatal(1,"status snapshot");
  for(k=0;k<4;k=k+1)begin
   if($signed(motor_frame[64+k*32+:32])<=0 || $signed(motor_frame[192+k*16+:16])<=0 || $signed(motor_frame[256+k*16+:16])<=0)$fatal(1,"wheel feedback %0d",k);
  end
  if({old1,old2,oldp1,oldp2,oldcal}!=0)$fatal(1,"old outputs");
  command(0,2,0,0);#1;if(dut.armed || in1 || in2)$fatal(1,"serial stop");
  reset_n=0;#1;if(in1 || in2)$fatal(1,"reset stop");
  $display("PASS serial top: physical UART CRC/ARM/POSE/STOP, four encoders/PI, telemetry CRC, old outputs and reset");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
