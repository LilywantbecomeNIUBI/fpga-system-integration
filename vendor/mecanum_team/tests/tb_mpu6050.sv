`timescale 1ns/1ps
module mpu6050_model(
 inout scl,sda,input present,freeze,corrupt,input [7:0] identity,
 input signed [15:0] gx,gy,gz
);
 reg drive_low=0;assign sda=drive_low ? 1'b0 : 1'bz;
 localparam RX_ADDR=0,RX_REG=1,RX_WRITE=2,TX_READ=3,MASTER_ACK=4,IGNORE=5;
 integer state=IGNORE,next_state=IGNORE,bits=0,pointer=0,writes=0,reads=0,burst_reads=0;
 reg [7:0] regs[0:127];reg [7:0] frozen[0:5];
 reg [7:0] shift=0,tx_byte=0,value;
 reg ack_phase=0,ack_clocked=0,ack_enable=0,master_ack_clocked=0,master_nack=0,pointer_valid=0;
 integer j;
 task defaults;integer k;begin for(k=0;k<128;k=k+1)regs[k]=0;regs['h6b]='h40;end endtask
 initial defaults();
 always begin #100000;if(!freeze && regs['h6b]==1)regs['h3a]=1;end
 function [7:0] get_byte(input integer p);
 begin
  if(p=='h75)get_byte=identity;
  else if(p>='h43 && p<='h48)get_byte=frozen[p-'h43];
  else if(p=='h1b && corrupt)get_byte=regs[p]^8'h08;
  else get_byte=regs[p];
 end endfunction
 task load_byte;begin tx_byte=get_byte(pointer);if(pointer=='h3a)regs['h3a]=0;end endtask
 always @(negedge sda)if(scl===1'b1)begin state=RX_ADDR;bits=0;shift=0;ack_phase=0;ack_clocked=0;drive_low=0;end
 always @(posedge sda)if(scl===1'b1)begin state=IGNORE;drive_low=0;pointer_valid=0;end
 always @(posedge scl)begin
  if(ack_phase)ack_clocked=1;
  else if(state==RX_ADDR || state==RX_REG || state==RX_WRITE)begin
   value={shift[6:0],sda};shift=value;
   if(bits==7)begin
    bits=0;ack_enable=present;ack_phase=1;ack_clocked=0;
    case(state)
     RX_ADDR:begin
      ack_enable=present && value[7:1]==7'h68;
      next_state=ack_enable ? (value[0] ? TX_READ : RX_REG) : IGNORE;
      if(ack_enable && value[0])begin
       if(!pointer_valid)$fatal(1,"MPU read without repeated START");
       reads=reads+1;
       frozen[0]=gx[15:8];frozen[1]=gx[7:0];frozen[2]=gy[15:8];frozen[3]=gy[7:0];frozen[4]=gz[15:8];frozen[5]=gz[7:0];
       if(pointer=='h43)burst_reads=burst_reads+1;
      end
     end
     RX_REG:begin pointer=value;pointer_valid=1;next_state=RX_WRITE;end
     RX_WRITE:begin
      next_state=RX_WRITE;
      if(present)begin writes=writes+1;if(pointer=='h6b && value=='h80)defaults();else regs[pointer]=value;pointer=(pointer+1)&127;end
     end
    endcase
   end else bits=bits+1;
  end else if(state==TX_READ)begin
   if(bits==7)begin state=MASTER_ACK;bits=0;master_ack_clocked=0;end else bits=bits+1;
  end else if(state==MASTER_ACK)begin master_nack=sda;master_ack_clocked=1;end
 end
 always @(negedge scl)begin
  if(ack_phase)begin
   if(!ack_clocked)drive_low=ack_enable;
   else begin ack_phase=0;drive_low=0;state=next_state;bits=0;if(state==TX_READ)begin load_byte();drive_low=!tx_byte[7];end end
  end else if(state==TX_READ)drive_low=!tx_byte[7-bits];
  else if(state==MASTER_ACK)begin
   drive_low=0;
   if(master_ack_clocked)begin
    if(master_nack)state=IGNORE;
    else begin pointer=(pointer+1)&127;bits=0;state=TX_READ;load_byte();drive_low=!tx_byte[7];end
   end
  end else drive_low=0;
 end
endmodule

module tb_mpu6050;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,restart=0,present=1,freeze=0,corrupt=0;reg [7:0] identity=8'h68;
 tri1 scl,sda;wire ready,fault,valid,stale;wire signed [15:0] gx,gy,gz;
 wire [31:0] sample_us,now_us;wire [7:0] who;wire [3:0] error_code;
 mpu6050_reader #(.CLK_HZ(1000000),.BOOT_US(100),.RESET_US(100),.SETTLE_US(100)) dut(
 .clk(clk),.reset_n(reset_n),.restart(restart),.scl(scl),.sda(sda),.ready(ready),.fault(fault),.sample_valid(valid),
 .gx(gx),.gy(gy),.gz(gz),.sample_us(sample_us),.now_us(now_us),.who_am_i(who),.error_code(error_code),.stale(stale));
 mpu6050_model model(.scl(scl),.sda(sda),.present(present),.freeze(freeze),.corrupt(corrupt),.identity(identity),.gx(-16'sd131),.gy(16'sd262),.gz(16'sd1310));
 task reset;begin @(negedge clk);reset_n=0;repeat(4)@(negedge clk);reset_n=1;end endtask
 reg [31:0] saved_sample;
 initial begin
  #1;reset();wait(valid);@(negedge clk);
  if(!ready || fault || stale || who!=8'h68 || gx!=-131 || gy!=262 || gz!=1310)$fatal(1,"MPU sample/config");
  if(model.regs['h19]!=9 || model.regs['h1a]!=3 || model.regs['h1b]!=0 || model.burst_reads<1)$fatal(1,"MPU configuration");
  wait(dut.state==11);@(negedge clk);saved_sample=sample_us;present=0;
  wait(dut.bus_error_count!=0);@(negedge clk);
  if(fault || sample_us!=saved_sample || gz!=1310 || valid)$fatal(1,"short read failure did not retain the last complete sample");
  present=1;wait(valid);@(negedge clk);
  if(fault || sample_us==saved_sample || gz!=1310)$fatal(1,"automatic retry did not publish fresh sample");
  wait(dut.state==14);@(negedge clk);saved_sample=sample_us;present=0;
  wait(error_code==6);@(negedge clk);
  if(sample_us!=saved_sample || gz!=1310 || valid)$fatal(1,"burst NACK overwrote cached data");
  repeat(30000)@(negedge clk);
  if(sample_us!=saved_sample || !ready)$fatal(1,"long communication failure discarded cache");
  present=1;wait(valid);@(negedge clk);
  if(fault || error_code!=0 || sample_us==saved_sample)$fatal(1,"burst failure did not auto-recover");
  freeze=1;repeat(40000)@(negedge clk);if(!stale)$fatal(1,"MPU frozen samples");
  present=0;wait(fault);repeat(3000)@(negedge clk);
  if(!ready || error_code==0)$fatal(1,"runtime communication state must retain initialized state");
  present=1;freeze=0;identity=8'h70;
  @(negedge clk);restart=1;repeat(2)@(negedge clk);restart=0;
  wait(valid);@(negedge clk);
  if(!ready || fault || stale || who!=8'h70 || gx!=-131 || gy!=262 || gz!=1310)$fatal(1,"MPU6500 sample");
  if(model.regs['h19]!=9 || model.regs['h1a]!=3 || model.regs['h1b]!=0)$fatal(1,"MPU6500 divider/DLPF/FCHOICE/full-scale config");
  if(dut.last_bus_error!=1)$fatal(1,"NACK diagnostic not retained after restart");
  identity=8'h71;reset();wait(fault);if(error_code!=1)$fatal(1,"unsupported MPU identity");
  identity=8'h68;corrupt=1;reset();wait(fault);if(error_code!=4)$fatal(1,"MPU readback");
  $display("PASS MPU6050/MPU6500: last-sample hold on NACK, automatic fresh-sample recovery, stale detection, manual communication restart and strict WHO/config validation");$finish;
 end
 initial begin #3000000;$fatal(1,"MPU timeout");end
endmodule
