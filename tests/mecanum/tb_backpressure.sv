`timescale 1ns/1ps
module tb_backpressure;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,ready=0,aux_valid=0;
 reg [383:0] auxiliary=0;
 wire [383:0] frame;wire valid,aux_ready,tx;
 wire [3:0] in1,in2;
 integer motors=0,poses=0,auxiliaries=0;
 reg stalled=0;reg [383:0] held;
 mc_mecanum_top #(.CLK_HZ(1000000),.IMU_ENABLED(0),.TELEMETRY_ETHERNET(1)) dut(
  .clk50m(clk),.reset_n(rst),.uart_rx(1'b1),.enc_a(4'b0),.enc_b(4'b0),
  .in1(in1),.in2(in2),.uart_tx(tx),.auxiliary_valid(aux_valid),.auxiliary_frame(auxiliary),
  .auxiliary_ready(aux_ready),.telemetry_frame(frame),.telemetry_valid(valid),.telemetry_ready(ready));
 function automatic [15:0] crc(input [367:0] body);
 reg [15:0] c;integer i,j;
 begin c=16'hffff;
  for(i=0;i<46;i=i+1)begin
   c=c^{body[i*8+:8],8'b0};
   for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;
  end
  crc=c;
 end endfunction
 always @(posedge clk)if(rst)begin
  if(stalled && (!valid || frame!==held))$fatal(1,"active frame changed under backpressure");
  stalled<=valid && !ready;held<=frame;
  if(in1!==0 || in2!==0 || tx!==1)$fatal(1,"unarmed pads or UART active");
  if(valid && ready)begin
   if(frame[383:368]!==crc(frame[367:0]))$fatal(1,"torn or invalid CRC");
   case(frame[15:0])
    16'h344d:motors=motors+1;
    16'h3450:poses=poses+1;
    16'h3cc3:begin
     if(frame!==auxiliary)$fatal(1,"radar frame overwritten");
     auxiliaries=auxiliaries+1;
    end
    default:$fatal(1,"unexpected identity");
   endcase
  end
 end
 initial begin
  repeat(8)@(negedge clk);rst=1;
  repeat(20)@(negedge clk);
  auxiliary[23:0]=24'h053cc3;auxiliary[367:24]=344'h123456789abc;
  auxiliary[383:368]=crc(auxiliary[367:0]);
  wait(aux_ready);@(negedge clk);aux_valid=1;
  @(negedge clk);aux_valid=0;
  // More than three snapshot periods, while the selected radar frame is held.
  repeat(180000)@(negedge clk);ready=1;
  repeat(120000)@(negedge clk);
  if(auxiliaries!=1 || motors<2 || poses<2)$fatal(1,"lost telemetry classes %d %d %d",auxiliaries,motors,poses);
  ready=0;repeat(60000)@(negedge clk);
  rst=0;stalled=0;repeat(8)@(negedge clk);
  if(valid || in1 || in2)$fatal(1,"reset did not discard pending frame");
  rst=1;ready=1;repeat(1000)@(negedge clk);
  if(valid)$fatal(1,"old pre-reset frame replayed");
  $display("PASS integrated telemetry: long backpressure, immutable CRC frames, radar/motor/pose progress, UART idle and reset isolation");$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
