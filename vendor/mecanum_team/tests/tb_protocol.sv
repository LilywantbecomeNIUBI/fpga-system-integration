`timescale 1ns/1ps
module tb_protocol;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,byte_valid=0,error=0;reg [7:0] data=0;
 wire valid;wire [175:0] body;wire [31:0] bad;
 reg [191:0] frame;integer hits=0,i;reg [31:0] old_bad;
 mecanum_protocol #(.CLK_HZ(100000)) dut(.clk(clk),.reset_n(reset_n),.byte_valid(byte_valid),.framing_error(error),.byte_data(data),.valid(valid),.body(body),.bad_frames(bad));
 always @(posedge clk)if(valid)hits=hits+1;
 function automatic [15:0] crc(input [175:0] bytes);
 reg [15:0] c;integer i,j;
 begin c=16'hffff;for(i=0;i<22;i=i+1)begin c=c^{bytes[i*8+:8],8'b0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;end crc=c;end
 endfunction
 task send(input [7:0] v);begin @(negedge clk);byte_valid=1;data=v;@(negedge clk);byte_valid=0;repeat(2)@(negedge clk);end endtask
 task send_frame;begin for(i=0;i<24;i=i+1)send(frame[i*8+:8]);repeat(4)@(negedge clk);end endtask
 initial begin
  #1;repeat(3)@(negedge clk);reset_n=1;
  frame=0;frame[15:0]=16'h5aa5;frame[23:16]=2;frame[31:24]=2;frame[39:32]=19;frame[79:64]=-123;frame[127:112]=256;
  frame[191:176]=crc(frame[175:0]);send_frame();
  if(hits!=1 || $signed(body[79:64])!=-123)$fatal(1,"valid frame");
  frame[80]=!frame[80];send_frame();if(hits!=1 || bad!=1)$fatal(1,"CRC");
  frame[80]=!frame[80];frame[144]=1;frame[191:176]=crc(frame[175:0]);send_frame();if(hits!=1 || bad!=2)$fatal(1,"reserved");
  send(8'ha5);send(8'h5a);repeat(2100)@(negedge clk);if(bad!=3)$fatal(1,"partial timeout");
  frame[144]=0;frame[191:176]=crc(frame[175:0]);send(8'ha5);send_frame();if(hits!=2)$fatal(1,"resync");
  send(8'ha5);@(negedge clk);error=1;@(negedge clk);error=0;repeat(3)@(negedge clk);if(bad!=4)$fatal(1,"framing");
  $display("PASS protocol: CRC, reserved, framing, partial timeout and recovery");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
