`timescale 1ns/1ps
module tb_lidar_import;
 reg clk=0;always #10 clk=~clk;reg rst=0,rx=1;
 wire bv,fe,pv;wire[7:0] byte_data,q;wire[15:0] d,angle;wire[3:0] ix;wire[31:0] packets,errors;
 stl27l_uart_rx uart(clk,rst,rx,bv,byte_data,fe);
 stl27l_node_parser parser(.clk(clk),.rst_n(rst),.byte_valid(bv),.byte_in(byte_data),.stream_error(fe),
  .point_valid(pv),.point_index(ix),.distance_mm(d),.intensity(q),.angle_q100(angle),.packet_count(packets),.error_count(errors));
 task send(input[7:0] value,input sb);integer k;begin rx=0;#1085.069;for(k=0;k<8;k=k+1)begin rx=value[k];#1085.069;end rx=sb;#1085.069;end endtask
 reg[7:0] data[0:469];integer f,count=0,i;
 always @(negedge clk)if(pv)begin count=count+1;$fwrite(f,"%d %d %d %d\n",ix,d,q,angle);end
 initial begin
  f=$fopen("build/task4/lidar_output.txt","w");$readmemh("build/task4/lidar_input.hex",data);
  repeat(4)@(negedge clk);rst=1;rx=0;#200;rx=1;#15000;
  send(8'h54,0);rx=1;#15000;
  // First packet has a deliberately damaged CRC; the other nine are valid.
  for(i=0;i<470;i=i+1)send(data[i],1);
  #10000;if(packets!=9 || count!=108 || errors<2)$fatal(1,"import UART count %d %d %d",packets,count,errors);
  $fclose(f);$display("PASS imported UART 921600 + CRC recovery + recorded packets + zero crossing");$finish;
 end
 initial begin #7000000;$fatal(1,"UART timeout");end
endmodule
