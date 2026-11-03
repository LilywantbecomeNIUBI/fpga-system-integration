`timescale 1ns/1ps
module tb_router_mirror;
 reg clk=0,rst=0,rv=0,tv=0;reg [7:0] rd=0;reg [511:0] payload=0;reg vs=0;wire vr,vd;reg [7:0] cursor=0;
 always #4 clk=~clk;
 wire txv,tr;wire [7:0] tx;
 network_runtime_dual #(.DEFAULT_IP(0),.TELEMETRY_ENABLE(1),.TELEMETRY_MIRROR_PRIMARY(1)) dut(
 .clk(clk),.rst_n(rst),.rx_clk(clk),.rx_rst_n(rst),.rx_data(rd),.rx_valid(rv),.rx_error(1'b0),
 .video_start(vs),.video_first(1'b1),.video_length(16'd32),.video_data(cursor),.video_read(vr),.video_done(vd),
 .vision_status(256'd0),.vision_status_valid(1'b0),.event_valid(1'b0),.event_payload(256'd0),
 .telemetry_valid(tv),.telemetry_payload(payload),.telemetry_ready(tr),.tx_data(tx),.tx_valid(txv),.motor_ready(1'b0));
 reg [7:0] data[0:79];integer k,fd;reg was=0;
 always @(posedge clk)begin
  if(txv)$fwrite(fd,"%02x\n",tx);else if(was)$fwrite(fd,"END\n");was<=txv;
 end
 always @(posedge clk)if(vr)cursor<=cursor+1;
 task inject(input string name,input integer n);begin
 $readmemh({"build/mecanum/router_tests/",name},data,0,n-1);
 for(k=0;k<n;k=k+1)begin @(negedge clk);rv=1;rd=data[k];end
 @(negedge clk);rv=0;repeat(400)@(negedge clk);
 end endtask
 task offer(input [7:0] value);begin
 @(negedge clk);payload={64{value}};tv=1;
 while(!tr)@(negedge clk);
 @(negedge clk);tv=0;payload=~payload;
 repeat(2500)@(negedge clk);
 end endtask
 initial begin
 fd=$fopen("build/mecanum/router_tests/mirror_tx.hex","w");
 repeat(5)@(negedge clk);rst=1;repeat(20)@(negedge clk);
 inject("set_pc.hex",74);inject("arp_pc.hex",72);
 inject("set_nano.hex",74);inject("arp_nano.hex",72);
 fork
  offer(8'h3c);
  begin @(negedge clk);vs=1;@(negedge clk);vs=0;wait(vd);if(cursor!=32)$fatal(1,"video source consumed twice");end
 join
 inject("stop_pc.hex",74);offer(8'h5a);
 inject("set_pc.hex",74);inject("arp_pc.hex",72);
 inject("stop_nano.hex",74);offer(8'ha5);
 repeat(100)@(negedge clk);$fclose(fd);fd=0;
 $display("PASS independent PC/Nano telemetry mirror and one-endpoint outage");$finish;
 end
 initial begin #1000000;$fatal(1,"mirror timeout");end
endmodule
