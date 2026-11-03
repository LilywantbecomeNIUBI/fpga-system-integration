`timescale 1ns/1ps
module tb_router_commands;
 reg clk=0,control=0,rxclk=0,rst=0,rxvalid=0;reg [7:0] rxdata=0;
 always #4 clk=~clk;always #10 control=~control;always #5 rxclk=~rxclk;
 reg command_error=1;
 wire mv,mr,bv;wire [191:0] mf;wire [7:0] bd;wire [3:0] in1,in2;
 network_runtime_dual #(.DEFAULT_IP(0),.MOTOR_ENABLE(1)) net(
 .clk(clk),.rst_n(rst),.rx_clk(rxclk),.rx_rst_n(rst),.rx_data(rxdata),.rx_valid(rxvalid),.rx_error(1'b0),
 .video_start(1'b0),.video_first(1'b0),.video_length(16'd0),.video_data(8'd0),
 .vision_status(256'd0),.vision_status_valid(1'b0),.event_valid(1'b0),.event_payload(256'd0),
 .telemetry_valid(1'b0),.telemetry_payload(512'd0),.motor_valid(mv),.motor_ready(mr),.motor_frame(mf));
 mc_command_cdc bridge(.src_clk(clk),.src_reset_n(rst),.src_valid(mv),.src_ready(mr),.src_frame(mf),
 .dst_clk(control),.dst_reset_n(rst),.byte_valid(bv),.byte_data(bd));
 mc_mecanum_top #(.CLK_HZ(100000),.IMU_ENABLED(0),.COMMAND_ETHERNET(1),.TELEMETRY_ETHERNET(1)) chassis(
 .clk50m(control),.reset_n(rst),.uart_rx(1'b1),.enc_a(4'd0),.enc_b(4'd0),.in1(in1),.in2(in2),
 .command_error(command_error),.command_byte_valid(bv),.command_byte(bd),
 .auxiliary_valid(1'b0),.auxiliary_frame(384'd0),.telemetry_ready(1'b1));
 reg [7:0] bytes[0:79];integer k;integer commands=0;
 always @(posedge control)if(chassis.command_valid)commands=commands+1;
 task send(input string name,input integer count);
 begin
  $readmemh({"build/mecanum/router_tests/",name},bytes,0,count-1);
  for(k=0;k<count;k=k+1)begin @(negedge rxclk);rxvalid=1;rxdata=bytes[k];end
  @(negedge rxclk);rxvalid=0;
  repeat(250)@(negedge control);
 end endtask
 task check(input expected_arm,input [7:0] seq,input [15:0] faults);
 begin
 if(chassis.armed!==expected_arm || chassis.seq!==seq || chassis.faults!==faults)
 $fatal(1,"state armed=%d seq=%d faults=%h expected=%d/%d/%h",chassis.armed,chassis.seq,chassis.faults,expected_arm,seq,faults);
 end endtask
 initial begin
 repeat(8)@(negedge control);rst=1;repeat(50)@(negedge control);
 if(chassis.bad!==0 || chassis.command_valid!==0)$fatal(1,"network startup fabricated framing errors");
 command_error=0;repeat(12)@(negedge control);
 // No IMU dynamics in this transport test; test zero-displacement pose only.
 force chassis.yaw_ok=1'b1;
 send("udp.hex",78);send("ip.hex",78);send("fragment.hex",78);send("subnet.hex",78);
 send("mac.hex",78);send("fcs.hex",78);send("wrong_port.hex",78);send("short.hex",77);send("long.hex",79);
 check(0,0,1);if(commands!=0)$fatal(1,"malformed envelope escaped");
 send("inner.hex",78);check(0,0,1);if(commands!=0)$fatal(1,"bad inner CRC escaped");
 send("clear.hex",78);check(0,0,0);
 send("arm.hex",78);check(1,2,0);
 send("pose.hex",78);check(1,3,0);
 send("pose.hex",78);check(1,3,0);if(chassis.rejected!=1)$fatal(1,"duplicate pose accepted");
 send("keep.hex",78);check(1,4,0);
 send("turn360.hex",78);check(1,5,0);
 if(chassis.core.goal_angle!==32'sd36000 || !chassis.core.continuous_turn)$fatal(1,"wire extended turn lost angle/flag");
 send("stop.hex",78);check(0,5,0);
 send("arm.hex",78);check(0,5,0); // delayed old ARM cannot restart
 send("arm6.hex",78);check(1,6,0); // legal IPv4 zero UDP checksum
 repeat(26000)@(negedge control);check(0,6,2);
 send("arm6.hex",78);check(0,6,2); // no automatic restart after link loss
 if(in1!==0 || in2!==0)$fatal(1,"outputs after watchdog");
 // Reset both mailbox domains while idle, no phantom command.
 rst=0;repeat(5)@(negedge control);rst=1;repeat(100)@(negedge control);check(0,0,1);
 $display("PASS router wire CRC/IP/UDP/length/port guards, async CDC, pose replay, stale ARM, STOP, 250ms watchdog, reset");$finish;
 end
 initial begin #5000000;$fatal(1,"timeout");end
endmodule
