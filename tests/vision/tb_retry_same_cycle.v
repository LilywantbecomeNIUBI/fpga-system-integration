`timescale 1ns/1ps
// Compare with the unmodified 0-setup/hold checkpoint. Cache inputs are changed
// before the edge; the new local lookup predicates themselves are NEVER forced.
module tb_retry_same_cycle;
 parameter HZ=125000000,CACHE=30;
 reg clk=0,rst=0;always #4 clk=~clk;
 reg cv0=0,cv1=0,c0=0,c1=0,tick=0;
 reg [7:0] age0=0,age1=0,op=0;
 reg [31:0] ip0=0,ip1=0;
 reg [3:0] ps=0,action=0;reg [2:0] ss=0,a0=0,a1=0;
 integer i;reg [31:0] random_state=32'h61497821;
 `define PORTS .clk(clk),.rst_n(rst),.rx_clk(clk),.rx_rst_n(rst),.rx_data(8'd0),.rx_valid(1'b0),.rx_error(1'b0),.video_start(1'b0),.video_first(1'b0),.video_length(16'd0),.video_data(8'd0),.vision_status(256'd0),.vision_status_valid(1'b0),.event_valid(1'b0),.event_payload(256'd0),.telemetry_valid(1'b0),.telemetry_payload(512'd0)
 network_runtime_dual #(.CLK_HZ(HZ),.CACHE_SECONDS(CACHE)) dut(`PORTS);
 network_runtime_dual_reference #(.CLK_HZ(HZ),.CACHE_SECONDS(CACHE)) ref_dut(`PORTS);
 `define INPUTS(D) \
  force D.cache_valid0=cv0;force D.cache_valid1=cv1; \
  force D.cache_age0=age0;force D.cache_age1=age1; \
  force D.cache_ip0=ip0;force D.cache_ip1=ip1; \
  force D.target_ip=32'hc0a8320a;force D.target2_ip=32'hc0a83214; \
  force D.clear_cache=c0;force D.clear_cache2=c1;force D.clear_cache2_disp=c1; \
  force D.parse_state=ps;force D.send_state=ss;force D.action=action; \
  force D.retry_tick=tick;force D.attempts=a0;force D.attempts2=a1; \
  force D.enabled=1;force D.enabled2=1; \
  force D.command_ok=1;force D.requested_ok0=1;force D.requested_ok1=1;
 initial begin
  `INPUTS(dut)
  `INPUTS(ref_dut)
  // Isolate the cache/lookup experiment from parser-generated invalidation
  // pulses; aliases receive the identical modeled current-cycle input.
  force dut.clear_cache_retry=c0;force dut.clear_cache2_retry=c1;force dut.clear_cache_disp=c0;
  force dut.retry_target_ip0=32'hc0a8320a;force dut.retry_target_ip1=32'hc0a83214;
  repeat(3)@(negedge clk);rst=1;
  for(i=0;i<50000;i=i+1)begin
   random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
   cv0=random_state[0];cv1=random_state[1];c0=random_state[5:2]==0;c1=random_state[9:6]==0;
   ip0=random_state[10]?32'hc0a8320a:32'hc0a83211;
   ip1=random_state[11]?32'hc0a83214:32'hc0a83212;
   age0=i%40;age1=(i/3)%40;tick=random_state[12];
   case(i%6)0:op=2;1:op=4;2:op=5;3:op=8;default:op=7;endcase
   ps=i%7==0?3:i%7==1?5:0;ss=random_state[13]?4:0;
   action=random_state[14]?4:5;a0=i%5;a1=(i+2)%5;
   dut.memory[47]=op;ref_dut.memory[47]=op;
   if(i%1001==1000)begin rst=0;@(negedge clk);rst=1;end
   @(negedge clk);
   if(dut.retry_hit_clear0!==(dut.lookup_valid && !dut.refresh_due) ||
      dut.retry_hit_clear1!==(dut.lookup2_valid && !dut.refresh2_due))$fatal(1,"same-edge cache predicate mismatch");
   if({dut.retry_count,dut.retry_count2,dut.retry_borrow,dut.retry_borrow2,dut.arp_due0,dut.arp_due1,dut.arp_issue} !==
      {ref_dut.retry_count,ref_dut.retry_count2,ref_dut.retry_borrow,ref_dut.retry_borrow2,ref_dut.arp_due0,ref_dut.arp_due1,ref_dut.arp_issue})
      $fatal(1,"cycle-equivalence mismatch at %d",i);
  end
  $display("PASS same-cycle retry/dispatch equivalence: 50000 cache/age/invalidation/reset combinations HZ=%d CACHE=%d",HZ,CACHE);$finish;
 end
endmodule
