`timescale 1ns/1ps
module tb_vision_lk_tracker;
 reg clk=0,rst=0,fs=0,fe=0,fg=1,fv=0,sv=0;
 always #6 clk=~clk;
 reg [15:0] fid=0,addr=0,feature=0;reg [256:0] seed=0;
 wire valid;wire [255:0] status;
 vision_tracker dut(.clk(clk),.rst_n(rst),.frame_start(fs),.frame_end(fe),.frame_good(fg),
 .frame_id(fid),.feature_valid(fv),.feature_address(addr),.feature(feature),.seed_valid(sv),.seed_data(seed),.status_valid(valid),.status_payload(status));
 reg [15:0] words[0:3599];integer n,k,timeout,log;reg [4095:0] path,folder;reg [15:0] expected_seed;
 task frame;input [15:0] f;input integer index;input good;
 begin
  $sformat(path,"%0s/frame_%0d.hex",folder,index);$readmemh(path,words);
  @(negedge clk);fid=f;fs=1;fg=good;@(negedge clk);fs=0;
  for(k=0;k<3600;k=k+1)begin fv=1;addr=k;feature=words[k];@(negedge clk);end
  fv=0;fe=1;@(negedge clk);fe=0;
 end endtask
 task semantic;input [15:0] f;input matched;
 begin
  @(negedge clk);expected_seed=f;seed={matched,32'h56444554,8'd1,8'd1,f,16'd1280,16'd720,16'd39,16'd900,
    16'd256,16'd192,16'd768,16'd576,16'd0,48'd0};sv=1;@(negedge clk);sv=0;
 end endtask
 task await_frame;input [15:0] f;
 begin
  timeout=0;
  while(!(valid && status[191:176]==f && status[175:160]==expected_seed && status[159:144]==39) && timeout<1000000)begin @(negedge clk);timeout=timeout+1;end
  if(timeout==1000000)$fatal(1,"timeout frame=%d status=%h",f,status);
  $display("OBS %0d %064x",f,status);$fwrite(log,"%0d %064x\n",f,status);
  $fflush();$fflush(log);
 end endtask
 task check_x;input integer expected;
 begin
  if(!status[209] || !status[208] || status[127:112]+6<expected || status[127:112]>expected+6)
   $fatal(1,"invalid LK translation expected x=%0d status=%h",expected,status);
 end endtask
 initial begin
`ifdef LK_NETLIST
  // Match the existing optical-flow netlist harness: vendor primitives use root GSR.
  force GSR.GSRI=1'b1;
`endif
  if(!$value$plusargs("DATA=%s",folder))folder="tests/vision_lk";
  log=$fopen("tests/vision_lk/observations.txt","w");repeat(32)@(negedge clk);rst=1;
  if($test$plusargs("NETLIST_SMOKE"))begin
   frame(100,0,1);frame(101,2,1);semantic(100,1);await_frame(101);check_x(272);
   $fclose(log);$display("PASS netlist complete 16-point LK forward/backward median and historical pair");$finish;
  end
  frame(100,0,1);frame(101,1,1);frame(102,2,1);semantic(100,1);await_frame(102);check_x(272);
  frame(103,3,1);await_frame(103);check_x(288);
  frame(104,2,1);await_frame(104);check_x(272);
  frame(105,4,1);await_frame(105);
  if(status[208] || status[209] || !status[212])$fatal(1,"flat scene must invalidate");
  frame(106,0,1);semantic(106,1);await_frame(106);
  frame(107,5,1);await_frame(107);
  if(status[208] || !status[209] || status[63:48]!=0)$fatal(1,"non-red motion must not fuse red");
  frame(108,0,0);await_frame(108);
  if(status[208] || !status[213])$fatal(1,"bad frame must reject history");
  semantic(108,0);await_frame(108);if(status[208] || status[210])$fatal(1,"no-target clears semantic");
  for(n=109;n<=126;n=n+1)begin frame(n,0,1);repeat(20)@(negedge clk);end
  semantic(100,1);repeat(6000)@(negedge clk);
  if(status[208] || !status[213])$fatal(1,"overwritten seed accepted");
  rst=0;repeat(8)@(negedge clk);rst=1;
  frame(200,0,1);semantic(200,1);await_frame(200);
  for(n=201;n<=215;n=n+1)begin frame(n,0,1);await_frame(n);end
  if(status[208] || status[210] || !status[212])$fatal(1,"expired semantic remained valid");
  rst=0;repeat(8)@(negedge clk);rst=1;
  frame(65534,0,1);frame(65535,1,1);frame(0,2,1);semantic(65534,1);await_frame(0);check_x(272);
  $fclose(log);$display("PASS multi-point LK replay/subpixel/reversal/flat/color/bad frame/history/expiry/rollover");$finish;
 end
 initial begin #2000000000;$fatal(1,"global LK test timeout");end
endmodule
