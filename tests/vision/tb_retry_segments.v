`timescale 1ns/1ps
// Exercise the production counters with independent integer arithmetic.
module tb_retry_segments;
    parameter HZ=125000000;
    reg clk=0,rst=0;always #4 clk=~clk;
    network_runtime_dual #(.CLK_HZ(HZ)) dut(.clk(clk),.rst_n(rst),.rx_clk(clk),.rx_rst_n(rst),
        .rx_data(8'd0),.rx_valid(1'b0),.rx_error(1'b0),.video_start(1'b0),.video_first(1'b0),
        .video_length(16'd0),.video_data(8'd0),.vision_status(256'd0),.vision_status_valid(1'b0),
        .event_valid(1'b0),.event_payload(256'd0),.telemetry_valid(1'b0),.telemetry_payload(512'd0));
    reg tick=0,hit0=0,hit1=0,refresh0=0,refresh1=0,ok=1,req0=1,req1=1;
    reg [3:0] op=0,ps=0,act=0;reg [2:0] ss=0,a0=0,a1=0;
    wire hit_clear0=hit0 && !refresh0,hit_clear1=hit1 && !refresh1;
    wire need_arp0=!hit_clear0,need_arp1=!hit_clear1;
    integer ref0=0,ref1=0,cycles=0,i,j;
    localparam SHORT=HZ>=1000?500:HZ/2,LONG=HZ>=1000?2000:HZ*2;
    initial begin
        force dut.retry_tick=tick;force dut.found=hit0;force dut.found2=hit1;
        force dut.refresh_due=refresh0;force dut.refresh2_due=refresh1;
        // This arithmetic unit test bypasses cache lookup. Drive its new local
        // predicate replicas consistently; tb_retry_same_cycle independently
        // verifies those replicas from actual pre-edge cache inputs.
        force dut.retry_hit_clear0=hit_clear0;force dut.retry_hit_clear1=hit_clear1;
        force dut.retry_hit_clear0_disp=hit_clear0;force dut.retry_hit_clear1_disp=hit_clear1;
        force dut.clear_cache_retry=0;force dut.clear_cache2_retry=0;
        force dut.clear_cache_disp=0;
        force dut.arp_needed0_disp=need_arp0;force dut.arp_needed1_disp=need_arp1;
        force dut.command_ok=ok;force dut.requested_ok0=req0;force dut.requested_ok1=req1;
        force dut.parse_state=ps;force dut.send_state=ss;force dut.action=act;
        force dut.enabled=1;force dut.enabled2=1;force dut.arp_due0=1;force dut.arp_due1=1;
        force dut.attempts=a0;force dut.attempts2=a1;
        // Drive the immutable mailbox opcode as a testbench RX write.
        dut.memory[47]=0;
        #1;rst=0;repeat(2)@(negedge clk);rst=1;
        for(i=0;i<80;i=i+1)begin
            // Reload short/long with both endpoint counters and cross every borrow.
            a0=i%5;a1=(i+2)%5;
            ss=4;act=4;@(negedge clk);act=5;@(negedge clk);ss=0;act=0;
            for(j=0;j<2100;j=j+1)begin tick=(j%13)!=0;@(negedge clk);end
            tick=0;
            // Stable command through the checksum wait, then one command edge.
            op=(i%4==0)?2:(i%4==1)?4:(i%4==2)?5:8;
            dut.memory[47]=op;req0=i%3!=0;req1=i%3!=1;ok=i%7!=0;
            ps=5;repeat(3)@(negedge clk);ps=3;@(negedge clk);ps=0;
            // Reload coincident with command: original dispatch priority wins.
            ps=5;repeat(3)@(negedge clk);ps=3;ss=4;act=i%2?4:5;@(negedge clk);ps=0;ss=0;act=0;
            hit0=1;hit1=1;refresh0=i%2;refresh1=!(i%2);tick=1;repeat(7)@(negedge clk);
            hit0=0;hit1=0;refresh0=0;refresh1=0;
            if(i%10==0)begin rst=0;repeat(2)@(negedge clk);rst=1;end
        end
        $display("PASS retry arithmetic/borrow, hold, zero, short/long, invalid commands, command/reload priority HZ=%d cycles=%d",HZ,cycles);$finish;
    end
    always @(posedge clk)begin
        if(!rst)begin ref0=0;ref1=0;end
        else begin
            if(tick && ref0!=0)ref0=ref0-1;
            if(tick && ref1!=0)ref1=ref1-1;
            if((hit0 && !refresh0) || (ps==3 && ok && ((op==2 && req0) || op==4)))ref0=0;
            if((hit1 && !refresh1) || (ps==3 && ok && ((op==5 && req1) || op==8)))ref1=0;
            if(ss==4 && act==4 && (!hit0 || refresh0))ref0=a0<3?SHORT:LONG;
            if(ss==4 && act==5 && (!hit1 || refresh1))ref1=a1<3?SHORT:LONG;
        end
        #1;cycles=cycles+1;
        if(dut.retry_count!==ref0 || dut.retry_count2!==ref1)
            $fatal(1,"counter mismatch cycle %d actual %d/%d reference %d/%d",cycles,dut.retry_count,dut.retry_count2,ref0,ref1);
        if(dut.retry_borrow!==(dut.retry_count % (1<<dut.RETRY_LO)==0) ||
           dut.retry_borrow2!==(dut.retry_count2 % (1<<dut.RETRY_LO)==0))$fatal(1,"borrow invariant");
        if(dut.clear_cache2_disp!==dut.clear_cache2 || dut.lookup2_valid_disp!==dut.lookup2_valid)$fatal(1,"dispatch replica diverged");
    end
endmodule
