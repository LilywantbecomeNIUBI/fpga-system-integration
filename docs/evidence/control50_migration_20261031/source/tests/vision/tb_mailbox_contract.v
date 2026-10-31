`timescale 1ns/1ps
// Exercise the held-data contract that justifies the bounded CDC data path.
module tb_mailbox_contract;
    reg sc=0,dc=0,rst=0,valid=0;reg [255:0] data=0;
    real sh=10,dh=4,phase=1.37;
    initial begin
        if($value$plusargs("SRC_HALF=%f",sh))begin end
        if($value$plusargs("DST_HALF=%f",dh))begin end
        forever #(sh)sc=~sc;
    end
    initial begin #(phase);forever #(dh)dc=~dc;end
    wire dv;wire [255:0] dd;
    vision_mailbox dut(.src_clk(sc),.src_rst_n(rst),.src_valid(valid),.src_data(data),
        .dst_clk(dc),.dst_rst_n(rst),.dst_valid(dv),.dst_data(dd));
    reg request=0,outstanding=0;reg [255:0] held;real launched=0;
    integer delivered=0,i;
    always @(negedge sc)if(rst)begin
        if(dut.req!=request)begin
            if(outstanding)$fatal(1,"source changed held data before destination accepted it");
            request=dut.req;held=dut.held;outstanding=1;launched=$realtime-sh;
        end
        if(outstanding && dut.held!==held)$fatal(1,"held bus changed in flight");
    end else begin request=0;outstanding=0;end
    always @(negedge dc)if(rst && dv)begin
        if(!outstanding || dd!==held)$fatal(1,"torn/unrequested/duplicate mailbox data");
        // req must pass two synchronizer stages before the capture edge.
        if($realtime-dh-launched < 4*dh-0.001)$fatal(1,"data captured before minimum bundled-data settling interval");
        outstanding=0;delivered=delivered+1;
    end
    initial begin
        #137;rst=1;
        for(i=1;i<=4000;i=i+1)begin
            @(negedge sc);valid=(i%11)!=0;
            data={8{32'(i*32'h1000101)}};
        end
        @(negedge sc);valid=0;
        repeat(30)@(negedge sc);
        if(dd!==data || delivered<200 || outstanding)$fatal(1,"latest coalesced update not delivered");
        rst=0;#137;rst=1;repeat(20)@(negedge sc);
        if(outstanding || dv)$fatal(1,"common reset replayed an old message");
        $display("PASS mailbox held bus + request/ack timing sh=%f dh=%f delivered=%d",sh,dh,delivered);$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
