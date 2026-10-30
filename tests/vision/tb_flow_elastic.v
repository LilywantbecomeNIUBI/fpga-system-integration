`timescale 1ns/1ps
module tb_flow_elastic;
    reg clk=0,rst=0,ready=1,obs=0;always #5 clk=~clk;
    reg [255:0] data=0;wire valid;wire [255:0] packet;
    flow_event_bridge #(.CLK_HZ(1000),.TIMEOUT_MS(100000)) dut(.clk(clk),.rst_n(rst),.stream_ready(1'b1),
        .observation_valid(obs),.observation_payload(data),.event_ready(ready),.event_valid(valid),.event_payload(packet),
        .stop_request(),.steer(),.speed_limit(),.control_state());
    task vector;input integer id;
        begin data=0;data[255:224]=32'h4f464c57;data[191:176]=id;obs=1;@(negedge clk);obs=0;@(negedge clk);end
    endtask
    integer batch,k,expected;reg [255:0] held;
    initial begin
        repeat(4)@(negedge clk);rst=1;
        repeat(8)@(negedge clk);
        for(batch=0;batch<100;batch=batch+1)begin
            ready=0;vector(batch*3+1);vector(batch*3+2);vector(batch*3+3);
            held=packet;
            repeat(1+batch%9)begin
                @(negedge clk);
                if(!valid || packet!==held || (^packet)===1'bx)$fatal(1,"elastic output changed/unknown while stalled");
            end
            ready=1;
            for(k=1;k<=3;k=k+1)begin
                wait(valid);expected=batch*3+k;
                if(packet[255:224]!=32'h4f464c57 || packet[191:176]!=expected)$fatal(1,"FIFO ordering/wrap got=%0d expected=%0d",packet[191:176],expected);
                @(negedge clk);
            end
            repeat(2)@(negedge clk);
            if(valid)$fatal(1,"duplicate output");
        end
        ready=0;vector(999);rst=0;@(negedge clk);rst=1;
        @(negedge clk);@(negedge clk);
        if(!valid || (^packet)===1'bx || packet[255:224]!=32'h4f465354 || !packet[200])$fatal(1,"reset leaked old/uninitialized data");
        $display("PASS elastic FIFO: 300 ordered vectors, wrap, full/pop/push, long stalls, reset isolation");$finish;
    end
    initial begin #1000000;$fatal(1,"elastic timeout");end
endmodule
