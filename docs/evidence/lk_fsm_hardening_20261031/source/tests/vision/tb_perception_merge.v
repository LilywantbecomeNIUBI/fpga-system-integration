`timescale 1ns/1ps
// Both independent producers contend for the same packet interface. A stopped
// PCLK is modeled by withholding updates; watchdogs run in the network clock.
module tb_perception_merge;
    reg clk=0,rst_n=0,ready=0,stream_ready=1,update=0;
    always #5 clk=~clk;
    wire av,bv,ar,br,valid,stop;
    wire [255:0] ap,bp,payload;
    reg [255:0] observation=0;
    flow_event_bridge #(.CLK_HZ(1000),.TIMEOUT_MS(100)) flow(
        .clk(clk),.rst_n(rst_n),.stream_ready(stream_ready),
        .observation_valid(update),.observation_payload(observation),
        .event_ready(br),.event_valid(bv),.event_payload(bp),.stop_request(stop),
        .steer(),.speed_limit(),.control_state());
    // Force a continuous stream of target events to demonstrate flow fairness.
    assign av=1;assign ap={32'h46535441,8'd2,216'd0};
    event_packet_mux mux(.clk(clk),.rst_n(rst_n),.a_valid(av),.a_payload(ap),.a_ready(ar),
        .b_valid(bv),.b_payload(bp),.b_ready(br),.out_valid(valid),.out_payload(payload),.out_ready(ready));
    integer targets=0,statuses=0,stale=0,file;
    reg [255:0] held;
    always @(posedge clk)if(valid && ready)begin
        if(payload[255:224]==32'h46535441)targets=targets+1;
        if(payload[255:224]==32'h4f465354)begin
            statuses=statuses+1;
            if(payload[205] && payload[200])stale=stale+1;
        end
        $fdisplay(file,"%064x",payload);
    end
    initial begin
        file=$fopen("build/tests/perception_events.hex","w");
        repeat(4)@(negedge clk);rst_n=1;
        repeat(5)@(negedge clk);held=payload;
        // A fresh forward proposal and subsequent stale mask must not mutate
        // the packet currently held by the network.
        observation={32'h4f465354,8'd1,8'd2,8'd2,8'd1,16'd1,
                     16'd10,16'd10,16'd10,16'd0,16'd0,16'd0,8'd255,8'd0,32'd1,32'd0};
        update=1;@(negedge clk);update=0;
        if(stop)$fatal(1,"fresh flow unavailable");
        repeat(110)begin @(negedge clk);if(payload!==held)$fatal(1,"packet changed while stalled");end
        if(!stop)$fatal(1,"stopped camera did not expire independently");
        ready=1;repeat(30)@(negedge clk);
        if(targets<5 || statuses<2 || stale<1)$fatal(1,"missing fair/masked events");
        // Check the original deadline exactly, including refresh at expiry.
        update=1;@(negedge clk);update=0;
        repeat(98)begin @(negedge clk);if(stop)$fatal(1,"watchdog expired early");end
        @(negedge clk);if(!stop)$fatal(1,"watchdog missed deadline");
        update=1;@(negedge clk);update=0;
        if(stop)$fatal(1,"fresh status did not clear deadline");
        repeat(98)begin @(negedge clk);if(stop)$fatal(1,"refresh inherited old deadline");end
        stream_ready=0;repeat(10)@(negedge clk);
        if(!stop)$fatal(1,"offline endpoint did not mask");
        $fclose(file);$display("PASS merged events: fairness, immutable stall, exact watchdog deadline/refresh and endpoint loss");$finish;
    end
endmodule
