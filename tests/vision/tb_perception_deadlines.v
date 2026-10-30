`timescale 1ns/1ps
// TTL is measured by the independent Ethernet clock, including refresh exactly
// at a deadline and semantic-result arrival after image ticks have stopped.
module tb_perception_deadlines;
    reg clk=0,rst=0,update=0,tick=0,result_valid=0;
    always #5 clk=~clk;
    reg [255:0] semantic=0,observation=0,result=0;
    wire event_valid;wire [255:0] event_payload;wire [3:0] led;
    vision_v23_bridge #(.CLK_HZ(1000),.TIMEOUT_MS(100)) bridge(
        .clk(clk),.rst_n(rst),.stream_ready(1'b1),.semantic_update(update),.semantic_valid(1'b1),
        .semantic_payload(semantic),.observation_valid(update),.observation_payload(observation),
        .event_ready(1'b1),.event_valid(event_valid),.event_payload(event_payload),.led(led));
    wire target,done,accepted;wire [255:0] status;
    vision_result_core #(.CLK_HZ(1000),.TIMEOUT_MS(100)) core(
        .clk(clk),.rst_n(rst),.stream_ready(1'b1),.frame_tick(tick),.frame_id(16'd1),
        .result_valid(result_valid),.result_payload(result),.target_valid(target),.target_direction(),
        .target_class(),.target_confidence(),.target_x(),.target_y(),.target_width(),.target_height(),
        .target_age(),.status_payload(status),.result_done(done),.accepted_pulse(accepted),.accepted_payload());
    initial begin
        semantic[207:192]=1;
        observation[255:216]={32'h46535441,8'd2};
        observation[215:208]=7;observation[175:160]=1;observation[199:192]=2;
        result={32'h56444554,8'd1,8'd1,16'd1,16'd1280,16'd720,16'd39,16'd900,
                16'd256,16'd192,16'd768,16'd576,16'd0,48'd0};
        repeat(4)@(negedge clk);rst=1;update=1;tick=1;
        @(negedge clk);update=0;
        repeat(98)begin @(negedge clk);if(!led[0])$fatal(1,"tracking TTL expired early");end
        @(negedge clk);@(negedge clk);if(led[0])$fatal(1,"tracking TTL missed expiry");
        update=1;@(negedge clk);update=0;
        @(negedge clk);if(!led[0])$fatal(1,"tracking refresh did not restore observation");
        result_valid=1;@(negedge clk);result_valid=0;
        wait(done);@(negedge clk);
        if(!accepted || !target)$fatal(1,"fresh semantic result rejected");
        repeat(99)begin @(negedge clk);if(!target)$fatal(1,"semantic TTL expired early");end
        @(negedge clk);if(target || core.reason!=6)$fatal(1,"semantic TTL missed expiry");
        tick=0;repeat(105)@(negedge clk);
        result_valid=1;@(negedge clk);result_valid=0;
        wait(done);@(negedge clk);
        if(accepted || target || core.reason!=4)$fatal(1,"stopped camera accepted a result");
        $display("PASS tracking and semantic TTL/refresh, stopped-camera result rejection");$finish;
    end
    initial begin #100000;$fatal(1,"deadline test timeout");end
endmodule
