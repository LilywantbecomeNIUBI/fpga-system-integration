`timescale 1ns/1ps
// Behavioral boundaries that can break when arithmetic/expiry is pipelined.
module tb_timing_boundaries;
    reg clk=0,rst=0;always #5 clk=~clk;
    reg fb=0,fd=0,inv=0,sv=0,good=1;reg [15:0] fid=0,x=0;
    wire status,stop;wire [15:0] status_id,nl,nc,nr,rl,rc,rr;
    flow_avoidance #(.MIN_VALID(1),.CLEAR_FRAMES(1),.TIMEOUT_CYCLES(1000)) avoidance(
        .clk(clk),.rst_n(rst),.frame_begin(fb),.frame_done(fd),.invalidate(inv),
        .sample_valid(sv),.sample_good(good),.frame_id(fid),.x(x),.y(16'd45),
        .u(16'sd0),.v(16'sd0),.status_valid(status),.status_id(status_id),
        .state(),.hazard(),.stop_request(stop),.steer(),.speed_limit(),
        .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr));
    wire event_valid;wire [255:0] event_payload;
    flow_event_source event_source(.clk(clk),.rst_n(rst),.frame_begin(fb),.sample_valid(sv),.sample_good(good),
        .status_valid(status),.frame_id(status ? status_id : fid),.x(x),.y(16'd45),.u(16'sd0),.v(16'sd0),
        .state(3'd1),.hazard(3'd0),.stop_request(stop),.steer(2'sd0),.speed_limit(8'd255),
        .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr),
        .cycles(32'd0),.dropped_frames(32'd0),.event_valid(event_valid),.event_payload(event_payload));

    reg tick=0,result_valid=0;reg [15:0] frame=0;reg [255:0] payload=0;
    wire accepted,done,target;wire [15:0] target_age;
    vision_result_core #(.CLK_HZ(1000),.TIMEOUT_MS(1000),.MAX_AGE_FRAMES(3)) semantics(
        .clk(clk),.rst_n(rst),.stream_ready(1'b1),.frame_tick(tick),.frame_id(frame),
        .result_valid(result_valid),.result_payload(payload),.target_valid(target),.target_direction(),
        .target_class(),.target_confidence(),.target_x(),.target_y(),.target_width(),.target_height(),
        .target_age(target_age),.status_payload(),.result_done(done),.accepted_pulse(accepted),.accepted_payload());

    network_runtime_dual #(.CLK_HZ(7),.DEFAULT_IP(0)) network(
        .clk(clk),.rst_n(rst),.rx_clk(clk),.rx_rst_n(rst),.rx_data(8'd0),.rx_valid(1'b0),.rx_error(1'b0),
        .video_start(1'b0),.video_first(1'b0),.video_length(16'd0),.video_data(8'd0),
        .vision_status(256'd0),.vision_status_valid(1'b0),.event_valid(1'b0),.event_payload(256'd0));
    integer cycles=0;
    always @(negedge clk)if(rst)begin
        if(network.second_count!==cycles%7)$fatal(1,"second counter period shifted: cycle=%0d count=%0d",cycles,network.second_count);
        cycles=cycles+1;
    end

    task new_frame;input [15:0] id;
        begin @(negedge clk);fb=1;fid=id;@(negedge clk);fb=0;end
    endtask
    task semantic_result;input [15:0] id;
        begin
            @(negedge clk);
            payload={32'h56444554,8'd1,8'd1,id,16'd1280,16'd720,16'd39,16'd900,
                     16'd256,16'd192,16'd768,16'd576,16'd0,48'd0};
            result_valid=1;@(negedge clk);result_valid=0;
            wait(done);@(negedge clk);
            if(!accepted || !target)$fatal(1,"fresh semantic result rejected after pipeline expiry");
        end
    endtask
    integer k;
    initial begin
        repeat(4)@(negedge clk);@(posedge clk);#1;rst=1;
        new_frame(16'd42);
        // Consecutive samples in all zones; frame_done coincides with the last.
        for(k=0;k<9;k=k+1)begin
            @(negedge clk);sv=1;x=20+(k%3)*60;fd=k==8;
        end
        @(negedge clk);sv=0;fd=0;fid=99;
        wait(status);@(negedge clk);
        if(stop || status_id!=42 || nl!=3 || nc!=3 || nr!=3 || rl || rc || rr)
            $fatal(1,"pipeline lost final sample, zone or frame ID");
        @(negedge clk);
        if(!event_valid || event_payload[255:224]!=32'h4f465354 || event_payload[191:176]!=42)
            $fatal(1,"status packet used the live frame ID instead of the drained frame");
        new_frame(43);
        @(negedge clk);sv=1;x=20;
        @(negedge clk);sv=0;inv=1;
        @(negedge clk);inv=0;
        if(!stop)$fatal(1,"in-flight invalidation failed to stop immediately");
        // An aborted frame must not add a delayed sample into a new frame.
        new_frame(44);repeat(6)@(negedge clk);
        if(avoidance.n[0] || avoidance.n[1] || avoidance.n[2])$fatal(1,"old sample leaked into new frame");

        frame=16'hfffe;tick=1;@(negedge clk);tick=0;
        semantic_result(16'hfffe);
        @(negedge clk);frame=1;tick=1;@(negedge clk);tick=0;
        repeat(3)@(negedge clk);
        if(!target || target_age!=3)$fatal(1,"frame-age wrap or inclusive limit changed");
        frame=2;tick=1;@(negedge clk);tick=0;
        repeat(3)@(negedge clk);
        if(target || semantics.reason!=4)$fatal(1,"pipelined frame-age expiry missed");
        semantic_result(2);
        repeat(6)@(negedge clk);
        if(!target || target_age!=0)$fatal(1,"previous sequence expiry cleared refreshed target");
        $display("PASS timing boundaries: contiguous/final samples, frame ownership, invalidation, frame-age wrap/refresh, exact second period");
        $finish;
    end
    initial begin #100000;$fatal(1,"timing boundaries timeout");end
endmodule
