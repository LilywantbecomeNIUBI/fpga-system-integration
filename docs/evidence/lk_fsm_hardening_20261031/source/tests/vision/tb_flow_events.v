`timescale 1ns/1ps
module tb_flow_events;
    reg clk=0,pclk=0,rxclk=0,rst=0,pclk_enable=1;
    always #4 clk=~clk;always #6 if(pclk_enable)pclk=~pclk;always #4.1 rxclk=~rxclk;
    reg [7:0] rx_data=0;reg rx_valid=0,video_start=0;reg [15:0] cursor=0;
    wire video_read,video_done,txen,cache,enabled,net_ready;wire [7:0] tx;
    wire event_valid;wire [255:0] event_payload;reg allow=1;
    network_runtime_dual #(.DEFAULT_IP(0),.VISION_ENABLE(1),.EVENT_ENABLE(1)) net(
        .clk(clk),.rst_n(rst),.rx_clk(rxclk),.rx_rst_n(rst),.rx_data(rx_data),.rx_valid(rx_valid),.rx_error(1'b0),
        .video_start(video_start),.video_first(1'b1),.video_length(16'd1456),.video_data(cursor[7:0]),.video_read(video_read),.video_done(video_done),
        .tx_data(tx),.tx_valid(txen),.target_ip(),.target_port(),.cache_hit(),.target_mac(),.enabled(),.arp_requests(),.rx_packets(),.rx_debug(),
        .target2_ip(),.target2_port(),.enabled2(enabled),.cache2_hit(cache),.target2_mac(),
        .vision_valid(),.vision_payload(),.vision_status(256'd0),.vision_status_valid(1'b0),
        .event_valid(event_valid && allow),.event_payload(event_payload),.event_ready(net_ready));
    reg fb=0,sv=0,status=0;reg [15:0] fid=700;reg [2:0] state=1;reg stop=0;
    wire source_valid,obs_valid;wire [255:0] source_payload,obs;
    flow_event_source source(.clk(pclk),.rst_n(rst),.frame_begin(fb),.sample_valid(sv),.sample_good(1'b1),.status_valid(status),
        .frame_id(fid),.x(16'd8),.y(16'd16),.u(-16'sd129),.v(16'sd65),.state(state),.hazard(3'd0),.stop_request(stop),
        .steer(2'sd0),.speed_limit(8'd255),.left_valid(16'd20),.center_valid(16'd20),.right_valid(16'd20),
        .left_risk(16'd0),.center_risk(16'd0),.right_risk(16'd0),.cycles(32'd895280),.dropped_frames(32'd7),
        .event_valid(source_valid),.event_payload(source_payload));
    vision_mailbox mb(.src_clk(pclk),.src_rst_n(rst),.src_valid(source_valid),.src_data(source_payload),
        .dst_clk(clk),.dst_rst_n(rst),.dst_valid(obs_valid),.dst_data(obs));
    wire cstop;wire signed [7:0] steer;wire [7:0] speed;wire [2:0] cstate;
    flow_event_bridge #(.CLK_HZ(1000000),.TIMEOUT_MS(100)) bridge(.clk(clk),.rst_n(rst),.stream_ready(cache && enabled),
        .observation_valid(obs_valid),.observation_payload(obs),.event_ready(net_ready && allow),.event_valid(event_valid),.event_payload(event_payload),
        .stop_request(cstop),.steer(steer),.speed_limit(speed),.control_state(cstate));
    integer log;reg tx_previous=0,seen_flow=0,seen_stale=0;reg [255:0] held;
    always @(posedge clk)if(video_read)cursor<=cursor+1'b1;
    always @(negedge clk)begin
        if(txen)$fwrite(log,"%02x",tx);if(tx_previous && !txen)$fwrite(log,"\n");tx_previous=txen;
        if(event_valid && net_ready && allow)begin
            if(event_payload[255:224]==32'h4f464c57)seen_flow=1;
            if(event_payload[255:224]==32'h4f465354 && event_payload[205] && event_payload[200])seen_stale=1;
        end
    end
    task inject;
        input [8*30-1:0] name;reg [7:0] bytes[0:255];integer n,k,fd;reg [8*180-1:0] path;
        begin
            $sformat(path,"tests/optical_flow/io/%0s.len",name);fd=$fopen(path,"r");k=$fscanf(fd,"%d",n);$fclose(fd);
            $sformat(path,"tests/optical_flow/io/%0s.hex",name);$readmemh(path,bytes,0,n-1);
            @(negedge rxclk);rx_valid=1;for(k=0;k<n;k=k+1)begin rx_data=bytes[k];@(negedge rxclk);end
            rx_valid=0;repeat(240)@(negedge clk);
        end
    endtask
    task snapshot;
        input [15:0] id;
        begin @(negedge pclk);fid=id;status=1;@(negedge pclk);status=0;end
    endtask
    initial begin
        log=$fopen("tests/optical_flow/io/network_tx.hex","w");#100;rst=1;
        inject("set_pc");inject("arp_pc");inject("set_nano");inject("arp_nano");
        snapshot(700);wait(!cstop);if(cstate!=1 || speed!=255)$fatal(1,"control bridge decode");
        fork
            begin @(negedge clk);cursor=0;video_start=1;@(negedge clk);video_start=0;wait(video_done);if(cursor!=1456)$fatal(1,"video cursor");end
            begin @(negedge pclk);fb=1;@(negedge pclk);fb=0;sv=1;@(negedge pclk);sv=0;end
        join
        wait(seen_flow);repeat(100)@(negedge clk);
        allow=0;snapshot(701);wait(event_valid);@(negedge clk);held=event_payload;
        snapshot(702);repeat(40)@(negedge clk);if(event_payload!==held)$fatal(1,"payload changed under backpressure");
        allow=1;repeat(500)@(negedge clk);
        @(negedge pclk);pclk_enable=0;
        repeat(101000)@(negedge clk);
        if(!cstop || cstate!=0 || speed!=0 || steer!=0 || !seen_stale)$fatal(1,"stopped PCLK did not stop control");
        inject("stop_nano");repeat(500)@(negedge clk);if(!cstop)$fatal(1,"offline endpoint unsafe");
        $fclose(log);$display("PASS async optical events, signed Q7, immutable backpressure, video coexistence and stopped-PCLK/offline watchdog");$finish;
    end
    initial begin #3000000;$fatal(1,"flow event timeout");end
endmodule
