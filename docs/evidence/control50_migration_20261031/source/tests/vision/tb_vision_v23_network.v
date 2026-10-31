`timescale 1ns/1ps
module tb_vision_v23_network;
    reg clk=0,pclk=0,rxclk=0,rst=0,pclk_enable=1;
    always #4 clk=~clk;always #4.1 rxclk=~rxclk;always #6 if(pclk_enable)pclk=~pclk;
    reg [7:0] rx_data=0;reg rx_valid=0;
    reg start=0,tick=0;reg [15:0] current_fid=102;reg [10:0] cursor=0;
    wire read,done,txen,cache,enabled,result_valid,result_done,semantic_update,semantic_target;
    wire [7:0] tx;wire [255:0] result,status,semantic_payload;
    wire event_valid,event_ready;wire [255:0] event_payload;wire [3:0] led;
    network_runtime_dual #(.DEFAULT_IP(0),.VISION_ENABLE(1),.EVENT_ENABLE(1)) net(
        .clk(clk),.rst_n(rst),.rx_clk(rxclk),.rx_rst_n(rst),.rx_data(rx_data),.rx_valid(rx_valid),.rx_error(1'b0),
        .video_start(start),.video_first(1'b1),.video_length(16'd1456),.video_data(cursor[7:0]),.video_read(read),.video_done(done),
        .tx_data(tx),.tx_valid(txen),.target_ip(),.target_port(),.cache_hit(),.target_mac(),.enabled(),.arp_requests(),.rx_packets(),.rx_debug(),
        .target2_ip(),.target2_port(),.enabled2(enabled),.cache2_hit(cache),.target2_mac(),
        .vision_valid(result_valid),.vision_payload(result),.vision_status(status),.vision_status_valid(result_done),
        .event_valid(event_valid),.event_payload(event_payload),.event_ready(event_ready));
    vision_result_core core(.clk(clk),.rst_n(rst),.stream_ready(cache && enabled),.frame_tick(tick),.frame_id(current_fid),
        .result_valid(result_valid),.result_payload(result),.target_valid(semantic_target),.target_direction(),.target_class(),.target_confidence(),
        .target_x(),.target_y(),.target_width(),.target_height(),.target_age(),.status_payload(status),.result_done(result_done),
        .accepted_pulse(semantic_update),.accepted_payload(semantic_payload));
    wire seed_valid,obs_valid,observation_valid;wire [256:0] seed;wire [255:0] obs,observation;
    vision_mailbox #(.WIDTH(257)) a(.src_clk(clk),.src_rst_n(rst),.src_valid(semantic_update),.src_data({semantic_target,semantic_payload}),
        .dst_clk(pclk),.dst_rst_n(rst),.dst_valid(seed_valid),.dst_data(seed));
    reg fs=0,fe=0,fv=0;reg [15:0] fid=0,addr=0,feature=0;
    vision_sad_tracker track(.clk(pclk),.rst_n(rst),.frame_start(fs),.frame_end(fe),.frame_good(1'b1),.frame_id(fid),
        .feature_valid(fv),.feature_address(addr),.feature(feature),.seed_valid(seed_valid),.seed_data(seed),.status_valid(obs_valid),.status_payload(obs));
    vision_mailbox b(.src_clk(pclk),.src_rst_n(rst),.src_valid(obs_valid),.src_data(obs),
        .dst_clk(clk),.dst_rst_n(rst),.dst_valid(observation_valid),.dst_data(observation));
    vision_v23_bridge #(.TIMEOUT_MS(1)) bridge(.clk(clk),.rst_n(rst),.stream_ready(cache && enabled),.semantic_update(semantic_update),
        .semantic_valid(semantic_target),.semantic_payload(semantic_payload),.observation_valid(observation_valid),.observation_payload(observation),
        .event_ready(event_ready),.event_valid(event_valid),.event_payload(event_payload),.led(led));
    integer log;reg previous=0,seen_track=0,seen_timeout=0;
    always @(posedge clk)if(read)cursor<=cursor+1'b1;
    always @(negedge clk)begin
        if(txen)$fwrite(log,"%02x",tx);if(previous && !txen)$fwrite(log,"\n");previous=txen;
        if(event_valid && event_ready && event_payload[191:176]==102 && event_payload[208] && event_payload[209])seen_track=1;
        if(event_valid && event_ready && !event_payload[208] && event_payload[207:200]==9)seen_timeout=1;
    end
    task inject;
        input [8*40-1:0] name;reg [7:0] bytes[0:255];integer n,k,fd;reg [8*200-1:0] path;
        begin
            $sformat(path,"tests/vision_v23/%0s.len",name);fd=$fopen(path,"r");k=$fscanf(fd,"%d",n);$fclose(fd);
            $sformat(path,"tests/vision_v23/%0s.hex",name);$readmemh(path,bytes,0,n-1);
            @(negedge rxclk);rx_valid=1;for(k=0;k<n;k=k+1)begin rx_data=bytes[k];@(negedge rxclk);end
            rx_valid=0;repeat(240)@(negedge clk);
        end
    endtask
    task frame;
        input integer n;reg [15:0] words[0:3599];reg [8*160-1:0] path;integer k;
        begin
            $sformat(path,"tests/vision_v23/full_feature_%0d.hex",n);$readmemh(path,words);
            @(negedge pclk);fid=n;fs=1;@(negedge pclk);fs=0;
            for(k=0;k<3600;k=k+1)begin fv=1;addr=k;feature=words[k];@(negedge pclk);end
            fv=0;fe=1;@(negedge pclk);fe=0;
        end
    endtask
    initial begin
        log=$fopen("tests/vision_v23/network_tx.hex","w");#100;rst=1;
        inject("set_pc");inject("arp_pc");inject("set_nano");inject("arp_nano");
        frame(100);frame(101);frame(102);
        @(negedge clk);tick=1;@(negedge clk);tick=0;
        fork
            begin @(negedge clk);cursor=0;start=1;@(negedge clk);start=0;wait(done);if(cursor!=1456)$fatal(1,"source count");end
            inject("semantic_100");
        join
        wait(seen_track);if(!led[0])$fatal(1,"fused LED");
        @(negedge pclk);pclk_enable=0;
        repeat(150000)@(negedge clk);if(led!=0 || !seen_timeout)$fatal(1,"stopped PCLK did not fail closed");
        inject("query");repeat(300)@(negedge clk);$fclose(log);
        $display("PASS UDP feedback -> accepted seed -> asynchronous history tracker -> fused UDP events; stopped-PCLK watchdog");$finish;
    end
    initial begin #6000000;$fatal(1,"network perception timeout state=%d %h",track.state,event_payload);end
endmodule
