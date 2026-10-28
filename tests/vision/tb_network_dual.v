`timescale 1ns/1ps
module tb_network_dual;
    reg clk=0,rxclk=0,rst_n=0;always #4 clk=~clk;always #4.1 rxclk=~rxclk;
    reg [7:0] rx_data=0;reg rx_valid=0,rx_error=0;
    reg start=0,first=1;reg [15:0] length=1456;
    wire done,read,en,hit,hit2,enabled,enabled2;wire [7:0] tx;
    wire [31:0] target,target2;wire [15:0] port,port2;wire [47:0] mac,mac2;
    reg [10:0] cursor=0;integer log;reg previous_en=0;
    wire [7:0] source_byte=cursor[7:0]^{5'd0,cursor[10:8]};
    network_runtime_dual #(.DEFAULT_IP(0),.CLK_HZ(200000),.CACHE_SECONDS(2)) dut(
        .clk(clk),.rst_n(rst_n),.rx_clk(rxclk),.rx_rst_n(rst_n),.rx_data(rx_data),.rx_valid(rx_valid),.rx_error(rx_error),
        .video_start(start),.video_first(first),.video_length(length),.video_data(source_byte),.video_read(read),.video_done(done),
        .tx_data(tx),.tx_valid(en),.target_ip(target),.target_port(port),.cache_hit(hit),.target_mac(mac),.enabled(enabled),
        .target2_ip(target2),.target2_port(port2),.cache2_hit(hit2),.target2_mac(mac2),.enabled2(enabled2),
        .arp_requests(),.rx_packets(),.rx_debug());
    always @(posedge clk)if(read)cursor<=cursor+1'b1;
    always @(negedge clk)begin if(en)$fwrite(log,"%02x\n",tx);else if(previous_en)$fwrite(log,"END\n");previous_en=en;end
    task inject;
        input [8*40-1:0] name;reg [7:0] bytes[0:127];integer n,k,fd;reg [8*200-1:0] path;
        begin
            $sformat(path,"tests/network_dual_20261014/%0s.len",name);fd=$fopen(path,"r");k=$fscanf(fd,"%d",n);$fclose(fd);
            $sformat(path,"tests/network_dual_20261014/%0s.hex",name);$readmemh(path,bytes,0,n-1);
            @(negedge rxclk);rx_valid=1;
            for(k=0;k<n;k=k+1)begin rx_data=bytes[k];@(negedge rxclk);end
            rx_valid=0;rx_data=0;repeat(220)@(negedge clk);
        end
    endtask
    task send_video;
        input is_first;input [15:0] size;
        begin
            @(negedge clk);cursor=0;length=size;first=is_first;start=1;@(negedge clk);start=0;
            wait(done);if(cursor!=size)$fatal(1,"Source consumed more/less than once: %d != %d",cursor,size);
            repeat(150)@(negedge clk);
        end
    endtask
    initial begin
        log=$fopen("tests/network_dual_20261014/tx.hex","w");#100;rst_n=1;
        inject("set_pc");inject("arp_pc");inject("set_nano");inject("arp_nano");
        if(!hit || !hit2 || target!=32'hc0a83264 || target2!=32'hc0a83214)$fatal(1,"Independent .100/.20 caches failed");
        send_video(1,1456);send_video(0,31);send_video(0,1);
        inject("bad_fcs");inject("bad_ip");inject("bad_udp");inject("fragment");inject("off_subnet");inject("duplicate_target");
        if(target2!=32'hc0a83214)$fatal(1,"Invalid command changed Nano target");
        inject("clear_pc");if(hit || !hit2)$fatal(1,"Clearing PC cache interfered with Nano");
        send_video(0,31);inject("arp_pc");send_video(0,31);send_video(1,31);
        inject("stop_pc");if(enabled || !enabled2)$fatal(1,"Stopping PC affected Nano");send_video(0,31);
        inject("set_pc");inject("arp_pc");
        fork
            send_video(1,1456);
            begin wait(cursor==32);inject("set_nano_new");end
        join
        if(target2!=32'hc0a83215)$fatal(1,"In-flight target update failed");
        send_video(0,31);inject("arp_nano_new");send_video(0,31);send_video(1,31);
        inject("stop_nano");send_video(0,31);
        inject("set_nano");send_video(0,31); // Offline Nano must not stall PC.
        inject("arp_nano");send_video(1,31);
        inject("clear_nano");if(!hit || hit2)$fatal(1,"Nano cache clear affected PC");
        inject("arp_nano");
        repeat(410000)@(negedge clk);if(hit || hit2)$fatal(1,"Endpoint cache TTL failed");
        inject("status_pc");inject("status_nano");repeat(250)@(negedge clk);$fclose(log);
        $display("PASS dual cache collision, identical replay, source once, frame gate, independent stop/clear, target snapshots, bad packet rejection and TTL");$finish;
    end
    initial begin #10000000;$fatal(1,"Dual simulation timeout");end
endmodule
