`timescale 1ns/1ps
// Real scan RAM -> frame mux -> asynchronous mailbox -> production UDP MAC.
module tb_lidar_ethernet;
    reg clk=0,ethclk=0,rxclk=0,rst_n=0,pv=0;
    always #10 clk=~clk;always #4 ethclk=~ethclk;always #4.1 rxclk=~rxclk;
    reg [15:0] angle=0,distance=0;reg [7:0] intensity=0;
    wire [383:0] scan,selected;wire scan_valid,scan_ready,selected_valid,selected_ready,unused_tx;
    lidar_scan_360_source #(.CLK_HZ(1000000)) source(.clk(clk),.rst_n(rst_n),
        .byte_valid(pv),.framing_error(1'b0),.point_valid(pv),.angle(angle),.distance(distance),.intensity(intensity),
        .packets(32'd123),.errors(32'd0),.crc_errors(32'd0),.format_errors(32'd0),.timeouts(32'd0),
        .frame(scan),.valid(scan_valid),.ready(scan_ready));
    reg motor_valid=0;reg [383:0] motor_memory[0:0];
    frame_uart_mux #(.CLK_HZ(1000000),.AUX_ENABLED(1),.ETHERNET(1)) mux(
        .clk(clk),.reset_n(rst_n),.motor_valid(motor_valid),.motor_frame(motor_memory[0]),
        .imu_valid(1'b0),.imu_frame(384'd0),.aux_valid(scan_valid && scan_ready),.aux_frame(scan),
        .aux_ready(scan_ready),.aux_drops(),.imu_drops(),.tx(unused_tx),
        .ethernet_frame(selected),.ethernet_valid(selected_valid),.ethernet_ready(selected_ready));
    wire [511:0] payload;wire payload_valid,payload_ready;
    telemetry_cdc #(.CLK_HZ(1000000)) mailbox(.src_clk(clk),.src_rst_n(rst_n),
        .src_valid(selected_valid),.src_frame(selected),.src_ready(selected_ready),
        .dst_clk(ethclk),.dst_rst_n(rst_n),.dst_valid(payload_valid),.dst_payload(payload),.dst_ready(payload_ready));
    reg [7:0] rx_data=0;reg rx_valid=0;
    wire [7:0] tx;wire tx_valid,video_read,video_done;
    reg video_start=0;reg [15:0] video_length=1456;reg [10:0] cursor=0;
    wire [7:0] video_data=cursor[7:0]^{5'd0,cursor[10:8]};
    network_runtime_dual #(.DEFAULT_IP(0),.TELEMETRY_ENABLE(1)) net(
        .clk(ethclk),.rst_n(rst_n),.rx_clk(rxclk),.rx_rst_n(rst_n),.rx_data(rx_data),.rx_valid(rx_valid),.rx_error(1'b0),
        .video_start(video_start),.video_first(1'b1),.video_length(video_length),.video_data(video_data),
        .video_read(video_read),.video_done(video_done),.tx_data(tx),.tx_valid(tx_valid),
        .target_ip(),.target_port(),.cache_hit(),.target_mac(),.enabled(),.arp_requests(),.rx_packets(),.rx_debug(),
        .target2_ip(),.target2_port(),.enabled2(),.cache2_hit(),.target2_mac(),
        .vision_valid(),.vision_payload(),.vision_status(256'd0),.vision_status_valid(1'b0),
        .event_valid(1'b0),.event_payload(256'd0),.event_ready(),
        .telemetry_valid(payload_valid),.telemetry_payload(payload),.telemetry_ready(payload_ready));
    integer log;reg was_tx=0;
    always @(posedge ethclk)if(video_read)cursor<=cursor+1'b1;
    always @(negedge ethclk)begin
        if(tx_valid)$fwrite(log,"%02x\n",tx);else if(was_tx)$fwrite(log,"END\n");
        was_tx=tx_valid;
    end
    reg [511:0] held;reg stalled=0;
    always @(posedge ethclk)begin
        if(!rst_n)stalled<=0;
        else begin
            if(stalled && payload_valid && held!==payload)$fatal(1,"CDC payload changed while stalled");
            stalled<=payload_valid && !payload_ready;held<=payload;
        end
    end
    always @(posedge clk)if(rst_n && unused_tx!==1'b1)$fatal(1,"Ethernet telemetry leaked onto UART");
    task inject(input integer kind);reg [7:0] bytes[0:127];integer n,k;begin
        if(kind==0)begin n=74;$readmemh("build/lidar_eth360_20261102/set_endpoint.hex",bytes,0,73);end
        else begin n=72;$readmemh("build/lidar_eth360_20261102/arp_endpoint.hex",bytes,0,71);end
        @(negedge rxclk);rx_valid=1;
        for(k=0;k<n;k=k+1)begin rx_data=bytes[k];@(negedge rxclk);end
        rx_valid=0;rx_data=0;repeat(250)@(negedge ethclk);
    end endtask
    task point(input integer a,d,q);begin
        @(negedge clk);angle=a;distance=d;intensity=q;pv=1;
        @(negedge clk);pv=0;repeat(100)@(negedge clk);
    end endtask
    task turn(input integer seed);integer n;begin
        for(n=0;n<360;n=n+1)begin
            point(n*100+17,seed+n+1000,10);
            point(n*100+37,seed+n,77);
            point(n*100+86,seed+n,88);
            if(n==0)repeat(400)@(negedge clk);
        end
    end endtask
    integer n;
    initial begin
        log=$fopen("build/lidar_eth360_20261102/tx.hex","w");
        $readmemh("build/lidar_eth360_20261102/motor.hex",motor_memory);
        repeat(5)@(negedge clk);rst_n=1;repeat(400)@(negedge clk);
        motor_valid=1;@(negedge clk);motor_valid=0;
        point(35900,1,1);turn(1000);point(0,2000,77);
        // The network initially has no endpoint. Complete snapshots must remain
        // immutable through CDC and queues until ARP/configuration is ready.
        wait(payload_valid);repeat(2000)@(negedge clk);
        inject(0);inject(1);
        fork
            begin
                wait(net.enabled2 && net.cache2_hit);
                @(negedge ethclk);video_start=1;@(negedge ethclk);video_start=0;
                wait(video_done);if(cursor!=1456)$fatal(1,"video source read count");
            end
            begin
                // Finish a second complete circle while the first is uploading.
                repeat(70000)@(negedge clk);
                for(n=0;n<360;n=n+1)point(n*100+37,2000+n,77);
                point(0,3000,77);
            end
        join
        repeat(20000)@(negedge clk);
        if(source.completed<2 || source.input_loss_count!=0)$fatal(1,"missing/overloaded clean scan");
        $fclose(log);$display("PASS 360-point RAM, measured angles, tie rule, complete CDC, UDP/video concurrency, UART idle");$finish;
    end
    initial begin #30000000;$fatal(1,"Ethernet test timeout");end
endmodule
