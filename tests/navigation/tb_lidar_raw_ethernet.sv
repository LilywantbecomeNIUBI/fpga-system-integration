`timescale 1ns/1ps
// Real scan RAM -> frame mux -> asynchronous mailbox -> production UDP MAC.
module tb_lidar_raw_ethernet;
    reg clk=0,ethclk=0,rxclk=0,rst_n=0,pv=0;
    always #10 clk=~clk;always #4 ethclk=~ethclk;always #4.1 rxclk=~rxclk;
    wire [15:0] angle,distance;wire [7:0] intensity;wire [3:0] point_index;
    reg native_valid=0;reg [7:0] native_byte=0;wire native_pv;
    wire [31:0] native_packets,native_errors,native_crc,native_format,native_timeouts;
    wire [383:0] scan,selected;wire scan_valid,scan_ready,selected_valid,selected_ready,unused_tx;
    stl27l_node_parser #(.BYTE_TIMEOUT_CYCLES(100000)) parser(.clk(clk),.rst_n(rst_n),
        .byte_valid(native_valid),.byte_in(native_byte),.stream_error(1'b0),
        .point_valid(native_pv),.point_index(point_index),.angle_q100(angle),.distance_mm(distance),.intensity(intensity),
        .packet_count(native_packets),.error_count(native_errors),.crc_error_count(native_crc),
        .format_error_count(native_format),.timeout_count(native_timeouts));
    lidar_points_source #(.CLK_HZ(50000000)) source(.clk(clk),.rst_n(rst_n),
        .byte_valid(native_valid),.framing_error(1'b0),.point_valid(native_pv),.point_index(point_index),
        .angle(angle),.distance(distance),.intensity(intensity),.packets(native_packets),.errors(native_errors),
        .crc_errors(native_crc),.format_errors(native_format),.timeouts(native_timeouts),
        .frame(scan),.valid(scan_valid),.ready(scan_ready));
    reg motor_valid=0;reg [383:0] motor_memory[0:0];
    frame_uart_mux #(.CLK_HZ(50000000),.AUX_ENABLED(1),.ETHERNET(1)) mux(
        .clk(clk),.reset_n(rst_n),.motor_valid(motor_valid),.motor_frame(motor_memory[0]),
        .imu_valid(1'b0),.imu_frame(384'd0),.aux_valid(scan_valid && scan_ready),.aux_frame(scan),
        .aux_ready(scan_ready),.aux_drops(),.imu_drops(),.tx(unused_tx),
        .ethernet_frame(selected),.ethernet_valid(selected_valid),.ethernet_ready(selected_ready));
    wire [511:0] payload;wire payload_valid,payload_ready;
    telemetry_cdc #(.CLK_HZ(50000000)) mailbox(.src_clk(clk),.src_rst_n(rst_n),
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
        if(kind==0)begin n=74;$readmemh("build/lidar_raw_ethernet_20261102/set_endpoint.hex",bytes,0,73);end
        else begin n=72;$readmemh("build/lidar_raw_ethernet_20261102/arp_endpoint.hex",bytes,0,71);end
        @(negedge rxclk);rx_valid=1;
        for(k=0;k<n;k=k+1)begin rx_data=bytes[k];@(negedge rxclk);end
        rx_valid=0;rx_data=0;repeat(250)@(negedge ethclk);
    end endtask
    reg [7:0] bytes[0:65535];integer input_size,fd,ignored,k;
    initial begin
        log=$fopen("build/lidar_raw_ethernet_20261102/tx.hex","w");
        $readmemh("build/lidar_raw_ethernet_20261102/motor.hex",motor_memory);
        fd=$fopen("build/lidar_raw_ethernet_20261102/input.len","r");ignored=$fscanf(fd,"%d",input_size);$fclose(fd);
        $readmemh("build/lidar_raw_ethernet_20261102/input.hex",bytes,0,input_size-1);
        repeat(5)@(negedge clk);rst_n=1;repeat(400)@(negedge clk);
        inject(0);inject(1);wait(net.enabled2 && net.cache2_hit);
        motor_valid=1;@(negedge clk);motor_valid=0;
        fork
            begin
                wait(native_packets>10);
                @(negedge ethclk);video_start=1;@(negedge ethclk);video_start=0;
                wait(video_done);if(cursor!=1456)$fatal(1,"video source read count");
            end
            begin
                for(k=0;k<input_size;k=k+1)begin
                    @(negedge clk);native_byte=bytes[k];native_valid=1;
                    @(negedge clk);native_valid=0;repeat(8)@(negedge clk);
                end
            end
        join
        repeat(10000)@(negedge clk);
        if(native_packets!=1000 || native_crc!=1 || source.forwarded!=1000 || source.dropped!=0)
            $fatal(1,"raw counters packets=%0d crc=%0d forwarded=%0d dropped=%0d",native_packets,native_crc,source.forwarded,source.dropped);
        $fclose(log);$display("PASS original STL packets -> production parser -> all-point RAM FIFO -> mixed CDC -> UDP with video, UART idle");$finish;
    end
    initial begin #30000000;$fatal(1,"raw Ethernet test timeout");end
endmodule
