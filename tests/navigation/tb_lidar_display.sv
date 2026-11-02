`timescale 1ns/1ps
module tb_lidar_display;
    reg clk=0,rst_n=0,rx=1,gate=1;
    always #10 clk=~clk;
    wire bv,fe,pv;wire [7:0] b,q;wire [15:0] d,a;
    wire [31:0] packets,errors,ce,fmt,to;
    stl27l_uart_rx receiver(.clk(clk),.rst_n(rst_n),.rx(rx),.byte_valid(bv),.byte_data(b),.framing_error(fe));
    stl27l_node_parser parser(.clk(clk),.rst_n(rst_n),.byte_valid(bv),.byte_in(b),.stream_error(fe),
        .point_valid(pv),.distance_mm(d),.intensity(q),.angle_q100(a),.packet_count(packets),.error_count(errors),
        .crc_error_count(ce),.format_error_count(fmt),.timeout_count(to));
    wire [383:0] scan,motor;wire sv,mv,ready,tx;
    lidar_scan_source display(.clk(clk),.rst_n(rst_n),.byte_valid(bv),.framing_error(fe),.point_valid(pv),
        .angle(a),.distance(d),.intensity(q),.packets(packets),.errors(errors),.crc_errors(ce),.format_errors(fmt),
        .timeouts(to),.ready(ready && gate),.frame(scan),.valid(sv));
    motor_status_source status(.clk(clk),.reset_n(rst_n),.armed(1'b0),.flags(8'd0),.last_sequence(8'd0),.faults(16'd1),
        .left_position(32'd0),.right_position(32'd0),.left_velocity(32'd0),.right_velocity(32'd0),
        .left_applied(16'd0),.right_applied(16'd0),.left_invalid(32'd0),.right_invalid(32'd0),
        .bad_frames(32'd0),.rejected_commands(32'd0),.frame(motor),.valid(mv));
    frame_uart_mux #(.AUX_ENABLED(1)) mux(.clk(clk),.reset_n(rst_n),.motor_valid(mv),.motor_frame(motor),
        .imu_valid(1'b0),.imu_frame(384'd0),.aux_valid(sv && gate && ready),.aux_frame(scan),.aux_ready(ready),.tx(tx));
    wire [7:0] received;wire rv,rfe;
    uart_rx_byte monitor(.clk(clk),.reset_n(rst_n),.rx(tx),.data(received),.valid(rv),.framing_error(rfe));
    integer file,i;reg [7:0] data[0:5122];
    reg stalled=0;reg [383:0] held;
    always @(posedge clk)begin
        if(rv)$fwrite(file,"%c",received);
        if(rfe)$fatal(1,"mixed UART framing error");
        if(sv && !gate)begin
            if(stalled && scan!==held)$fatal(1,"display changed while consumer stalled");
            held<=scan;stalled<=1;
        end else stalled<=0;
    end
    task send(input [7:0] value);integer k;begin
        rx=0;#1085.069;
        for(k=0;k<8;k=k+1)begin rx=value[k];#1085.069;end
        rx=1;#1085.069;
    end endtask
    initial begin
        file=$fopen("build/task4/lidar_display_uart.bin","wb");
        $readmemh("build/task4/lidar_display_input.hex",data);
        repeat(5)@(negedge clk);rst_n=1;
        // One bad CRC followed by three complete turns and the next wrap.
        for(i=0;i<5123;i=i+1)send(data[i]);
        #200000000;
        if(packets!=108 || ce!=1 || errors!=1)$fatal(1,"receiver diagnostics %d %d %d",packets,ce,errors);
        $fclose(file);$display("PASS raw921600 -> parser -> complete scan -> stalled sender -> mixed115200 UART");$finish;
    end
    initial begin
        #36000000;gate=0;#15000000;gate=1;
    end
    initial begin #350000000;$fatal(1,"display timeout");end
endmodule
