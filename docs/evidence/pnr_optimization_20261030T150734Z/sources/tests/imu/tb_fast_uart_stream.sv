`timescale 1ns/1ps
module tb_fast_uart_stream;
    reg clk=0,rst_n=0,valid=0,ready=1,fault=0;
    always #10 clk=~clk;
    reg signed [15:0] ax=1024,ay=-2048,az=8192,gx=-32767,gy=1000,gz=32767,temp=132;
    reg [31:0] seq=1,stamp=5000,now_us=0;
    wire tx;
    wire [15:0] drops;
    imu_fast_uart_stream #(.STATUS_US(5000),.DECIMATION(1)) dut(.clk(clk),.rst_n(rst_n),.sample_valid(valid),
        .ready(ready),.fault(fault),.ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),
        .temperature_raw(temp),.sample_seq(seq),.timestamp_us(stamp),.now_us(now_us),
        .who_am_i(8'h47),.last_error(8'h00),.error_count(16'd0),.invalid_count(16'd0),
        .sample_stale(~ready),.temperature_valid(1'b1),.observe_to_commit_us(16'd145),.sample_interval_us(16'd1000),.tx(tx),.drop_count(drops),.debug_seq());
    reg [7:0] captured[0:143];
    reg [7:0] byte_in;
    reg [15:0] crc;
    integer received=0,i,j,f;
    function [15:0] update_crc;
        input [15:0] c0;
        input [7:0] b;
        integer k;
        reg [15:0] c;
        begin c=c0^{b,8'b0}; for(k=0;k<8;k=k+1) c=c[15] ? (c<<1)^16'h1021 : c<<1; update_crc=c; end
    endfunction
    task sample;
        begin @(negedge clk); valid=1; @(negedge clk); valid=0; end
    endtask
    initial f=$fopen("results/fast_uart_frames.bin","wb");
    always begin
        @(negedge tx); repeat(217) @(posedge clk); #1;
        if(tx!==0) $fatal(1,"UART start bit invalid");
        for(i=0;i<8;i=i+1) begin repeat(434) @(posedge clk); #1; byte_in[i]=tx; end
        repeat(434) @(posedge clk); #1;
        if(tx!==1) $fatal(1,"UART stop bit invalid");
        captured[received]=byte_in; $fwrite(f,"%c",byte_in); received=received+1;
        if(received%48==0) begin
            crc=16'hffff;
            for(j=received-48;j<received-2;j=j+1) crc=update_crc(crc,captured[j]);
            if({captured[received-2],captured[received-1]}!==crc) $fatal(1,"UART CRC mismatch");
        end
        if(received==144) begin
            if(captured[0]!==8'ha5 || captured[1]!==8'h5a || captured[3]!==1 ||
               captured[4]!==1 || captured[12]!==0 || captured[13]!==4 ||
               captured[14]!==0 || captured[15]!==8'hf8 || captured[27]!==8'h47)
                $fatal(1,"snapshot/endianness mismatch");
            if(captured[52]!==4 || captured[82]!==2 || captured[99]!==2 || drops!==2)
                $fatal(1,"drop accounting or status record mismatch");
            $fclose(f);
            $display("PASS fast UART: actual 50MHz/115200 framing, signed snapshot, CRC, backpressure, status heartbeat");
            $finish;
        end
    end
    initial begin
        repeat(10) @(negedge clk); rst_n=1; sample();
        // Changing source registers while transmitting must not tear the frame.
        seq=2; ax=77; stamp=10000; repeat(100) @(negedge clk); sample();
        seq=3; repeat(100) @(negedge clk); sample();
        wait(received==48); wait(dut.state==0);
        seq=4; ax=1024; stamp=20000; sample();
        wait(received==96); ready=0; fault=1; now_us=25000;
    end
    initial begin #20000000; $fatal(1,"UART test timeout %0d bytes",received); end
endmodule
