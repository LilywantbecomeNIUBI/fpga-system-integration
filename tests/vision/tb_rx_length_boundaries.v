`timescale 1ns/1ps
module tb_rx_length_boundaries;
    reg clk=0,rst=0,valid=0;reg [7:0] data=0;always #4 clk=~clk;
    network_runtime_dual #(.CLK_HZ(1000),.DEFAULT_IP(0),.VISION_ENABLE(1),.TELEMETRY_ENABLE(1)) dut(
        .clk(clk),.rst_n(rst),.rx_clk(clk),.rx_rst_n(rst),.rx_data(data),.rx_valid(valid),.rx_error(1'b0),
        .video_start(1'b0),.video_first(1'b0),.video_length(16'd0),.video_data(8'd0),
        .vision_status(256'd0),.vision_status_valid(1'b0),.event_valid(1'b0),.event_payload(256'd0),
        .telemetry_valid(1'b0),.telemetry_payload(512'd0));
    function [31:0] crc_byte(input [31:0] crc,input [7:0] value);
        integer j;reg [31:0] c;
        begin c=crc^value;for(j=0;j<8;j=j+1)c=c[0] ? (c>>1)^32'hedb88320 : c>>1;crc_byte=c;end
    endfunction
    integer accepted=0,last_length=0;
    task frame(input integer length,input corrupt);
        integer k;reg [31:0] crc,fcs;
        begin
            @(negedge clk);valid=1;data=8'h55;
            repeat(7)@(negedge clk);data=8'hd5;@(negedge clk);
            crc=32'hffffffff;
            for(k=0;k<length-4;k=k+1)begin data=0;crc=crc_byte(crc,0);@(negedge clk);end
            fcs=~crc;
            for(k=0;k<4;k=k+1)begin data=(fcs>>(k*8)) ^ (corrupt && k==0 ? 8'h01 : 8'h00);@(negedge clk);end
            valid=0;repeat(100)@(negedge clk);
            if(length>=64 && length<=128 && !corrupt)begin accepted=accepted+1;last_length=length;end
            if(dut.rx_good!=accepted || dut.frame_length!=last_length || dut.rx_count!=(length<255 ? length : 255))
                $fatal(1,"RX range/saturation length=%0d good=%0d frame_length=%0d count=%0d",length,dut.rx_good,dut.frame_length,dut.rx_count);
            if(dut.request_toggle!=dut.ack_sync1)$fatal(1,"mailbox did not drain");
        end
    endtask
    initial begin
        repeat(4)@(negedge clk);rst=1;repeat(8)@(negedge clk);
        frame(63,0);frame(64,0);frame(65,0);frame(127,0);frame(128,0);frame(129,0);
        frame(254,0);frame(255,0);frame(256,0);frame(300,0);frame(64,1);frame(64,0);
        $display("PASS RX 63/64/65/127/128/129 and 254/255/256/300-byte boundaries, saturation, bad FCS, recovery");$finish;
    end
endmodule
