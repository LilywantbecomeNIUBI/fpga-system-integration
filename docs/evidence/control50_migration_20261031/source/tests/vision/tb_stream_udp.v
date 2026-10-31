`timescale 1ns/1ps
module tb_stream_udp;
    reg wr=0,clk=0,rst=0,fs=0,fe=0,pv=0;
    always #6 wr=~wr;always #4 clk=~clk;
    reg [15:0] pixel=0;wire read,done,start,en;wire [7:0] payload,tx;wire [15:0] length;
    vision_frame_stream #(.WIDTH(16),.ACTIVE_HEIGHT(8),.CANVAS_HEIGHT(16)) stream(
        .wr_clk(wr),.wr_rst_n(rst),.frame_start(fs),.frame_end(fe),.frame_good(1'b1),.frame_id(16'd42),
        .pixel(pixel),.pixel_valid(pv),.rd_clk(clk),.rd_rst_n(rst),.payload_read(read),.tx_done(done),
        .payload_data(payload),.payload_length(length),.packet_frame_id(),.packet_first(),.tx_start(start),.dropped_frames());
    eth_udp_tx_gmii udp(.clk125m(clk),.reset_p(!rst),.tx_en_pulse(start),.tx_done(done),
        .dst_mac(48'h020000000002),.src_mac(48'h020000000001),.dst_ip(32'hc0a83214),.src_ip(32'hc0a8321e),
        .dst_port(16'd6102),.src_port(16'd5000),.data_length(length),.payload_req_o(read),.payload_dat_i(payload),
        .gmii_tx_clk(),.gmii_txen(en),.gmii_txd(tx));
    integer file,k,frames=0;reg previous=0;
    always @(negedge clk)begin
        if(en)$fwrite(file,"%02x",tx);
        if(previous && !en)begin $fwrite(file,"\n");frames=frames+1;end
        previous=en;
    end
    initial begin
        file=$fopen("build/tests/stream_udp.hex","w");
        repeat(4)@(negedge wr);rst=1;fs=1;@(negedge wr);fs=0;
        for(k=0;k<128;k=k+1)begin pv=1;pixel=(k*257)^16'h5a3c;@(negedge wr);end
        pv=0;fe=1;@(negedge wr);fe=0;
        wait(frames==16);@(negedge clk);$fclose(file);
        $display("PASS real frame stream -> registered byte stage -> production UDP, 16 packets captured");$finish;
    end
    initial begin #1000000;$fatal(1,"stream UDP timeout");end
endmodule
