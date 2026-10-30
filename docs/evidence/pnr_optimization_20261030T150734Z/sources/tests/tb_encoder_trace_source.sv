`timescale 1ns/1ps
module tb_encoder_trace_source;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0;
    wire [383:0] frame;wire valid;
    integer traces=0,counters=0,left_frames=0,right_frames=0;
    encoder_diagnostic_source #(.CLK_HZ(10000),.ENCODER_B_MASK(1)) dut(
        .clk(clk),.reset_n(reset_n),.left_counters(288'd123),.right_counters(288'd456),
        .left_levels(4'h9),.right_levels(4'h6),
        .left_trace(256'h17e),.right_trace(256'h2bd),
        .left_trace_count(32'd3),.right_trace_count(32'd3),.frame(frame),.valid(valid));
    always @(negedge clk) if(valid) begin
        if(frame[0+:24]!=24'h033cc3 && frame[0+:24]!=24'h023cc3)$fatal(1,"frame identity");
        if(frame[16+:8]==3)begin
            traces=traces+1;
            if(frame[24+:8]==1)begin
                left_frames=left_frames+1;
                if(frame[48+:8]!=8'hb2 || frame[56+:8]!=8'h09 || frame[64+:256]!=256'h17e || frame[320+:32]!=3)
                    $fatal(1,"left trace packing");
            end else if(frame[24+:8]==2)begin
                right_frames=right_frames+1;
                if(frame[48+:8]!=8'ha2 || frame[56+:8]!=8'h06 || frame[64+:256]!=256'h2bd || frame[320+:32]!=3)
                    $fatal(1,"right trace packing");
            end else $fatal(1,"trace axis");
        end else counters=counters+1;
    end
    initial begin
        #1;repeat(3)@(negedge clk);reset_n=1;repeat(2600)@(negedge clk);
        if(traces<20 || counters<2 || left_frames<9 || right_frames<9)$fatal(1,"frame cadence");
        $display("PASS ordered trace source: byte packing, both axes, mode identity, statistics coexistence");$finish;
    end
endmodule
