`timescale 1ns/1ps
module tb_encoder_recovery;
    reg clk=0;always #10 clk=~clk;
    reg reset_n=0,a=0,b=0,tick=0,reverse=0;
    wire signed [31:0] position,velocity;
    wire [31:0] invalid;
    reg [3:0] positive_events[0:212],negative_events[0:109];
    encoder_ab #(.FILTER_CYCLES(8),.RECOVER_DIAGONALS(1),.RECOVERY_MAX_GAP_CYCLES(80)) dut(
        .clk(clk),.reset_n(reset_n),.a(a),.b(b),.sample_tick(tick),.reverse(reverse),
        .position(position),.velocity(velocity),.invalid_count(invalid));
    task restart(input [1:0] ab,input polarity);begin
        @(negedge clk);reset_n=0;{a,b}=ab;reverse=polarity;tick=0;
        repeat(3)@(negedge clk);reset_n=1;repeat(24)@(negedge clk);
        if(position!=0)$fatal(1,"startup baseline");
    end endtask
    task state(input [1:0] ab);begin
        @(negedge clk);#3;{a,b}=ab;repeat(24)@(negedge clk);
    end endtask
    task snapshot;begin tick=1;@(negedge clk);tick=0;end endtask
    integer i;
    initial begin
        $readmemh("tests/fixtures/encoder_positive_20261030.hex",positive_events);
        $readmemh("tests/fixtures/encoder_negative_20261030.hex",negative_events);
        #1;restart(positive_events[0][3:2],0);
        for(i=0;i<213;i=i+1)state(positive_events[i][1:0]);
        if(position!=164 || invalid!=0)$fatal(1,"positive recorded replay %d",position);
        snapshot();if(velocity!=164)$fatal(1,"replay velocity sum");
        restart(negative_events[0][3:2],0);
        for(i=0;i<110;i=i+1)state(negative_events[i][1:0]);
        if(position!=-77 || invalid!=0)$fatal(1,"negative recorded replay %d",position);
        restart(positive_events[0][3:2],1);
        for(i=0;i<213;i=i+1)state(positive_events[i][1:0]);
        if(position!=-164)$fatal(1,"recorded replay polarity");
        restart(0,0);
        state(3);state(0);if(position!=0)$fatal(1,"unknown diagonal invents direction");
        state(1);state(3);state(2);state(0);if(position!=4)$fatal(1,"clean forward scale");
        snapshot();if(velocity!=4)$fatal(1,"clean window");
        state(2);state(3);state(1);state(0);if(position!=0)$fatal(1,"clean reverse scale");
        repeat(100)@(negedge clk);
        state(3);state(0);if(position!=0)$fatal(1,"stale direction used for diagonal");
        $display("PASS recovery candidate: both complete recorded traces, reverse, clean scale, unknown/stale ambiguity ignored");$finish;
    end
    initial begin #2000000;$fatal(1,"timeout");end
endmodule
