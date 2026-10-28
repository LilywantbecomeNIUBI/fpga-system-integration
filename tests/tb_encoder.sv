`timescale 1ns/1ps
module tb_encoder;
    reg clk=0; always #5 clk=~clk;
    reg reset_n=0,a=0,b=0,tick=0,reverse=0;
    wire signed [31:0] position,velocity;
    wire [31:0] invalid;
    encoder_ab #(.FILTER_CYCLES(3)) dut(.clk(clk),.reset_n(reset_n),.a(a),.b(b),.sample_tick(tick),.reverse(reverse),.position(position),.velocity(velocity),.invalid_count(invalid));
    task state(input [1:0] value); begin @(negedge clk);{a,b}=value;repeat(15) @(negedge clk);end endtask
    task sample; begin tick=1;@(negedge clk);tick=0;end endtask
    initial begin
        #1; reset_n=0; repeat(3) @(negedge clk);reset_n=1;repeat(15) @(negedge clk);
        state(1);state(3);state(2);state(0);
        if(position!=4) $fatal(1,"x4 forward: %d",position);
        sample(); if(velocity!=4) $fatal(1,"velocity snapshot");
        state(2);state(3);state(1);state(0);
        if(position!=0) $fatal(1,"reverse signed count");
        sample();if(velocity!=-4) $fatal(1,"negative window");
        b=1;@(negedge clk);b=0;repeat(20) @(negedge clk);
        if(position!=0) $fatal(1,"glitch counted");
        state(3); if(invalid!=1 || position!=0) $fatal(1,"diagonal transition");
        reverse=1;state(1);if(position!=1) $fatal(1,"encoder polarity");
        // A nonzero AB state at reset release must establish a baseline, not a step.
        reset_n=0;reverse=0;{a,b}=1;repeat(3) @(negedge clk);reset_n=1;repeat(20) @(negedge clk);
        if(position!=0 || invalid!=0) $fatal(1,"nonzero startup baseline");
        $display("PASS encoder: x4 directions, window boundaries, filter, invalid, polarity");$finish;
    end
    initial begin #100000;$fatal(1,"timeout");end
endmodule
