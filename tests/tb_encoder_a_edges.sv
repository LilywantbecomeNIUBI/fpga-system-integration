`timescale 1ns/1ps
module tb_encoder_a_edges;
    reg clk=0; always #10 clk=~clk;
    reg reset_n=0,a=0,b=0,tick=0,reverse=0;
    wire signed [31:0] position,velocity;
    wire [31:0] invalid;
    wire [287:0] diagnostics;wire [3:0] levels;
    wire signed [31:0] b_position,b_velocity;wire [31:0] b_invalid;
    wire [255:0] transition_trace;wire [31:0] transition_count;
    encoder_ab #(.FILTER_CYCLES(8)) dut(
        .clk(clk),.reset_n(reset_n),.a(a),.b(b),.sample_tick(tick),
        .reverse(reverse),.position(position),.velocity(velocity),.invalid_count(invalid),
        .diagnostics(diagnostics),.diagnostic_levels(levels),
        .transition_trace(transition_trace),.transition_count(transition_count));
    encoder_ab #(.FILTER_CYCLES(8),.COUNT_B_EDGES(1)) b_candidate(
        .clk(clk),.reset_n(reset_n),.a(a),.b(b),.sample_tick(tick),
        .reverse(reverse),.position(b_position),.velocity(b_velocity),.invalid_count(b_invalid));
    task state(input [1:0] value);begin
        @(negedge clk); #3; {a,b}=value; repeat(24) @(negedge clk);
    end endtask
    task restart(input [1:0] value,input polarity);begin
        @(negedge clk);reset_n=0;{a,b}=value;reverse=polarity;tick=0;
        repeat(3)@(negedge clk);reset_n=1;repeat(30)@(negedge clk);
        if(position!=0 || invalid!=0 || b_position!=0 || b_invalid!=0)$fatal(1,"startup baseline %b",value);
    end endtask
    task snapshot;begin tick=1;@(negedge clk);tick=0;end endtask
    function [1:0] phase(input integer index);
        case(index&3) 0:phase=0;1:phase=1;2:phase=3;3:phase=2;endcase
    endfunction
    integer start,polarity,direction,i,index,expected;
    initial begin
        #1;
        // Ideal quadrature must retain original x4 sign and full-cycle scale
        // from every starting phase, with both runtime polarity choices.
        for(start=0;start<4;start=start+1)
          for(polarity=0;polarity<2;polarity=polarity+1)
            for(direction=-1;direction<=1;direction=direction+2)begin
                restart(phase(start),polarity!=0);index=start;
                for(i=0;i<32;i=i+1)begin index=index+direction;state(phase(index));end
                expected=direction*(polarity ? -32 : 32);
                if(position!=expected || invalid!=0)$fatal(1,"direction/scale %d %d %d: %d",start,polarity,direction,position);
                if(b_position!=expected || b_invalid!=0)$fatal(1,"B candidate direction/scale %d %d %d: %d",start,polarity,direction,b_position);
                if(transition_count!=32 || transition_trace[3:0]!={phase(index-direction),phase(index)})
                    $fatal(1,"ordered transition trace");
                if(diagnostics[0+:32]!=16 || diagnostics[32+:32]!=16 ||
                   diagnostics[64+:32]!=16 || diagnostics[96+:32]!=16 || diagnostics[256+:32]!=0)
                    $fatal(1,"edge counters for ideal quadrature");
                if(direction==1 && (diagnostics[128+:32]!=16 || diagnostics[160+:32]!=0 ||
                   diagnostics[192+:32]!=16 || diagnostics[224+:32]!=0)) $fatal(1,"positive phase counters");
                if(direction==-1 && (diagnostics[128+:32]!=0 || diagnostics[160+:32]!=16 ||
                   diagnostics[192+:32]!=0 || diagnostics[224+:32]!=16)) $fatal(1,"negative phase counters");
                snapshot();if(velocity!=expected)$fatal(1,"window total");
                if(b_velocity!=expected)$fatal(1,"B candidate window total");
                snapshot();if(velocity!=0)$fatal(1,"window reset");
            end
        restart(0,0);
        // Repeated stable B-only distortion never counts an A edge.
        for(i=0;i<10;i=i+1)begin state(1);state(0);end
        if(position!=0)$fatal(1,"B-only edges counted");
        if(b_position!=0)$fatal(1,"B candidate B bounce cancellation");
        if(diagnostics[64+:32]!=0 || diagnostics[96+:32]!=20 ||
           diagnostics[192+:32]!=10 || diagnostics[224+:32]!=10) $fatal(1,"B-only diagnostic cancellation");
        // Short A/B input glitches do not pass the original stability filter.
        @(negedge clk);a=1;b=1;repeat(2)@(negedge clk);a=0;b=0;
        repeat(30)@(negedge clk);if(position!=0)$fatal(1,"short glitch counted");
        if(diagnostics[0+:32]!=2 || diagnostics[32+:32]!=22 || diagnostics[64+:32]!=0)
            $fatal(1,"raw versus filtered glitch diagnostics");
        // Late B transition coincident with A rising: sampled phase is equal.
        state(3);if(position!=2 || invalid!=0)$fatal(1,"coincident rising A");
        state(2);if(position!=2)$fatal(1,"B-only falling edge");
        state(0);if(position!=4)$fatal(1,"falling A scaling");
        // Known limitation: arbitrary correlated AB toggles can invent motion.
        // Do not mistake permanently zero invalid for a quality measurement.
        state(3);state(0);if(position!=8 || invalid!=0)$fatal(1,"documented diagonal semantics");
        if(diagnostics[256+:32]!=3) $fatal(1,"passive diagonal diagnostic");
        $display("PASS A-edge decoder: all phases/polarities/directions, scaled windows, B distortion, glitches, diagonal limitation");$finish;
    end
    initial begin #2000000;$fatal(1,"timeout");end
endmodule
