`timescale 1ns/1ps
module tb_encoder_research;
    reg clk=0;always #10 clk=~clk; // Production 50 MHz.
    reg reset_n=0,a=0,b=0;
    wire signed [31:0] position,velocity;
    wire [31:0] invalid_count;
    encoder_ab #(.FILTER_CYCLES(8)) dut(.clk(clk),.reset_n(reset_n),.a(a),.b(b),
        .sample_tick(1'b0),.reverse(1'b0),.position(position),.velocity(velocity),.invalid_count(invalid_count));
    task state(input [1:0] value);begin
        // Non-clock-aligned input and variable dwell model independent encoder edges.
        @(negedge clk);#3;{a,b}=value;repeat(20+$urandom_range(0,30))@(negedge clk);
    end endtask
    task restart;begin
        reset_n=0;{a,b}=0;repeat(3)@(negedge clk);reset_n=1;repeat(20)@(negedge clk);
    end endtask
    integer i;
    initial begin
        #1;restart();
        for(i=0;i<200;i=i+1)begin state(1);state(3);state(2);state(0);end
        if(position!=800||invalid_count!=0)$fatal(1,"clean forward %d %d",position,invalid_count);
        for(i=0;i<200;i=i+1)begin state(2);state(3);state(1);state(0);end
        if(position!=0||invalid_count!=0)$fatal(1,"clean reverse");
        $display("PASS clean async quadrature: +800 then -800, no false invalid transitions");
        // A correlated two-input disturbance returns to the same physical state.
        state(3);state(0);
        if(position!=0||invalid_count!=2)$fatal(1,"zero-net disturbance");
        $display("PASS zero-net diagonal disturbance: position remains 0, invalid=2");
        restart();
        // An omitted intermediate state in forward 00->01->11->10->00.
        state(3);state(2);state(0);
        if(position!=2||invalid_count!=1)$fatal(1,"skipped state");
        $display("PASS skipped-state example: expected physical +4, observed +2, invalid=1");
        restart();
        state(1);state(0);
        if(position!=0||invalid_count!=0)$fatal(1,"single phase bounce");
        $display("PASS one-phase bounce: net position 0 and invalid=0; invalid does not detect all disturbances");
        $finish;
    end
    initial begin #10000000;$fatal(1,"timeout");end
endmodule
