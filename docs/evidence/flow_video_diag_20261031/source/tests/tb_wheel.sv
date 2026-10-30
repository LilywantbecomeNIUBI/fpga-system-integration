`timescale 1ns/1ps
module tb_wheel #(parameter integer PWM_LIMIT=300);
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,tick=0,enable=0,motor_reverse=0;
    reg signed [15:0] target=0;
    reg signed [31:0] velocity=0;
    reg [15:0] kp=256,ki=0;
    wire signed [15:0] applied;
    wire in1,in2;
    wheel_control #(.PWM_LIMIT(PWM_LIMIT),.REVERSE_TICKS(3)) dut(.clk(clk),.reset_n(reset_n),.tick(tick),.enable(enable),.motor_reverse(motor_reverse),.target(target),.velocity(velocity),.kp(kp),.ki(ki),.applied(applied));
    at8236_pwm #(.CLK_HZ(1000000),.PWM_HZ(10000)) pwm(.clk(clk),.reset_n(reset_n),.enable(enable),.command(applied),.in1(in1),.in2(in2));
    task update;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(12) @(negedge clk);end endtask
    integer i,highs;
    initial begin
        #1;reset_n=0;repeat(2) @(negedge clk);reset_n=1;enable=1;target=1000;
        for(i=0;i<20;i=i+1) update();
        if(applied!=PWM_LIMIT) $fatal(1,"limit %d",applied);
        repeat(2000) @(negedge clk);highs=0;
        repeat(100) begin @(negedge clk);if(!in1 && in2) highs=highs+1;if(!in2) $fatal(1,"slow decay hold input");end
        if(highs!=PWM_LIMIT/10) $fatal(1,"PWM duty %d",highs);
        velocity=4;target=-200;repeat(20) update();
        if(applied!=0) $fatal(1,"reversal while moving");
        velocity=0;repeat(3) update();if(applied!=0) $fatal(1,"quiet interval");
        repeat(15) update();if(applied!=-200) $fatal(1,"reverse release");
        enable=0;#1;if(in1||in2) $fatal(1,"disable latency");update();
        enable=1;ki=64;target=100;velocity=0;
        repeat(30) update();if(applied!=PWM_LIMIT) $fatal(1,"PI saturation");
        velocity=200;repeat(50) update();if(applied!=0) $fatal(1,"reverse waits feedback stop");
        velocity=0;target=0;repeat(30) update();if(applied!=0 || dut.integral!=0) $fatal(1,"PI zero reset");
        // Simple deterministic plant: 10 permille per count/window, no inertia.
        enable=0;update();enable=1;target=20;velocity=0;ki=32;
        for(i=0;i<180;i=i+1) begin velocity=$signed(applied)/10;update();end
        if(velocity<18 || velocity>22) $fatal(1,"PI plant convergence %d",velocity);
        $display("PASS wheel: saturation, PWM duty, slew, reversal, disable, PI antiwindup and plant convergence");$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
