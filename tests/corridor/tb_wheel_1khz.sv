`timescale 1ns/1ps
module tb_wheel_1khz #(parameter integer PWM_LIMIT=300);
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,enable=0,tick=0,legacy_tick=0,stopped=1;
    reg signed [15:0] target=0;
    reg signed [31:0] velocity=0;
    reg [15:0] kp=256,ki=0;
    wire signed [15:0] applied,legacy_applied;
    wheel_control_1khz #(.PWM_LIMIT(PWM_LIMIT)) dut(.clk(clk),.reset_n(reset_n),.tick(tick),.enable(enable),.motor_reverse(1'b0),
        .feedback_stopped(stopped),.target(target),.velocity_q8(velocity),.kp(kp),.ki(ki),.applied(applied));
    wheel_control #(.PWM_LIMIT(PWM_LIMIT)) legacy(.clk(clk),.reset_n(reset_n),.tick(legacy_tick),.enable(enable),.motor_reverse(1'b0),
        .target(target),.velocity(velocity>>>8),.kp(kp),.ki(ki),.applied(legacy_applied));
    task update(input integer legacy_now);begin
        @(negedge clk);tick=1;legacy_tick=legacy_now;
        @(negedge clk);tick=0;legacy_tick=0;repeat(45)@(negedge clk);
    end endtask
    task restart;begin enable=0;update(0);enable=1;end endtask
    integer i,old;
    initial begin
        repeat(3)@(negedge clk);reset_n=1;restart();target=1000;
        for(i=1;i<=PWM_LIMIT/2;i=i+1)begin
            old=applied;update(i%10==0);
            if(applied-old!=2)$fatal(1,"1ms slew %d %d",old,applied);
            if(i%10==0 && applied!=legacy_applied)$fatal(1,"changed physical slew");
        end
        if(applied!=PWM_LIMIT)$fatal(1,"limit");
        target=-200;velocity=4*256;stopped=0;
        repeat(250)update(0);
        if(applied!=0)$fatal(1,"reversed while moving");
        velocity=0;stopped=1;
        repeat(100)update(0);if(applied!=0)$fatal(1,"100ms reversal wait shortened");
        repeat(101)update(0);if(applied!=-200)$fatal(1,"reverse release %d",applied);
        restart();target=1;velocity=0;kp=0;ki=1;
        for(i=1;i<=100;i=i+1)update(i%10==0);
        if(dut.integral!=legacy.integral*2560 || dut.integral!=25600)
            $fatal(1,"small Ki lost / physical integration changed %d %d",dut.integral,legacy.integral);
        restart();target=-1;
        for(i=1;i<=100;i=i+1)update(i%10==0);
        if(dut.integral!=-25600 || dut.integral!=legacy.integral*2560)$fatal(1,"negative integral precision");
        restart();target=1000;kp=4096;ki=1024;
        repeat(200)update(0);
        if(applied!=PWM_LIMIT || dut.integral!=0)$fatal(1,"saturated PI windup");
        target=0;repeat(200)update(0);
        if(applied!=0 || dut.integral!=0)$fatal(1,"zero target reset");
        // Fractional feedback must affect P, not be rounded to zero before PI.
        restart();kp=2560;ki=0;target=1;velocity=128;
        repeat(4)update(0);if(applied!=5)$fatal(1,"fractional feedback lost %d",applied);
        enable=0;@(negedge clk);if(applied!=0)$fatal(1,"disable did not abort pipeline");
        restart();target=20;velocity=0;kp=256;ki=32;
        // Independent ideal wheel load: ten PWM permille per unit/10ms.
        for(i=0;i<5000;i=i+1)begin velocity=$signed(applied)*256/10;update(0);end
        if(velocity<19*256 || velocity>21*256)$fatal(1,"ideal plant convergence %d",velocity);
        $display("PASS 1kHz PI: equal-time legacy slew/integration, small +/-Ki, fractional feedback, antiwindup, 100ms reverse wait, zero/disable, ideal plant convergence");$finish;
    end
    initial begin #6000000;$fatal(1,"timeout");end
endmodule
