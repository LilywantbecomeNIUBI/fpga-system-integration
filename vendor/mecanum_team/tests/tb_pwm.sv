`timescale 1ns/1ps
module tb_pwm;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,enable=0;
    reg signed [15:0] command=0;
    wire in1,in2;
    at8236_pwm #(.CLK_HZ(1000000),.PWM_HZ(10000)) dut(.*);
    task cycles(input integer n);begin repeat(n) @(negedge clk);end endtask
    task duty_check(input integer expected,input integer negative);integer n,drives;begin
        drives=0;
        for(n=0;n<100;n=n+1)begin
            @(negedge clk);
            if(negative)begin
                if(!in1)$fatal(1,"negative hold input");
                if(!in2)drives=drives+1;
            end else begin
                if(!in2)$fatal(1,"positive hold input");
                if(!in1)drives=drives+1;
            end
        end
        if(drives!=expected)$fatal(1,"drive clocks %d expected %d",drives,expected);
    end endtask
    initial begin
        #1;cycles(3);reset_n=1;enable=1;command=200;
        cycles(500);if(!in1||!in2)$fatal(1,"wake must be 11");
        enable=0;#1;if(in1||in2)$fatal(1,"disable during wake");cycles(3);
        enable=1;cycles(1500);duty_check(20,0);
        command=50;cycles(200);duty_check(5,0);
        command=0;#1;if(in1||in2)$fatal(1,"zero coast");cycles(3);
        command=-300;cycles(1500);duty_check(30,1);
        reset_n=0;#1;if(in1||in2)$fatal(1,"reset override");
        $display("PASS PWM: slow decay signed drive duty, 11 wake/brake, zero coast, disable and reset");$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
