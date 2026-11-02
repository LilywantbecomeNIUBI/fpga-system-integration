`timescale 1ns/1ps
module tb_core_1khz;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,cv=0,a=0,b=0;
    reg [7:0] op=0,seq=0,flags=1;
    reg signed [15:0] target=0;
    wire armed,l1,l2,r1,r2;
    wire [15:0] faults;
    wire signed [31:0] lp,rp,lv,rv;
    wire signed [15:0] la,ra;
    motor_core #(.CLK_HZ(1000000),.WHEEL_1KHZ(1),.FILTER_CYCLES(1)) dut(
        .clk(clk),.reset_n(reset_n),.left_enc_a(a),.left_enc_b(b),.right_enc_a(1'b0),.right_enc_b(1'b0),
        .command_valid(cv),.opcode(op),.sequence_id(seq),.command_flags(flags),
        .left_target(target),.right_target(16'sd0),.command_kp(16'd256),.command_ki(16'd0),
        .left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2),.armed(armed),.faults(faults),
        .left_position(lp),.right_position(rp),.left_velocity(lv),.right_velocity(rv),.left_applied(la),.right_applied(ra));
    task cycles(input integer n);begin repeat(n)@(negedge clk);end endtask
    task command(input [7:0] code,input signed [15:0] value);begin
        op=code;target=value;seq=seq+1;cv=1;cycles(1);cv=0;cycles(5);
    end endtask
    task arm;begin command(0,0);command(3,0);command(1,0);if(!armed)$fatal(1,"arm %h",faults);end endtask
    task drive_ms(input integer n,input signed [15:0] value);integer i;begin
        for(i=0;i<n;i=i+1)begin if(i%50==0)command(2,value);cycles(1000);end
    end endtask
    integer clocks=0,last_control=0,last_feedback=0,controls=0,feedbacks=0;
    always @(posedge clk)begin
        clocks=clocks+1;
        if(reset_n)begin
            if(dut.control_tick)begin
                if(last_control && clocks-last_control!=1000)$fatal(1,"not 1kHz");
                if(dut.fast_wheels.speed_l.dividing || dut.fast_wheels.wheel_l.state!=0)
                    $fatal(1,"previous observer/PI operation missed its deadline");
                controls=controls+1;last_control=clocks;
            end
            if(dut.feedback_control_tick)begin
                if(last_feedback && clocks-last_feedback!=10000)$fatal(1,"telemetry window changed");
                feedbacks=feedbacks+1;last_feedback=clocks;
            end
            if(!armed && (l1||l2||r1||r2))$fatal(1,"unarmed PWM");
            if(ra!=0 || r1 || r2)$fatal(1,"independent stationary wheel moved");
        end
    end
    initial begin
        cycles(3);reset_n=1;cycles(10);
        command(1,0);if(armed)$fatal(1,"boot lock bypass");
        arm();drive_ms(60,80);
        if(!armed || la!=80 || rv!=0)$fatal(1,"legacy target units changed %d",la);
        command(0,0);cycles(2);if(armed || la!=0 || l1 || l2)$fatal(1,"STOP");
        arm();drive_ms(20,40);cycles(251000);
        if(armed || !faults[1])$fatal(1,"250ms watchdog changed %h",faults);
        arm();drive_ms(450,300);
        if(!armed || faults!=0)$fatal(1,"500ms stall shortened %h",faults);
        drive_ms(200,300);
        if(armed || !faults[4] || faults[1])$fatal(1,"stall not independent of 1kHz %h",faults);
        arm();command(2,40);
        // Positive quadrature, each A edge contributes two units. One cycle/ms
        // is 40 position units per 10ms in the unchanged external interface.
        repeat(24)begin
            {a,b}=1;cycles(250);{a,b}=3;cycles(250);{a,b}=2;cycles(250);{a,b}=0;cycles(250);
        end
        if(lv<38 || lv>42 || dut.fast_wheels.lv_q8<9728 || dut.fast_wheels.lv_q8>10752)
            $fatal(1,"feedback units %d / %d",lv,dut.fast_wheels.lv_q8);
        command(0,0);cycles(3);
        if(armed || l1 || l2 || r1 || r2)$fatal(1,"STOP after fresh feedback");
        if(controls<100 || feedbacks<10)$fatal(1,"insufficient scheduling coverage");
        $display("PASS 1kHz core: exact periods, 10ms command/telemetry units, CLEAR/ARM, STOP, watchdog, unchanged 500ms stall, wheel isolation");$finish;
    end
    initial begin #20000000;$fatal(1,"timeout");end
endmodule
