`timescale 1ns/1ps
module tb_core;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,command_valid=0;
    reg a=0,b=0,c=0,d=0;
    reg [7:0] opcode=0,seq=0,command_flags=1;
    reg signed [15:0] lt=0,rt=0;
    reg [15:0] kp=256,ki=0;
    wire l1,l2,r1,r2,armed;
    wire [15:0] faults;
    wire [7:0] flags,last_sequence;
    wire signed [31:0] lp,rp,lv,rv;
    wire signed [15:0] la,ra;
    wire [31:0] li,ri,rejected;
    motor_core #(.CLK_HZ(100000),.CONTROL_HZ(1000),.PWM_HZ(10000),.WATCHDOG_MS(20),.FILTER_CYCLES(1),.REVERSE_TICKS(2),.STALL_TICKS(10),.MAX_COUNTS(10)) dut(
        .clk(clk),.reset_n(reset_n),.left_enc_a(a),.left_enc_b(b),.right_enc_a(c),.right_enc_b(d),
        .command_valid(command_valid),.opcode(opcode),.sequence_id(seq),.command_flags(command_flags),.left_target(lt),.right_target(rt),.command_kp(kp),.command_ki(ki),
        .left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2),.armed(armed),.faults(faults),.flags(flags),.last_sequence(last_sequence),
        .left_position(lp),.right_position(rp),.left_velocity(lv),.right_velocity(rv),.left_applied(la),.right_applied(ra),.left_invalid(li),.right_invalid(ri),.rejected_commands(rejected));
    task command(input [7:0] op,input signed [15:0] l,r);begin
        @(negedge clk);opcode=op;lt=l;rt=r;seq=seq+1;command_valid=1;
        @(negedge clk);command_valid=0;repeat(5) @(negedge clk);
    end endtask
    task wait_cycles(input integer n);begin repeat(n) @(negedge clk);end endtask
    task clear_arm;begin command(0,0,0);command(3,0,0);command(1,0,0);if(!armed) $fatal(1,"arm failed %h",faults);end endtask
    always @(posedge clk) begin
        if(!armed && (l1||l2||r1||r2)) $fatal(1,"unarmed output");
    end
    integer i;
    initial begin
        #1;reset_n=0;wait_cycles(3);reset_n=1;wait_cycles(10);
        command(1,0,0);if(armed) $fatal(1,"boot bypassed clear");
        command(3,1,0);if(faults!=1) $fatal(1,"CLEAR accepted a nonzero target");
        clear_arm();command(2,80,-60);wait_cycles(800);
        if(la!=80 || ra!=-60) $fatal(1,"independent signed targets %d %d",la,ra);
        // A duplicate sequence and a changed config cannot refresh watchdog.
        @(negedge clk);command_valid=1;repeat(1) @(negedge clk);command_valid=0;
        if(rejected==0) $fatal(1,"duplicate accepted");
        command_flags=3;command(2,20,20);command_flags=1;
        wait_cycles(1300);if(armed || !faults[1]) $fatal(1,"watchdog");
        command(2,10,10);if(armed) $fatal(1,"fault self rearm");
        clear_arm();command(2,80,80);wait_cycles(600);
        command(0,0,0);if(l1||l2||r1||r2||armed) $fatal(1,"STOP output");
        command_flags=0;command(1,0,0);if(armed) $fatal(1,"legacy open loop accepted");command_flags=1;
        clear_arm();command(2,300,300);wait_cycles(1800);
        if(armed || !faults[4]) $fatal(1,"stall");
        clear_arm();command(2,0,0);
        // Deliberately impossible diagonal AB transitions, >8 in one window.
        @(negedge clk);while(!dut.sample_tick) @(negedge clk);
        for(i=0;i<9;i=i+1) begin {a,b}=~{a,b};wait_cycles(9);end
        wait_cycles(120);if(armed || !faults[2]) $fatal(1,"encoder invalid trip %h",faults);
        clear_arm();command(2,0,0);
        while(!dut.sample_tick) @(negedge clk);
        for(i=0;i<4;i=i+1) begin {a,b}=0;wait_cycles(7);{a,b}=1;wait_cycles(7);{a,b}=3;wait_cycles(7);{a,b}=2;wait_cycles(7);end
        wait_cycles(200);if(armed || !faults[3]) $fatal(1,"overspeed %h",faults);
        clear_arm();command(0,0,0);seq=253;command(1,0,0);command(2,40,40);command(2,60,60);
        if(last_sequence!=0 || dut.request_l!=60) $fatal(1,"sequence wrap");
        command_flags=0;command(2,40,40);if(dut.request_l!=60) $fatal(1,"live config change accepted");
        $display("PASS core: arm guard, dual commands, sequence/wrap, watchdog, STOP and open loop rejection, stall, encoder faults");$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
