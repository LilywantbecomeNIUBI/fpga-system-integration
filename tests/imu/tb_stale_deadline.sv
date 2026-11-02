`timescale 1ns/1ps
module stale_deadline_case #(parameter WARN_US=7)(output reg finished=0);
    reg clk=0,rst=0;always #5 clk=~clk;
    tri1 scl,sda;wire stale;wire [31:0] now_us,stamp,age;
    mpu6050_reader #(.CLK_HZ(1000000),.I2C_HZ(100000),.STALE_WARN_US(WARN_US)) dut(
        .clk(clk),.rst_n(rst),.imu_i2c_scl(scl),.imu_i2c_sda(sda),
        .sample_stale(stale),.now_us(now_us),.sample_age_us(age),.commit_timestamp_us(stamp));
    integer k;reg expected;
    initial begin
        repeat(3)@(negedge clk);rst=1;
        // Inject one successful commit close to microsecond wrap. Subsequent
        // bus transactions are held off; the real timestamp/watchdog logic runs.
        @(negedge clk);dut.us_counter=32'hfffffffc;dut.us_divider=0;
        force dut.ready=1;force dut.state=13;force dut.done=1;
        force dut.bus_error=0;force dut.result=0;
        @(posedge clk);#1;
        expected=age>=WARN_US;
        if(stale!==expected)$fatal(1,"commit-on-us-tick threshold=%0d age=%0d",WARN_US,age);
        @(negedge clk);force dut.done=0;
        for(k=0;k<25;k=k+1)begin
            @(posedge clk);#1;expected=age>=WARN_US;
            if(stale!==expected)$fatal(1,"deadline/wrap threshold=%0d age=%0d got=%b",WARN_US,age,stale);
        end
        // Refresh after expiry while the timer is still running.
        @(negedge clk);force dut.done=1;
        @(posedge clk);#1;expected=age>=WARN_US;
        if(stale!==expected)$fatal(1,"refresh after expiry");
        @(negedge clk);force dut.done=0;
        for(k=0;k<10;k=k+1)begin
            @(posedge clk);#1;expected=age>=WARN_US;
            if(stale!==expected)$fatal(1,"refreshed deadline");
        end
        finished=1;
    end
endmodule
module tb_stale_deadline;
    wire a,b,c;
    stale_deadline_case #(.WARN_US(0)) zero(a);
    stale_deadline_case #(.WARN_US(1)) one(b);
    stale_deadline_case #(.WARN_US(7)) seven(c);
    initial begin wait(a && b && c);$display("PASS IMU stale: exact deadline, threshold 0/1/7, microsecond wrap, refresh, commit on tick");$finish;end
    initial begin #10000;$fatal(1,"stale deadline timeout");end
endmodule
