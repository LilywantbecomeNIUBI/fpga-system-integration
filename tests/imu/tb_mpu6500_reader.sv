`timescale 1ns/1ps
module tb_mpu6500_reader;
    reg clk=0,rst_n=0,present=1;
    reg [7:0] identity=8'h70;
    always #10 clk=~clk;
    tri1 scl,sda;
    wire ready,fault,valid,stale;
    wire [7:0] who,error;
    wire [31:0] seq;
    wire signed [15:0] ax,ay,az,gx,gy,gz,temp;
    imu_1khz_core #(.POWERUP_US(100),.RESET_US(1200),.STARTUP_US(500),.RETRY_US(2000)) dut(
        .clk(clk),.rst_n(rst_n),.imu_i2c_scl(scl),.imu_i2c_sda(sda),
        .ready(ready),.fault(fault),.sample_valid(valid),.sample_stale(stale),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.who_am_i(who),.last_error(error));
    mpu6050_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(present),
        .identity(identity),.corrupt_readback(1'b0),.invalid_data(1'b0),
        .freeze_samples(1'b0),.reset_stuck(1'b0));
    wire [383:0] frame;wire frame_valid;
    imu_status_source #(.DECIMATION(1)) telemetry(.clk(clk),.reset_n(rst_n),.sample_valid(valid),
        .ready(ready),.fault(fault),.sample_stale(stale),.temperature_valid(1'b1),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.timestamp_us(32'd0),.now_us(32'd0),.who_am_i(who),.last_error(error),
        .error_count(16'd0),.invalid_count(16'd0),.observe_to_commit_us(16'd0),
        .sample_interval_us(16'd1000),.drop_count(16'd0),.frame(frame),.valid(frame_valid));
    integer count=0,packets=0,old_seq;
    always @(posedge clk)begin
        if(valid)begin
            if(who!=8'h70 || !ready || fault || stale)$fatal(1,"MPU6500 sample identity/state");
            if({ax,ay,az,temp,gx,gy,gz}!==112'h0400f80020000154800103e87fff)
                $fatal(1,"MPU6500 burst axis/temp order");
            count=count+1;
        end
        if(frame_valid)begin
            if(frame[239:232]!=6 || frame[223:216]!=8'h70)$fatal(1,"MPU6500 telemetry profile");
            packets=packets+1;
        end
    end
    initial begin
        repeat(10)@(negedge clk);rst_n=1;
        wait(count>=8 && packets>=5);
        if(sensor.regs['h1a]!=2 || sensor.regs['h1d]!=2 || sensor.regs['h37]!=0 ||
           sensor.regs['h19]!=0 || sensor.regs['h1b]!=8'h10 || sensor.regs['h1c]!=8'h08)
            $fatal(1,"profile differs from working STM32 MPU6500 settings");
        present=0;wait(fault);if(error!=1)$fatal(1,"MPU6500 NACK diagnostic");
        old_seq=seq;identity=8'h47;present=1;wait(error==8'h10 && who==8'h47);
        if(seq!=old_seq || ready)$fatal(1,"unsupported identity accepted");
        identity=8'h70;wait(seq>old_seq && ready && !fault);
        $display("PASS MPU6500: identity70, STM32 filter profile, 14-byte burst, config6, NACK/WHO recovery");
        $finish;
    end
    initial begin #60000000;$fatal(1,"MPU6500 regression timeout");end
endmodule
