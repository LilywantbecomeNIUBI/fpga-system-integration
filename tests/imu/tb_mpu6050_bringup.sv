`timescale 1ns/1ps
module tb_mpu6050_bringup;
    reg clk=0,reset_n=0;
    always #10 clk=~clk;
    tri1 scl,sda;
    wire tx;wire [7:0] led;
    wire [19:0] motor_pads;
    mpu6050_bringup_top dut(.clk50m(clk),.reset_n(reset_n),.imu_i2c_scl(scl),.imu_i2c_sda(sda),
        .uart_tx(tx),.led(led),.left_in1(motor_pads[0]),.left_in2(motor_pads[1]),
        .right_in1(motor_pads[2]),.right_in2(motor_pads[3]),.legacy_m1_disable(motor_pads[4]),.legacy_m2_disable(motor_pads[5]),
        .legacy_m1_pwm_disabled(motor_pads[11:6]),.legacy_m2_pwm_disabled(motor_pads[17:12]),.legacy_cal_disabled(motor_pads[19:18]));
    defparam dut.imu.POWERUP_US=100;
    defparam dut.imu.RESET_US=1200;
    defparam dut.imu.STARTUP_US=500;
    mpu6050_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(1'b1),.identity(8'h68),
        .corrupt_readback(1'b0),.invalid_data(1'b0),.freeze_samples(1'b0),.reset_stuck(1'b0));
    wire [7:0] byte_data;wire byte_valid,framing_error;
    uart_rx_byte monitor(.clk(clk),.reset_n(reset_n),.rx(tx),.data(byte_data),.valid(byte_valid),.framing_error(framing_error));
    integer file,bytes=0;
    always @(posedge clk) begin
        if(motor_pads!==0) $fatal(1,"bringup motor pad not fixed low");
        if(byte_valid)begin $fwrite(file,"%c",byte_data);bytes=bytes+1;end
        if(reset_n && framing_error) $fatal(1,"bringup UART framing");
    end
    initial begin
        file=$fopen("build/tests/mpu6050_bringup_uart.bin","wb");
        repeat(10)@(negedge clk);reset_n=1;
        #150000000;
        if(bytes<6*48 || dut.seq<130 || dut.fault || dut.drops || dut.errors) $fatal(1,"bringup telemetry");
        $fclose(file);$display("PASS MPU6050 bringup: production 50Hz UART, fixed-low 20 motor pads, no command input");$finish;
    end
endmodule
