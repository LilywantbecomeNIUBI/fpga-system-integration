`timescale 1ns/1ps
module tb_motor_imu;
    reg clk=0,reset_n=0;always #10 clk=~clk;
    tri1 scl,sda;wire tx,l1,l2,r1,r2;
    brushed_motor_top #(.CLK_HZ(5000000),.PWM_LIMIT(350),.IMU_ENABLED(1)) dut(
        .clk50m(clk),.reset_n(reset_n),.uart_rx(1'b1),.imu_i2c_scl(scl),.imu_i2c_sda(sda),
        .left_enc_a(1'b0),.left_enc_b(1'b0),.right_enc_a(1'b0),.right_enc_b(1'b0),
        .uart_tx(tx),.left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2));
    defparam dut.with_imu.imu.POWERUP_US=100;
    defparam dut.with_imu.imu.RESET_US=1200;
    defparam dut.with_imu.imu.STARTUP_US=500;
    icm42688_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(1'b1),.identity(8'h47),.corrupt_readback(1'b0),.invalid_data(1'b0),.freeze_samples(1'b0));
    wire [7:0] data;wire valid,error;
    uart_rx_byte #(.CLK_HZ(5000000)) monitor(.clk(clk),.reset_n(reset_n),.rx(tx),.data(data),.valid(valid),.framing_error(error));
    integer file,samples=0;
    always @(posedge clk)begin
        if(valid)$fwrite(file,"%c",data);
        if(dut.with_imu.imu_sample_valid)samples=samples+1;
        if(reset_n && (l1||l2||r1||r2||error))$fatal(1,"STOP outputs or UART framing");
    end
    initial begin
        file=$fopen("build/tests/motor_imu_uart.bin","wb");repeat(10)@(negedge clk);reset_n=1;
        #45000000;
        if(samples<30 || dut.imu_drops!=0)$fatal(1,"IMU samples/drops %d %d",samples,dut.imu_drops);
        $fclose(file);$display("PASS motor IMU: sensor acquisition, concurrent diagnostics/UART, STOP outputs");$finish;
    end
endmodule
