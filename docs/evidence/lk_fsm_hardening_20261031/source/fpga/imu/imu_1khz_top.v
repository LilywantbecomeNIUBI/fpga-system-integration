`timescale 1ns/1ps
module imu_1khz_top # (parameter integer I2C_HZ=1000000)(
    input clk50m, input reset_n, inout imu_i2c_scl, inout imu_i2c_sda,
    output uart_tx, output [7:0] led,
    output m1_pwm_ah, m1_pwm_al, m1_pwm_bh, m1_pwm_bl, m1_pwm_ch, m1_pwm_cl,
    output m2_pwm_ah, m2_pwm_al, m2_pwm_bh, m2_pwm_bl, m2_pwm_ch, m2_pwm_cl,
    output m1_drv_en, m2_drv_en, m1_cal, m2_cal, lidar_moto_pwm,
    output adc_convst, adc_cs_n, adc_sclk, adc_reset,
    output enc1_cs_n, enc1_clk, enc2_cs_n, enc2_clk, lidar_uart_tx
);
    wire rst_n;
    reset_sync reset_release(.clk(clk50m),.arst_n(reset_n),.rst_n(rst_n));
    wire ready,fault,valid,stale,temp_valid;
    wire signed [15:0] ax,ay,az,gx,gy,gz,temp;
    wire [31:0] seq,stamp,commit_stamp,age_us,now_us,debug_seq;
    wire [7:0] who,error;
    wire [15:0] errors,invalid,dropped,read_us,interval_us;
    imu_1khz_core #(.I2C_HZ(I2C_HZ)) core(.clk(clk50m),.rst_n(rst_n),
        .imu_i2c_scl(imu_i2c_scl),.imu_i2c_sda(imu_i2c_sda),.ready(ready),.fault(fault),
        .sample_valid(valid),.sample_stale(stale),.temperature_valid(temp_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.timestamp_us(stamp),.commit_timestamp_us(commit_stamp),.sample_age_us(age_us),
        .now_us(now_us),.observe_to_commit_us(read_us),.sample_interval_us(interval_us),
        .error_count(errors),.invalid_count(invalid),.who_am_i(who),.last_error(error));
    imu_fast_uart_stream #(.FAST_BUS(I2C_HZ>400000)) debug_stream(.clk(clk50m),.rst_n(rst_n),
        .sample_valid(valid),.ready(ready),.fault(fault),.sample_stale(stale),.temperature_valid(temp_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.timestamp_us(stamp),.now_us(now_us),.who_am_i(who),.last_error(error),
        .error_count(errors),.invalid_count(invalid),.observe_to_commit_us(read_us),
        .sample_interval_us(interval_us),.tx(uart_tx),.drop_count(dropped),.debug_seq(debug_seq));
    reg [25:0] heartbeat;
    always @(posedge clk50m or negedge rst_n)
        if (!rst_n) heartbeat<=0; else heartbeat<=heartbeat+1'b1;
    assign led={dropped!=0,errors!=0,stale,seq[0],seq!=0,fault,ready,heartbeat[25]};
    assign {m1_pwm_ah,m1_pwm_al,m1_pwm_bh,m1_pwm_bl,m1_pwm_ch,m1_pwm_cl}=6'b0;
    assign {m2_pwm_ah,m2_pwm_al,m2_pwm_bh,m2_pwm_bl,m2_pwm_ch,m2_pwm_cl}=6'b0;
    assign {m1_drv_en,m2_drv_en,m1_cal,m2_cal,lidar_moto_pwm}=5'b0;
    assign adc_convst=0,adc_cs_n=1,adc_sclk=0,adc_reset=0;
    assign enc1_cs_n=1,enc1_clk=0,enc2_cs_n=1,enc2_clk=0,lidar_uart_tx=1;
endmodule
