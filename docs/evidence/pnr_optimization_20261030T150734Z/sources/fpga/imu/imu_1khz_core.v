`timescale 1ns/1ps
// Teammate integration point. All outputs are synchronous to clk.
// sample_valid is ONE clock, all six axes and metadata commit together.
// There is no UART, UI or downstream backpressure in this path.
module imu_1khz_core #(
    parameter integer CLK_HZ=50000000, I2C_HZ=1000000,
    parameter integer POLL_US=150, STALE_WARN_US=3000,
    parameter integer POWERUP_US=50000, RESET_US=2000, STARTUP_US=50000,
    parameter integer STALE_US=10000, RETRY_US=1000000,
    parameter integer LOW_LATENCY_FILTER=1
)(
    input clk, input rst_n, inout imu_i2c_scl, inout imu_i2c_sda,
    output ready, fault, sample_valid, sample_stale, temperature_valid,
    output signed [15:0] ax, ay, az, gx, gy, gz, temperature_raw,
    output [31:0] sample_seq, timestamp_us, commit_timestamp_us, sample_age_us, now_us,
    output [15:0] observe_to_commit_us, sample_interval_us, error_count, invalid_count,
    output [7:0] who_am_i, last_error
);
    icm42688_fast_reader #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.POLL_US(POLL_US),
        .STALE_WARN_US(STALE_WARN_US),.POWERUP_US(POWERUP_US),.RESET_US(RESET_US),.STARTUP_US(STARTUP_US),
        .STALE_US(STALE_US),.RETRY_US(RETRY_US),.LOW_LATENCY_FILTER(LOW_LATENCY_FILTER)) reader(
        .clk(clk),.rst_n(rst_n),.imu_i2c_scl(imu_i2c_scl),.imu_i2c_sda(imu_i2c_sda),
        .ready(ready),.fault(fault),.sample_valid(sample_valid),.sample_stale(sample_stale),
        .temperature_valid(temperature_valid),.ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),
        .temperature_raw(temperature_raw),.sample_seq(sample_seq),.timestamp_us(timestamp_us),
        .commit_timestamp_us(commit_timestamp_us),.sample_age_us(sample_age_us),.now_us(now_us),
        .observe_to_commit_us(observe_to_commit_us),.sample_interval_us(sample_interval_us),
        .error_count(error_count),.invalid_count(invalid_count),.who_am_i(who_am_i),.last_error(last_error));
endmodule
