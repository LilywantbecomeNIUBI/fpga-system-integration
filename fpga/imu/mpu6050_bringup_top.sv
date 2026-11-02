// Static RevB MPU6050 check. Motor pads are constants; there is no command receiver.
module mpu6050_bringup_top(
    input wire clk50m,reset_n,
    inout wire imu_i2c_scl,imu_i2c_sda,
    output wire uart_tx,output wire [7:0] led,
    output wire left_in1,left_in2,right_in1,right_in2,
    output wire legacy_m1_disable,legacy_m2_disable,
    output wire [5:0] legacy_m1_pwm_disabled,legacy_m2_pwm_disabled,
    output wire [1:0] legacy_cal_disabled
);
    reg [1:0] release_pipe;
    always @(posedge clk50m or negedge reset_n)
        if(!reset_n) release_pipe<=0;
        else release_pipe<={release_pipe[0],1'b1};
    wire rst_n=release_pipe[1];
    wire ready,fault,valid,stale,temp_valid;
    wire signed [15:0] ax,ay,az,gx,gy,gz,temp;
    wire [31:0] seq,stamp,now_us;
    wire [7:0] who,error;
    wire [15:0] errors,invalid,drops,read_us,interval_us;
    imu_1khz_core imu(.clk(clk50m),.rst_n(rst_n),.imu_i2c_scl(imu_i2c_scl),.imu_i2c_sda(imu_i2c_sda),
        .ready(ready),.fault(fault),.sample_valid(valid),.sample_stale(stale),.temperature_valid(temp_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.timestamp_us(stamp),.now_us(now_us),.who_am_i(who),.last_error(error),
        .error_count(errors),.invalid_count(invalid),.observe_to_commit_us(read_us),.sample_interval_us(interval_us),
        .commit_timestamp_us(),.sample_age_us());
    imu_fast_uart_stream #(.DECIMATION(20),.FAST_BUS(0),.PROFILE_ID(4)) telemetry(
        .clk(clk50m),.rst_n(rst_n),.sample_valid(valid),.ready(ready),.fault(fault),
        .sample_stale(stale),.temperature_valid(temp_valid),.ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),
        .temperature_raw(temp),.sample_seq(seq),.timestamp_us(stamp),.now_us(now_us),.who_am_i(who),
        .last_error(error),.error_count(errors),.invalid_count(invalid),.observe_to_commit_us(read_us),
        .sample_interval_us(interval_us),.tx(uart_tx),.drop_count(drops),.debug_seq());
    reg [25:0] heartbeat;
    always @(posedge clk50m or negedge rst_n)
        if(!rst_n) heartbeat<=0;else heartbeat<=heartbeat+1'b1;
    assign led={drops!=0,errors!=0,stale,seq[0],seq!=0,fault,ready,heartbeat[25]};
    assign {left_in1,left_in2,right_in1,right_in2}=0;
    assign {legacy_m1_disable,legacy_m2_disable}=0;
    assign {legacy_m1_pwm_disabled,legacy_m2_pwm_disabled,legacy_cal_disabled}=0;
endmodule
