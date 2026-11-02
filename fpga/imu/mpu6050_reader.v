`timescale 1ns/1ps
// MPU-6050: RM-MPU-6000A-00 Rev4.2, PS-MPU-6000A-00 Rev3.4.
// Address 0x68 (AD0 measured low); accepts MPU6050/MPU6500 identities.
// 1 kHz, +/-4 g (8192 LSB/g), +/-1000 dps (32.8 LSB/dps).
// Poll INT_STATUS then burst all 14 bytes from ACCEL_XOUT_H. Sequence counts
// successful reads, not conversions; timestamps mark observation, not sampling.
module mpu6050_reader #(
    parameter integer CLK_HZ=50000000, I2C_HZ=400000,
    parameter integer POWERUP_US=100000, RESET_US=100000, STARTUP_US=100000,
    parameter integer POLL_US=150, STALE_US=10000, RETRY_US=1000000,
    parameter integer STALE_WARN_US=3000, BUS_TIMEOUT_US=2000,
    parameter integer LOW_LATENCY_FILTER=1
)(
    input clk, input rst_n, inout imu_i2c_scl, inout imu_i2c_sda,
    output reg ready, fault, sample_valid,
    output reg signed [15:0] ax, ay, az, gx, gy, gz, temperature_raw,
    output reg [31:0] sample_seq, timestamp_us,
    output reg [7:0] who_am_i, last_error,
    output reg [15:0] error_count, invalid_count,
    output [31:0] now_us,
    output sample_stale, output [31:0] sample_age_us,
    output reg temperature_valid,
    output reg [31:0] commit_timestamp_us,
    output reg [15:0] observe_to_commit_us, sample_interval_us
);
    localparam integer US_DIV=CLK_HZ/1000000;
    localparam BOOT=0, ENTRY=1, READ_ISSUE=2, READ_WAIT=3,
        WRITE_ISSUE=4, WRITE_WAIT=5, DELAY=6, VERIFY_ISSUE=7,
        VERIFY_WAIT=8, POLL_WAIT=9, STATUS_ISSUE=10, STATUS_WAIT=11,
        DATA_ISSUE=12, DATA_WAIT=13, FAILED=14, RETRY=15;
    localparam integer LAST_STEP=15;
    reg [3:0] state, step;
    reg [31:0] wait_count, poll_count, age_count;
    reg [31:0] us_counter, us_divider, observation;
    reg have_sample, have_previous;
    reg stale_due;
    reg [31:0] stale_deadline_minus1;
    wire us_tick=us_divider==US_DIV-1;
    wire is_mpu6500=who_am_i==8'h70;
    assign now_us=us_counter;
    assign sample_age_us=have_sample ? us_counter-commit_timestamp_us : 32'hffffffff;
    // Equality is evaluated before the microsecond counter advances. The
    // registered stale flag therefore asserts on the original deadline edge,
    // including wrap; no 32-bit age subtractor feeds navigation enables.
    assign sample_stale=!ready || !have_sample || stale_due;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin us_counter<=0; us_divider<=0; end
        else if (us_divider==US_DIV-1) begin us_counter<=us_counter+1'b1; us_divider<=0; end
        else us_divider<=us_divider+1'b1;

    reg req, bus_read;
    reg [7:0] address, write_value;
    reg [5:0] length;
    wire busy, done;
    wire [3:0] bus_error;
    wire [255:0] result;
    // MPU-6050 only supports Standard/Fast mode, even if an old caller asks for 1 MHz.
    imu_fast_i2c_master #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ>400000 ? 400000 : I2C_HZ),
        .TIMEOUT_US(BUS_TIMEOUT_US)) bus (
        .clk(clk),.rst_n(rst_n),.req(req),.read_not_write(bus_read),
        .device_addr(7'h68),.reg_addr(address),.write_data(write_value),.read_length(length),
        .busy(busy),.done(done),.error(bus_error),.read_data(result),
        .scl(imu_i2c_scl),.sda(imu_i2c_sda));

    function [7:0] config_addr;
        input [3:0] n;
        begin case(n)
            0,3:config_addr=8'h75; // WHO_AM_I before/after device reset
            1,2,4:config_addr=8'h6b;
            5:config_addr=8'h6c; 6:config_addr=8'h6a;
            7:config_addr=8'h23; 8:config_addr=8'h24;
            9:config_addr=8'h37; 10:config_addr=8'h38;
            11:config_addr=8'h1a; 12:config_addr=8'h1b;
            13:config_addr=8'h1c; 14:config_addr=8'h19;
            15:config_addr=8'h1d; // MPU6500 ACCEL_CONFIG2, absent on MPU6050.
            default:config_addr=0;
        endcase end
    endfunction
    function [7:0] config_value;
        input [3:0] n;
        begin case(n)
            0:config_value=8'h68; 3:config_value=who_am_i; 1:config_value=8'h80;
            4:config_value=8'h01; // wake, temperature enabled, X gyro PLL
            9:config_value=is_mpu6500 ? 8'h00 : 8'h20;
            10:config_value=8'h01; // enable DATA_RDY status; INT pin is unused
            11:config_value=is_mpu6500 ? 8'h02 : (LOW_LATENCY_FILTER ? 8'h01 : 8'h03);
            12:config_value=8'h10; 13:config_value=8'h08;
            15:config_value=8'h02; // Match the working STM32 MPU6500 profile.
            // FIFO, auxiliary master, standby, divider all explicitly zero.
            default:config_value=0;
        endcase end
    endfunction
    function [7:0] config_mask;
        input [3:0] n;
        begin case(n)
            2:config_mask=8'h80; 4:config_mask=8'hef;
            6:config_mask=8'h7f; 9:config_mask=8'hfe;
            11:config_mask=8'h3f; 12,13:config_mask=8'hf8;
            15:config_mask=8'h0f;
            default:config_mask=8'hff;
        endcase end
    endfunction
    wire [7:0] mask=config_mask(step), expected=config_value(step);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=BOOT; step<=0; wait_count<=0; poll_count<=0; age_count<=0;
            req<=0; bus_read<=0; address<=0; write_value<=0; length<=1;
            ready<=0; fault<=0; sample_valid<=0; who_am_i<=0; last_error<=0;
            error_count<=0; invalid_count<=0; sample_seq<=0; timestamp_us<=0; observation<=0;
            ax<=0; ay<=0; az<=0; gx<=0; gy<=0; gz<=0; temperature_raw<=0;
            temperature_valid<=0; have_sample<=0; have_previous<=0; commit_timestamp_us<=0;
            stale_due<=1;stale_deadline_minus1<=0;
            observe_to_commit_us<=0; sample_interval_us<=0;
        end else begin
            req<=0; sample_valid<=0;
            if(us_tick && us_counter==stale_deadline_minus1)stale_due<=1;
            if (poll_count<US_DIV*POLL_US) poll_count<=poll_count+1'b1;
            if (ready && age_count<US_DIV*STALE_US) age_count<=age_count+1'b1;
            if (ready && age_count>=US_DIV*STALE_US) begin
                last_error<=8'h13; state<=FAILED; ready<=0;
            end else case (state)
                BOOT: if (wait_count>=US_DIV*POWERUP_US-1) begin
                    wait_count<=0; step<=0; state<=ENTRY;
                end else wait_count<=wait_count+1'b1;
                ENTRY: begin
                    address<=config_addr(step); length<=1;
                    if (step==1) begin write_value<=8'h80; state<=WRITE_ISSUE; end
                    else state<=READ_ISSUE;
                end
                READ_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=READ_WAIT; end
                READ_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else if (step==0 || step==2 || step==3) begin
                        if (step!=2) who_am_i<=result[7:0];
                        if ((step==0 && result[7:0]!=8'h68 && result[7:0]!=8'h70) ||
                            (step!=0 && (result[7:0]&mask)!=(expected&mask))) begin
                            last_error<=step==2 ? 8'h12 : 8'h10; state<=FAILED;
                        end else begin step<=step+1'b1; state<=ENTRY; end
                    end else begin
                        write_value<=(result[7:0]&~mask)|(expected&mask); state<=WRITE_ISSUE;
                    end
                end
                WRITE_ISSUE: if (!busy) begin bus_read<=0; req<=1; state<=WRITE_WAIT; end
                WRITE_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else if (step==1 || step==4) begin wait_count<=0; state<=DELAY; end
                    else state<=VERIFY_ISSUE;
                end
                DELAY: if ((step==1 && wait_count>=US_DIV*RESET_US-1) ||
                           (step==4 && wait_count>=US_DIV*STARTUP_US-1)) begin
                    wait_count<=0;
                    if (step==1) begin step<=2; state<=ENTRY; end
                    else state<=VERIFY_ISSUE;
                end else wait_count<=wait_count+1'b1;
                VERIFY_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=VERIFY_WAIT; end
                VERIFY_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else if ((result[7:0]&mask)!=(expected&mask)) begin last_error<=8'h11; state<=FAILED; end
                    else if (step==LAST_STEP || (step==14 && !is_mpu6500)) begin
                        ready<=1; fault<=0; last_error<=0; age_count<=0; poll_count<=0; state<=POLL_WAIT;
                        have_sample<=0; have_previous<=0; temperature_valid<=0;
                    end else begin step<=step+1'b1; state<=ENTRY; end
                end
                POLL_WAIT: if (poll_count>=US_DIV*POLL_US) begin
                    address<=8'h3a; length<=1; state<=STATUS_ISSUE; poll_count<=0;
                end
                STATUS_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=STATUS_WAIT; end
                STATUS_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else if (result[0]) begin
                        observation<=us_counter; address<=8'h3b; length<=14; state<=DATA_ISSUE;
                    end else state<=POLL_WAIT;
                end
                DATA_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=DATA_WAIT; end
                DATA_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else begin
                        // Unlike ICM42688, 0x8000 is a valid MPU6050 full-scale value.
                        ax<=result[111:96]; ay<=result[95:80]; az<=result[79:64];
                        temperature_raw<=result[63:48];
                        gx<=result[47:32]; gy<=result[31:16]; gz<=result[15:0];
                        timestamp_us<=observation; sample_seq<=sample_seq+1'b1;
                        sample_valid<=1; age_count<=0; temperature_valid<=1;
                        have_sample<=1; have_previous<=1; commit_timestamp_us<=us_counter;
                        stale_deadline_minus1<=us_counter+STALE_WARN_US-1;
                        stale_due<=STALE_WARN_US==0 || (STALE_WARN_US==1 && us_tick);
                        observe_to_commit_us<=us_counter-observation;
                        sample_interval_us<=have_previous ? us_counter-commit_timestamp_us : 0;
                        state<=POLL_WAIT;
                    end
                end
                FAILED: begin
                    ready<=0; fault<=1; wait_count<=0; have_previous<=0; temperature_valid<=0;
                    if (error_count!=16'hffff) error_count<=error_count+1'b1;
                    state<=RETRY;
                end
                RETRY: if (wait_count>=US_DIV*RETRY_US-1 && !busy) begin
                    step<=0; state<=ENTRY; wait_count<=0;
                end else if (wait_count<US_DIV*RETRY_US) wait_count<=wait_count+1'b1;
                default: begin last_error<=8'h1f; state<=FAILED; end
            endcase
        end
    end
endmodule
