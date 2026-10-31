`timescale 1ns/1ps
// ICM-42688-P DS-000347: banks 0/1, interface settings section 12.3.
// 1 kHz, +/-4 g, +/-1000 dps, FIFO bypass; 150us idle polling.
// Latest-sample polling: published sequence counts successful reads, NOT sensor
// conversion events. timestamp_us is observation time, not ADC sample time.
module icm42688_fast_reader #(
    parameter integer CLK_HZ=50000000, I2C_HZ=1000000,
    parameter integer POWERUP_US=50000, RESET_US=2000, STARTUP_US=50000,
    parameter integer POLL_US=150, STALE_US=10000, RETRY_US=1000000,
    parameter integer STALE_WARN_US=3000, TEMP_PERIOD_US=100000,
    parameter integer BUS_TIMEOUT_US=2000, LOW_LATENCY_FILTER=1
)(
    input clk, input rst_n, inout imu_i2c_scl, inout imu_i2c_sda,
    output reg ready, output reg fault, output reg sample_valid,
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
        DATA_ISSUE=12, DATA_WAIT=13, FAILED=14, RETRY=15,
        TEMP_ISSUE=16, TEMP_WAIT=17;
    localparam integer LAST_STEP=20;
    reg [4:0] state, step;
    reg [31:0] wait_count, poll_count, age_count;
    reg [31:0] us_counter, us_divider, observation;
    reg [31:0] temp_count;
    reg have_sample, have_previous;
    assign now_us=us_counter;
    assign sample_age_us=have_sample ? us_counter-commit_timestamp_us : 32'hffffffff;
    assign sample_stale=!ready || !have_sample || sample_age_us>=STALE_WARN_US;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin us_counter<=0; us_divider<=0; end
        else if (us_divider==US_DIV-1) begin us_counter<=us_counter+1'b1; us_divider<=0; end
        else us_divider<=us_divider+1'b1;

    // req is pulsed only when the transaction engine is idle; command arguments
    // remain stable through its acceptance edge. 32-byte bursts reserve room
    // for a future FIFO reader without changing the bus interface.
    reg req, bus_read;
    reg [7:0] address, write_value;
    reg [5:0] length;
    wire busy, done;
    wire [3:0] bus_error;
    wire [255:0] result;
    imu_fast_i2c_master #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ),.TIMEOUT_US(BUS_TIMEOUT_US)) bus (
        .clk(clk),.rst_n(rst_n),.req(req),.read_not_write(bus_read),
        .device_addr(7'h69),.reg_addr(address),.write_data(write_value),.read_length(length),
        .busy(busy),.done(done),.error(bus_error),.read_data(result),
        .scl(imu_i2c_scl),.sda(imu_i2c_sda));

    function [7:0] config_addr;
        input [4:0] n;
        begin case(n)
            0:config_addr=8'h76; 1:config_addr=8'h75; 2:config_addr=8'h11;
            3:config_addr=8'h2d; 4:config_addr=8'h13; 5:config_addr=8'h4c;
            6:config_addr=8'h65; 7:config_addr=8'h68; 8:config_addr=8'h62;
            9:config_addr=8'h4f; 10:config_addr=8'h50;
            11:config_addr=8'h51; 12:config_addr=8'h53; 13:config_addr=8'h52;
            14:config_addr=8'h76; 15:config_addr=8'h7a; 16:config_addr=8'h7b;
            17:config_addr=8'h7c; 18:config_addr=8'h76; 19:config_addr=8'h16;
            20:config_addr=8'h4e;
            default:config_addr=0;
        endcase end
    endfunction
    function [7:0] config_value;
        input [4:0] n;
        begin case(n)
            1:config_value=8'h47; 2:config_value=8'h01; 3:config_value=8'h10;
            4:config_value=8'h09; // I2C slew=1, SPI slew=1, per section 12.3
            5:config_value=8'h12; // big-endian data, disable SPI (keep I2C)
            9:config_value=8'h26; 10:config_value=8'h46;
            11:config_value=8'h04; 12:config_value=8'h08; // explicit 2nd-order UI filters
            13:config_value=LOW_LATENCY_FILTER ? 8'h00 : 8'h11;
            14:config_value=8'h01; 17:config_value=8'h10; // I3C_EN=1, SDR/DDR/IBI off
            20:config_value=8'h0f;
            default:config_value=0;
        endcase end
    endfunction
    function [7:0] config_mask;
        input [4:0] n;
        begin case(n)
            0,14,18:config_mask=8'h07;
            3:config_mask=8'h10; 4:config_mask=8'h3f; 5:config_mask=8'h13;
            6,7:config_mask=8'h7f; 8:config_mask=8'h7f;
            9,10:config_mask=8'hef;
            11:config_mask=8'h0c; 12:config_mask=8'h18; 13:config_mask=8'hff;
            15:config_mask=8'h40; 16:config_mask=8'h06; 17:config_mask=8'h1f;
            19:config_mask=8'hc0; 20:config_mask=8'h3f;
            default:config_mask=8'hff;
        endcase end
    endfunction
    wire [7:0] mask=config_mask(step), expected=config_value(step);
    wire invalid_sample=result[95:80]==16'h8000 || result[79:64]==16'h8000 ||
        result[63:48]==16'h8000 || result[47:32]==16'h8000 ||
        result[31:16]==16'h8000 || result[15:0]==16'h8000;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=BOOT; step<=0; wait_count<=0; poll_count<=0; age_count<=0;
            req<=0; bus_read<=0; address<=0; write_value<=0; length<=1;
            ready<=0; fault<=0; sample_valid<=0; who_am_i<=0; last_error<=0;
            error_count<=0; invalid_count<=0; sample_seq<=0; timestamp_us<=0; observation<=0;
            ax<=0; ay<=0; az<=0; gx<=0; gy<=0; gz<=0; temperature_raw<=0;
            temp_count<=US_DIV*TEMP_PERIOD_US; temperature_valid<=0;
            have_sample<=0; have_previous<=0; commit_timestamp_us<=0;
            observe_to_commit_us<=0; sample_interval_us<=0;
        end else begin
            req<=0; sample_valid<=0;
            if (poll_count<US_DIV*POLL_US) poll_count<=poll_count+1'b1;
            if (temp_count<US_DIV*TEMP_PERIOD_US) temp_count<=temp_count+1'b1;
            if (ready && age_count<US_DIV*STALE_US) age_count<=age_count+1'b1;
            // Staleness includes unsuccessful or invalid reads; no stale sample
            // can be published as current after the watchdog expires.
            if (ready && age_count>=US_DIV*STALE_US) begin
                last_error<=8'h13; state<=FAILED; ready<=0;
            end else case (state)
                BOOT: if (wait_count>=US_DIV*POWERUP_US-1) begin
                    wait_count<=0; step<=0; state<=ENTRY;
                end else wait_count<=wait_count+1'b1;
                ENTRY: begin
                    address<=config_addr(step); length<=1;
                    if (step==2) begin write_value<=1; state<=WRITE_ISSUE; end
                    else state<=READ_ISSUE;
                end
                READ_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=READ_WAIT; end
                READ_WAIT: if (done) begin
                    if (step==1) who_am_i<=result[7:0];
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else if (step==1 || step==3) begin
                        if ((result[7:0]&mask)!=(expected&mask)) begin
                            last_error<=step==1 ? 8'h10 : 8'h12; state<=FAILED;
                        end else begin step<=step+1'b1; state<=ENTRY; end
                    end else begin
                        write_value<=(result[7:0]&~mask)|(expected&mask); state<=WRITE_ISSUE;
                    end
                end
                WRITE_ISSUE: if (!busy) begin bus_read<=0; req<=1; state<=WRITE_WAIT; end
                WRITE_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else if (step==2 || step==LAST_STEP) begin wait_count<=0; state<=DELAY; end
                    else state<=VERIFY_ISSUE;
                end
                DELAY: if ((step==2 && wait_count>=US_DIV*RESET_US-1) ||
                           (step==LAST_STEP && wait_count>=US_DIV*STARTUP_US-1)) begin
                    wait_count<=0;
                    if (step==2) begin step<=3; state<=ENTRY; end
                    else state<=VERIFY_ISSUE;
                end else wait_count<=wait_count+1'b1;
                VERIFY_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=VERIFY_WAIT; end
                VERIFY_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else if ((result[7:0]&mask)!=(expected&mask)) begin last_error<=8'h11; state<=FAILED; end
                    else if (step==LAST_STEP) begin
                        ready<=1; fault<=0; age_count<=0; poll_count<=0; state<=POLL_WAIT;
                        have_previous<=0; temperature_valid<=0; temp_count<=US_DIV*TEMP_PERIOD_US;
                    end else begin step<=step+1'b1; state<=ENTRY; end
                end
                POLL_WAIT: if (poll_count>=US_DIV*POLL_US) begin
                    address<=8'h2d; length<=1; state<=STATUS_ISSUE; poll_count<=0;
                end else if (temp_count>=US_DIV*TEMP_PERIOD_US) begin
                    address<=8'h1d; length<=2; state<=TEMP_ISSUE;
                end
                STATUS_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=STATUS_WAIT; end
                STATUS_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else if (result[3]) begin
                        observation<=us_counter; address<=8'h1f; length<=12; state<=DATA_ISSUE;
                    end else state<=POLL_WAIT;
                end
                DATA_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=DATA_WAIT; end
                DATA_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else begin
                        if (invalid_sample) begin
                            if (invalid_count!=16'hffff) invalid_count<=invalid_count+1'b1;
                        end else begin
                            ax<=result[95:80]; ay<=result[79:64];
                            az<=result[63:48]; gx<=result[47:32]; gy<=result[31:16]; gz<=result[15:0];
                            timestamp_us<=observation; sample_seq<=sample_seq+1'b1;
                            sample_valid<=1; age_count<=0;
                            have_sample<=1; have_previous<=1; commit_timestamp_us<=us_counter;
                            observe_to_commit_us<=us_counter-observation;
                            sample_interval_us<=have_previous ? us_counter-commit_timestamp_us : 0;
                        end
                        state<=POLL_WAIT;
                    end
                end
                TEMP_ISSUE: if (!busy) begin bus_read<=1; req<=1; state<=TEMP_WAIT; end
                TEMP_WAIT: if (done) begin
                    if (bus_error!=0) begin last_error<={4'b0,bus_error}; state<=FAILED; end
                    else begin
                        temperature_raw<=result[15:0]; temperature_valid<=1; temp_count<=0; state<=POLL_WAIT;
                    end
                end
                FAILED: begin
                    ready<=0; fault<=1; wait_count<=0;
                    have_previous<=0; temperature_valid<=0;
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
