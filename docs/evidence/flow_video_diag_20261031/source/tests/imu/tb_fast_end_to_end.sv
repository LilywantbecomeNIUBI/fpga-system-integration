`timescale 1ns/1ps
module tb_fast_end_to_end;
    reg clk=0,rst_n=0;
    always #10 clk=~clk;
    tri1 scl,sda;
    wire ready,fault,valid,stale,temp_valid,tx;
    wire signed [15:0] ax,ay,az,gx,gy,gz,temp;
    wire [31:0] seq,stamp,commit_stamp,age,now_us,debug_seq;
    wire [7:0] who,error;
    wire [15:0] errors,invalids,read_us,interval_us,drops;
    wire slow_tx;
    wire [15:0] slow_drops;
    imu_1khz_core #(.POWERUP_US(100),.RESET_US(1200),.STARTUP_US(500)) core(
        .clk(clk),.rst_n(rst_n),.imu_i2c_scl(scl),.imu_i2c_sda(sda),
        .ready(ready),.fault(fault),.sample_valid(valid),.sample_stale(stale),.temperature_valid(temp_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.timestamp_us(stamp),.commit_timestamp_us(commit_stamp),.sample_age_us(age),
        .who_am_i(who),.last_error(error),.error_count(errors),.invalid_count(invalids),
        .observe_to_commit_us(read_us),.sample_interval_us(interval_us),.now_us(now_us));
    imu_fast_uart_stream debug_stream(.clk(clk),.rst_n(rst_n),.sample_valid(valid),.ready(ready),.fault(fault),
        .sample_stale(stale),.temperature_valid(temp_valid),.ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),
        .temperature_raw(temp),.sample_seq(seq),.timestamp_us(stamp),.now_us(now_us),.who_am_i(who),
        .last_error(error),.error_count(errors),.invalid_count(invalids),.observe_to_commit_us(read_us),
        .sample_interval_us(interval_us),.tx(tx),.drop_count(drops),.debug_seq(debug_seq));
    icm42688_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(1'b1),.identity(8'h47),
        .corrupt_readback(1'b0),.invalid_data(1'b0),.freeze_samples(1'b0));
    // Intentionally overload a second, slower debug consumer. Its drops must
    // never slow down or otherwise change the 1kHz control sample stream.
    imu_fast_uart_stream #(.BAUD(57600)) overloaded_debug(.clk(clk),.rst_n(rst_n),
        .sample_valid(valid),.ready(ready),.fault(fault),.sample_stale(stale),.temperature_valid(temp_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.timestamp_us(stamp),.now_us(now_us),.who_am_i(who),.last_error(error),
        .error_count(errors),.invalid_count(invalids),.observe_to_commit_us(read_us),
        .sample_interval_us(interval_us),.tx(slow_tx),.drop_count(slow_drops),.debug_seq());
    integer f,raw_file,bytes=0,i,samples=0;
    reg [7:0] value;
    initial begin
        f=$fopen("results/fast_end_to_end.bin","wb"); raw_file=$fopen("results/raw_samples.csv","w");
        $fwrite(raw_file,"seq,observe_us,commit_us,read_us,interval_us,stale\n");
        repeat(10) @(negedge clk); rst_n=1;
    end
    always @(posedge clk) begin
        #1;
        if(valid) begin
            samples=samples+1;
            $fwrite(raw_file,"%0d,%0d,%0d,%0d,%0d,%0d\n",seq,stamp,commit_stamp,read_us,interval_us,stale);
        end
    end
    always begin
        @(negedge tx); repeat(217) @(posedge clk); #1;
        if(tx!==0) $fatal(1,"start bit");
        for(i=0;i<8;i=i+1) begin repeat(434) @(posedge clk); #1; value[i]=tx; end
        repeat(434) @(posedge clk); #1;
        if(tx!==1) $fatal(1,"stop bit");
        $fwrite(f,"%c",value); bytes=bytes+1;
        if(bytes==1920) begin
            if(errors || invalids || drops || !ready || fault || stale || samples<200) $fatal(1,"integration counters raw=%0d drops=%0d",samples,drops);
            if(slow_drops==0) $fatal(1,"overloaded debug consumer was not stressed");
            $fclose(f); $fclose(raw_file);
            $display("PASS fast end-to-end: %0d raw samples, 40 decimated v2 records, UART independent, no drops",samples); $finish;
        end
    end
    initial begin #230000000; $fatal(1,"integration timeout bytes=%0d raw=%0d",bytes,samples); end
endmodule
