`timescale 1ns/1ps
module tb_fast_reader;
    reg clk=0,rst_n=0,present=1,corrupt=0,invalid=0,freeze_samples=0;
    reg [7:0] identity=8'h47;
    always #10 clk=~clk;
    tri1 scl,sda;
    wire ready,fault,valid,stale,temp_valid;
    wire signed [15:0] ax,ay,az,gx,gy,gz,temp;
    wire [31:0] seq,stamp,commit_stamp,age,now_us;
    wire [7:0] who,error;
    wire [15:0] errors,invalids,read_us,interval_us;
    imu_1khz_core #(.POWERUP_US(100),.RESET_US(1200),.STARTUP_US(500),.RETRY_US(2000)) dut(
        .clk(clk),.rst_n(rst_n),.imu_i2c_scl(scl),.imu_i2c_sda(sda),
        .ready(ready),.fault(fault),.sample_valid(valid),.sample_stale(stale),.temperature_valid(temp_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.timestamp_us(stamp),.commit_timestamp_us(commit_stamp),.sample_age_us(age),
        .who_am_i(who),.last_error(error),.error_count(errors),.invalid_count(invalids),
        .observe_to_commit_us(read_us),.sample_interval_us(interval_us),.now_us(now_us));
    icm42688_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(present),.identity(identity),
        .corrupt_readback(corrupt),.invalid_data(invalid),.freeze_samples(freeze_samples));
    integer samples=0,old_seq;
    reg [31:0] previous_stamp=0;
    reg [95:0] previous_sample=0;
    wire [95:0] full_sample={ax,ay,az,gx,gy,gz};
    always @(posedge clk) if(rst_n) begin
        #1;
        if(valid) begin
            if(!ready || fault || stale) $fatal(1,"published invalid state");
            if(full_sample!==96'h0400f8002000800103e87fff) $fatal(1,"signed/atomic sample");
            if(stamp>=commit_stamp || commit_stamp>now_us || age>1) $fatal(1,"timestamps");
            if(read_us<120 || read_us>250 || commit_stamp-stamp!=read_us) $fatal(1,"transport latency %0d",read_us);
            if(samples!=0 && stamp<=previous_stamp) $fatal(1,"nonmonotonic observation");
            previous_stamp=stamp; samples=samples+1;
        end
        if(!valid && full_sample!==previous_sample) $fatal(1,"partial sample");
        previous_sample=full_sample;
    end
    initial begin
        repeat(10) @(negedge clk); rst_n=1;
        wait(samples>=12);
        if(sensor.regs['h4f]!==8'h26 || sensor.regs['h50]!==8'h46 || sensor.regs['h52]!==8'h00 ||
           sensor.regs['h51]!==8'h16 || sensor.regs['h53]!==8'h0d || sensor.regs[128+'h7c]!==8'h50 ||
           sensor.regs['h16]!==0 || who!==8'h47 || !temp_valid || temp!==132) $fatal(1,"profile/readback/temperature");
        if(errors || invalids || stale) $fatal(1,"normal counters");
        freeze_samples=1;
        wait(stale); #1;
        if(!ready || fault || age<3000 || age>3001) $fatal(1,"3ms stale warning late %0d",age);
        old_seq=seq;
        wait(fault); #1;
        if(error!=8'h13 || seq!=old_seq) $fatal(1,"10ms watchdog/duplicate");
        freeze_samples=0; wait(ready && !stale); wait(seq>old_seq);
        present=0; wait(fault); #1; if(error!=1 || ready || !stale) $fatal(1,"NACK handling");
        old_seq=seq; identity=8'h42; present=1; wait(error==8'h10 && fault);
        if(who!==8'h42) $fatal(1,"bad WHO");
        identity=8'h47; wait(ready && !stale); wait(seq>old_seq);
        invalid=1; wait(invalids>=2); old_seq=seq;
        wait(error==8'h13 && fault); if(seq!=old_seq) $fatal(1,"invalid published");
        invalid=0; corrupt=1; wait(error==8'h11 && fault);
        old_seq=seq; #5000000;
        if(ready || seq!=old_seq) $fatal(1,"readback failure admitted");
        corrupt=0; wait(ready && !stale); wait(seq>old_seq);
        $display("PASS fast reader: 1kHz profile, 12-byte atomic axes, slow temp, metadata, 3ms stale, faults/recovery");
        $finish;
    end
    initial begin #120000000; $fatal(1,"fast reader timeout state=%0d step=%0d error=%h",dut.reader.state,dut.reader.step,error); end
endmodule
