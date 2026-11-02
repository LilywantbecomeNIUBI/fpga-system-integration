`timescale 1ns/1ps
module tb_mpu6050_reader;
    reg clk=0,rst_n=0,present=1,corrupt=0,full_scale=0,freeze_samples=0,reset_stuck=0,hold_scl=0;
    reg [7:0] identity=8'h68;
    always #10 clk=~clk;
    tri1 scl,sda;
    assign scl=hold_scl ? 1'b0 : 1'bz;
    wire ready,fault,valid,stale,temp_valid;
    wire signed [15:0] ax,ay,az,gx,gy,gz,temp;
    wire [31:0] seq,stamp,commit_stamp,age,now_us;
    wire [7:0] who,error;
    wire [15:0] errors,invalids,read_us,interval_us;
    // Deliberately request the old 1MHz rate: the replacement must cap it.
    imu_1khz_core #(.I2C_HZ(1000000),.POWERUP_US(100),.RESET_US(1200),.STARTUP_US(500),.RETRY_US(2000)) dut(
        .clk(clk),.rst_n(rst_n),.imu_i2c_scl(scl),.imu_i2c_sda(sda),
        .ready(ready),.fault(fault),.sample_valid(valid),.sample_stale(stale),.temperature_valid(temp_valid),
        .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.temperature_raw(temp),
        .sample_seq(seq),.timestamp_us(stamp),.commit_timestamp_us(commit_stamp),.sample_age_us(age),
        .who_am_i(who),.last_error(error),.error_count(errors),.invalid_count(invalids),
        .observe_to_commit_us(read_us),.sample_interval_us(interval_us),.now_us(now_us));
    mpu6050_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(present),.identity(identity),
        .corrupt_readback(corrupt),.invalid_data(full_scale),.freeze_samples(freeze_samples),.reset_stuck(reset_stuck));
    integer samples=0,old_seq,saturated=0;
    reg [31:0] previous_stamp=0;
    reg [111:0] previous_sample=0;
    wire [111:0] full_sample={ax,ay,az,temp,gx,gy,gz};
    time last_rise=0,last_fall=0;
    always @(posedge scl) begin
        if(last_rise && $time-last_rise<2500) $fatal(1,"MPU6050 exceeds 400kHz");
        if(last_fall && $time-last_fall<1300) $fatal(1,"Fast mode tLOW");
        last_rise=$time;
    end
    always @(negedge scl) begin
        if(last_rise && $time-last_rise<600) $fatal(1,"Fast mode tHIGH");
        last_fall=$time;
    end
    always @(posedge clk) if(rst_n) begin
        #1;
        if(valid) begin
            if(!ready || fault || stale || !temp_valid) $fatal(1,"published invalid state");
            if((ax!==16'sd1024 && ax!==16'sh8000) ||
                {ay,az,temp,gx,gy,gz}!==96'hf80020000154800103e87fff) $fatal(1,"signed/order/temperature sample");
            if(ax===16'sh8000) saturated=saturated+1;
            if(stamp>=commit_stamp || commit_stamp>now_us || age>1) $fatal(1,"timestamps");
            if(read_us<400 || read_us>550 || commit_stamp-stamp!=read_us) $fatal(1,"transport latency %0d",read_us);
            if(samples!=0 && stamp<=previous_stamp) $fatal(1,"nonmonotonic observation");
            previous_stamp=stamp; samples=samples+1;
        end
        if(!valid && full_sample!==previous_sample) $fatal(1,"partial axes/temperature commit");
        previous_sample=full_sample;
    end
    initial begin
        repeat(10) @(negedge clk); rst_n=1;
        wait(samples>=12); $display("COVER normal");
        if(sensor.regs['h6b]!==1 || sensor.regs['h6c]!==0 || sensor.regs['h6a]!==0 ||
           sensor.regs['h23]!==0 || sensor.regs['h24]!==0 || sensor.regs['h37]!==8'h20 ||
           sensor.regs['h38]!==1 || sensor.regs['h1a]!==1 || sensor.regs['h1b]!==8'h10 ||
           sensor.regs['h1c]!==8'h08 || sensor.regs['h19]!==0 || who!==8'h68 || temp!==340)
            $fatal(1,"profile/readback/auxiliary bus");
        if(errors || invalids || stale || interval_us<900 || interval_us>1100) $fatal(1,"normal counters/rate %0d",interval_us);
        full_scale=1; wait(saturated>=2); $display("COVER full-scale"); if(invalids) $fatal(1,"MPU full-scale rejected"); full_scale=0;
        freeze_samples=1; wait(stale); #1;
        if(!ready || fault || age<3000 || age>3001) $fatal(1,"3ms warning %0d",age);
        old_seq=seq; wait(fault); #1;
        if(error!=8'h13 || seq!=old_seq || temp_valid) $fatal(1,"10ms watchdog/duplicate");
         $display("COVER stale watchdog"); freeze_samples=0; wait(ready && !stale); wait(seq>old_seq);
        present=0; wait(fault); #1; if(error!=1 || ready || !stale) $fatal(1,"NACK");
         $display("COVER NACK"); old_seq=seq; identity=8'h47; present=1; wait(error==8'h10 && fault);
        if(who!==8'h47) $fatal(1,"wrong identity diagnostic");
         $display("COVER wrong WHO"); identity=8'h68; wait(ready && !stale); wait(seq>old_seq);
        // Hold SCL after entering a data burst; no partial packet may commit.
        wait(dut.reader.state==13 && dut.reader.bus.state==12);
        @(negedge scl); hold_scl=1; old_seq=seq;
        wait(fault); #1; if(error!=2 || seq!=old_seq || ready || temp_valid) $fatal(1,"timeout/atomic failure");
         $display("COVER burst timeout"); corrupt=1; hold_scl=0; wait(error==8'h11 && fault);
         $display("COVER config mismatch"); old_seq=seq; reset_stuck=1;
        #4000000; if(ready || seq!=old_seq) $fatal(1,"bad config admitted");
        corrupt=0; wait(error==8'h12 && fault);
         $display("COVER reset stuck"); reset_stuck=0; wait(ready && !stale); wait(seq>old_seq);
        if(error || errors<6 || invalids) $fatal(1,"recovery counters");
        if(sensor.burst_reads<samples || sensor.master_nacks==0) $fatal(1,"burst completion");
        $display("PASS MPU6050: 0x68, <=400kHz, 1kHz, 14-byte atomic axes/temp, full-scale, stale/NACK/timeout/WHO/config/reset recovery");
        $finish;
    end
    initial begin #180000000; $fatal(1,"MPU6050 timeout state=%0d step=%0d error=%h",dut.reader.state,dut.reader.step,error); end
endmodule
