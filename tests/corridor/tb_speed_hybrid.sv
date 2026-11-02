`timescale 1ns/1ps
module tb_speed_hybrid;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,clear=1;
    reg signed [31:0] position=0;
    reg [9:0] phase=0;
    wire tick=phase==999;
    always @(posedge clk)if(!reset_n)phase<=0;else phase<=tick ? 0 : phase+1'b1;
    wire signed [31:0] speed;
    wire recent,fault,count_mode;
    reg capture_low=0;
    integer low_file,last_sample_position=0;
    always @(negedge clk)if(capture_low && phase==40)begin
        $fwrite(low_file,"%0d,%0d\n",speed,(position-last_sample_position)*2560);
        last_sample_position=position;
    end
    wheel_speed_hybrid #(.CLK_HZ(1000000)) dut(.clk(clk),.reset_n(reset_n),.clear(clear),.tick(tick),
        .position(position),.velocity_q8(speed),.motion_recent(recent),.fault(fault),.count_mode(count_mode));
    task cycles(input integer n);begin repeat(n)@(negedge clk);end endtask
    task restart(input signed [31:0] p);begin
        clear=1;position=p;cycles(3);clear=0;cycles(2);
    end endtask
    task edge_train(input integer delta,interval,n);integer j;begin
        for(j=0;j<n;j=j+1)begin position=position+delta;cycles(interval);end
    end endtask
    integer saved,i;
    initial begin
        cycles(3);reset_n=1;restart(0);
        edge_train(2,5500,14);
        if(speed<920 || speed>940 || count_mode || fault)$fatal(1,"T-mode two-unit edge %d",speed);
        // Fractional output, without the 0/2 counts-per-ms pulse train.
        low_file=$fopen("build/corridor/tests/low_speed.csv","w");
        $fwrite(low_file,"hybrid_q8,one_ms_count_q8\n");
        last_sample_position=position;capture_low=1;
        for(i=0;i<8;i=i+1)begin
            position=position+2;cycles(5500);
            if(speed<920 || speed>940)$fatal(1,"low speed jitter %d",speed);
        end
        capture_low=0;$fclose(low_file);
        saved=speed;cycles(20000);
        if(speed<=0 || speed>=saved)$fatal(1,"edge age must decay speed %d",speed);
        cycles(51000);if(speed!=0 || recent)$fatal(1,"stopped timeout retained speed");
        restart(0);edge_train(1,5500,14);
        if(speed<460 || speed>470 || count_mode)$fatal(1,"one-unit decoder %d",speed);
        restart(32'h7ffffffc);edge_train(2,5500,14);
        if(position>=0 || speed<920 || speed>940 || fault)$fatal(1,"signed position wrap");
        restart(0);
        // Exercise unsigned timestamp rollover before a fresh pair is acquired.
        dut.timestamp=32'hfffff000;
        edge_train(2,5500,14);
        if(speed<920 || speed>940 || fault)$fatal(1,"timestamp rollover %d",speed);
        restart(0);edge_train(2,1000,16);
        if(count_mode || speed<5100 || speed>5140)$fatal(1,"T hysteresis entry %d",speed);
        edge_train(2,200,100);
        if(!count_mode || speed<24960 || speed>26240 || fault)$fatal(1,"M mode %d",speed);
        edge_train(2,1000,16);
        if(!count_mode || speed<4480 || speed>5760)$fatal(1,"M hysteresis hold %d",speed);
        edge_train(2,2000,12);
        if(count_mode || speed<2540 || speed>2580)$fatal(1,"M to T transition %d",speed);
        // Reverse while an old period quotient may be in flight.
        restart(0);edge_train(2,5500,5);
        while(!dut.dividing)cycles(1);
        position=position-2;cycles(3);
        if(speed>0 || count_mode)$fatal(1,"stale forward result after reverse");
        edge_train(-2,5500,10);
        if(speed> -920 || speed< -940 || fault)$fatal(1,"negative T mode %d",speed);
        clear=1;cycles(1);clear=0;
        position=position+9;cycles(3);
        if(!fault || speed!=0)$fatal(1,"position discontinuity not latched");
        cycles(10000);if(!fault || speed!=0)$fatal(1,"fault auto-cleared");
        restart(position);if(fault || speed!=0)$fatal(1,"explicit observer clear");
        edge_train(2,5500,4);while(!dut.dividing)cycles(1);
        reset_n=0;cycles(2);reset_n=1;clear=1;cycles(2);clear=0;cycles(2000);
        if(speed!=0 || fault)$fatal(1,"reset leaked in-flight quotient");
        $display("PASS hybrid speed: fractional +/-1/2 steps, aging/stop, position/time wrap, M/T hysteresis, reversal, discontinuity, reset");$finish;
    end
    initial begin #30000000;$fatal(1,"timeout");end
endmodule
