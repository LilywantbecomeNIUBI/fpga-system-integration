`timescale 1ns/1ps
module tb_lidar_points;
    reg clk=0,rst_n=0,pv=0,ready=1;always #10 clk=~clk;
    reg [3:0] index=0;reg [15:0] angle=0,distance=0;reg [7:0] intensity=0;
    reg [31:0] packets=0,errors=0;
    wire [383:0] frame;wire valid;
    lidar_points_source #(.CLK_HZ(1000000)) dut(.clk(clk),.rst_n(rst_n),
        .byte_valid(pv),.framing_error(1'b0),.point_valid(pv),.point_index(index),
        .angle(angle),.distance(distance),.intensity(intensity),.packets(packets),.errors(errors),
        .crc_errors(errors),.format_errors(32'd0),.timeouts(32'd0),.ready(ready),.frame(frame),.valid(valid));
    integer log,n,j,base,phase=0,raw_frames=0;reg stalled=0;reg [383:0] held;
    function automatic [15:0] crc16(input [367:0] value);
        reg [15:0] c;integer b,k;begin
            c=16'hffff;
            for(b=0;b<46;b=b+1)begin
                c=c^{value[b*8+:8],8'd0};
                for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;
            end
            crc16=c;
        end
    endfunction
    always @(posedge clk)begin
        if(!rst_n)stalled<=0;
        else begin
            if(stalled && valid && held!==frame)$fatal(1,"raw frame changed while blocked");
            stalled<=valid && !ready;held<=frame;
            if(valid && ready)begin
                if(frame[383:368]!==crc16(frame[367:0]))$fatal(1,"raw CRC mismatch");
                $fwrite(log,"%0d %096h\n",phase,frame);
                if(frame[31:0]==32'h01073cc3)begin
                    raw_frames=raw_frames+1;
                    if(frame[79:64]>=400 || frame[119:112]>1)$fatal(1,"raw metadata");
                end else if(frame[31:0]!=32'h03063cc3)$fatal(1,"unknown raw source frame");
            end
        end
    end
    task reset_case;begin
        @(negedge clk);rst_n=0;pv=0;ready=1;packets=0;errors=0;
        repeat(4)@(negedge clk);rst_n=1;repeat(4)@(negedge clk);raw_frames=0;
    end endtask
    task packet(input integer seq,input integer idle_cycles);integer i,a,d,q;begin
        for(i=0;i<12;i=i+1)begin
            @(negedge clk);packets=seq;index=i;pv=1;
            a=(((seq-1)*12+i)*100+37)%36000;
            if(seq%30==5 && i==2)a=(((seq-1)*12+i-1)*100+37)%36000;
            d=(seq%30==6 && i==2)?0:1000+(seq-1)*12+i;
            q=(seq%30==7 && i==3)?0:50+i;
            angle=a;distance=d;intensity=q;
        end
        @(negedge clk);pv=0;repeat(idle_cycles)@(negedge clk);
    end endtask
    initial begin
        log=$fopen("build/lidar_raw_ethernet_20261102/points_frames.hex","w");
        reset_case();phase=1;
        for(n=1;n<=91;n=n+1)packet(n,1000);
        repeat(2000)@(negedge clk);
        if(raw_frames!=182 || dut.dropped!=0 || dut.forwarded!=91)$fatal(1,"nominal all-point forwarding");
        $display("PASS raw points preserve duplicates, zero returns and every native packet");
        reset_case();phase=2;ready=0;
        for(n=1;n<=20;n=n+1)packet(n,1000);
        if(dut.dropped==0 || dut.queue_count!=8)$fatal(1,"raw FIFO overload was not detected");
        ready=1;repeat(10000)@(negedge clk);packet(21,1000);
        repeat(2000)@(negedge clk);
        if(dut.forwarded+dut.dropped!=21)$fatal(1,"raw FIFO packet accounting");
        $display("PASS raw FIFO overload, packet sequence gaps and recovery");
        reset_case();phase=3;ready=0;packet(1,1000);packet(2,1000);
        repeat(420000)@(negedge clk);ready=1;repeat(5000)@(negedge clk);
        if(raw_frames!=0 || dut.dropped!=2)$fatal(1,"stale raw packet was revived");
        packet(3,1000);repeat(1000)@(negedge clk);
        if(raw_frames!=2)$fatal(1,"stale raw stream failed to recover");
        $display("PASS source expiry while stalled and clean-packet recovery");
        reset_case();phase=4;packet(1,1000);errors=1;packet(2,1000);
        repeat(1000)@(negedge clk);
        if(dut.dropped!=0 || raw_frames!=4)$fatal(1,"parser epoch forwarding");
        $fclose(log);$display("PASS raw point stream RAM regression");$finish;
    end
    initial begin #15000000;$fatal(1,"raw source regression timeout");end
endmodule
