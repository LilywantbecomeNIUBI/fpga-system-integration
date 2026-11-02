`timescale 1ns/1ps
// Accelerated device milliseconds; the RAM/CRC still use real single clocks.
module tb_lidar_scan_ram;
    reg clk=0,rst_n=0,pv=0,ready=1;
    always #10 clk=~clk;
    reg [15:0] angle=0,distance=0;
    reg [7:0] intensity=0;
    reg [31:0] errors=0;
    wire [383:0] frame;
    wire valid;
    lidar_scan_source #(.CLK_HZ(100000)) dut(.clk(clk),.rst_n(rst_n),
        .byte_valid(pv),.framing_error(1'b0),.point_valid(pv),.angle(angle),
        .distance(distance),.intensity(intensity),.packets(32'd42),.errors(errors),
        .crc_errors(errors),.format_errors(32'd0),.timeouts(32'd0),
        .ready(ready),.frame(frame),.valid(valid));

    reg [23:0] expected [0:71];
    integer scan_frames=0,diag_frames=0,next_chunk=0,expected_sequence=1;
    integer j,bin_number;
    reg [383:0] held;
    reg stalled=0;
    reg [15:0] c;
    function automatic [15:0] crc16(input [367:0] value);
        reg [15:0] v;integer b,k;
        begin
            v=16'hffff;
            for(b=0;b<46;b=b+1)begin
                v=v^{value[b*8+:8],8'd0};
                for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;
            end
            crc16=v;
        end
    endfunction
    always @(posedge clk)begin
        if(!rst_n)stalled<=0;
        else begin
            if(stalled && valid && frame!==held)$fatal(1,"frame changed under backpressure");
            stalled<=valid && !ready;held<=frame;
            if(valid && ready)begin
                c=crc16(frame[367:0]);
                if(frame[383:368]!==c)$fatal(1,"frame CRC mismatch");
                if(frame[31:0]==32'h01053cc3)begin
                    if(frame[47:32]!=expected_sequence || frame[55:48]!=next_chunk ||
                            frame[63:56]!=6 || frame[79:64]>=400)
                        $fatal(1,"scan header/order mismatch seq=%d chunk=%d age=%d",
                            frame[47:32],frame[55:48],frame[79:64]);
                    for(j=0;j<12;j=j+1)begin
                        bin_number=next_chunk*12+j;
                        if(frame[80+j*24+:24]!==expected[bin_number])
                            $fatal(1,"bin %d got %h expected %h",bin_number,
                                frame[80+j*24+:24],expected[bin_number]);
                    end
                    scan_frames=scan_frames+1;next_chunk=next_chunk+1;
                    if(next_chunk==6)begin next_chunk=0;expected_sequence=expected_sequence+1;end
                end else if(frame[31:0]==32'h01063cc3)begin
                    if(frame[63:56]!=0 || frame[55:53]!=0 || frame[367:352]>72)
                        $fatal(1,"diagnostic layout changed");
                    diag_frames=diag_frames+1;
                end else $fatal(1,"unknown protocol frame");
            end
        end
    end
    task push(input integer a,d,q);begin
        @(negedge clk);pv=1;angle=a;distance=d;intensity=q;
    end endtask
    task gap(input integer cycles);begin
        @(negedge clk);pv=0;
        repeat(cycles)@(negedge clk);
    end endtask
    task settle;begin
        gap(0);
        wait(dut.fifo_count==0 && dut.point_state==0 && !dut.clear_active);
        @(negedge clk);
    end endtask
    task reset_case;begin
        @(negedge clk);rst_n=0;pv=0;ready=1;errors=0;
        repeat(4)@(negedge clk);
        scan_frames=0;diag_frames=0;next_chunk=0;expected_sequence=1;
        rst_n=1;repeat(80)@(negedge clk);
    end endtask
    task expect_seed(input integer seed);integer n;begin
        for(n=0;n<72;n=n+1)expected[n]={8'd50,16'(seed+n)};
    end endtask
    task turn(input integer seed);integer n;begin
        for(n=0;n<72;n=n+1)begin push(n*500,seed+n,50);gap(7);end
        settle();
    end endtask
    task publish_seed(input integer seed);begin
        push(35500,1,1);settle();
        turn(seed);push(0,seed+1000,50);settle();
    end endtask

    integer n,before_frames;
    reg [31:0] published_ms;
    initial begin
        // A burst straddles the initial clear. Invalid returns count toward
        // coverage, a nearer valid return replaces, and ties retain intensity.
        reset_case();expect_seed(1000);
        expected[0]={8'd20,16'd1000};expected[71]=0;
        push(35500,99,1);push(0,1500,10);push(0,1000,20);
        push(0,1000,30);push(0,500,0);push(0,0,40);push(0,1100,40);
        for(n=1;n<=12;n=n+1)push(n*500,1000+n,50);
        settle();
        for(n=13;n<71;n=n+1)begin push(n*500,1000+n,50);gap(7);end
        push(35500,0,50);settle();push(36000,1,1);settle();push(0,2000,50);settle();
        wait(scan_frames==6);gap(10);
        if(dut.completed!=1 || dut.last_coverage!=72 || dut.input_loss_count!=0)
            $fatal(1,"nearest/coverage/burst counters wrong");
        $display("PASS nearest returns, equal-distance tie, invalid returns, 72 bins, burst and CRC");

        // Leave an entire snapshot blocked while two subsequent turns reuse
        // the writer. Repeated swaps here would corrupt the protected reader.
        reset_case();expect_seed(2000);ready=0;publish_seed(2000);
        wait(valid);published_ms=dut.snapshot_ms;
        for(n=1;n<72;n=n+1)begin push(n*500,3000+n,50);gap(7);end
        push(0,4000,50);settle();
        for(n=1;n<72;n=n+1)begin push(n*500,4000+n,50);gap(7);end
        push(0,5000,50);settle();
        if(dut.snapshot_ms!=published_ms || dut.completed!=3 || dut.skipped!=2)
            $fatal(1,"blocked snapshot ownership/rate limiting wrong");
        ready=1;wait(scan_frames==6);gap(10);
        // Publish again after the 200 ms rate limit, with an invalid previous
        // turn, to check both bank directions and invalidation by a large gap.
        wait(dut.now_ms-published_ms>=200);
        push(35500,1,1);settle();
        expect_seed(6000);turn(6000);push(0,7000,50);settle();
        wait(scan_frames==12);gap(10);
        if(dut.scan_sequence!=2)$fatal(1,"second bank did not publish");
        $display("PASS blocked snapshot, skipped turns, 5 Hz limit and both bank directions");

        // At 399 ms, start another chunk CRC so checked_valid arrives AFTER
        // expiry. Consume the 200 ms diagnostic first to isolate that race.
        reset_case();expect_seed(8000);ready=0;publish_seed(8000);
        wait(valid);wait(dut.age>=200);@(negedge clk);ready=1;
        wait(scan_frames==1 && diag_frames==1);@(negedge clk);ready=0;
        wait(valid && frame[31:0]==32'h01053cc3);
        wait(dut.age==399 && dut.ms_div==60);@(negedge clk);ready=1;
        wait(scan_frames==2);@(negedge clk);ready=0;
        wait(!dut.transmitting);gap(120);
        if(valid && frame[31:0]==32'h01053cc3)$fatal(1,"late CRC revived an expired scan");
        if(dut.pack_state==2)$fatal(1,"expired CRC did not drain");
        ready=1;gap(100);before_frames=scan_frames;
        if(before_frames!=2)$fatal(1,"expired scan emitted extra chunks");
        // Recover after the 250 ms input timeout; partial expired scans do not
        // require the next published sequence to start at the old chunk index.
        next_chunk=0;expected_sequence=2;expect_seed(9000);
        publish_seed(9000);wait(scan_frames==8);gap(10);
        $display("PASS expiry while CRC is in flight, stale discard, timeout and recovery");

        // No ready pin exists on input. Deliberate overload must invalidate
        // the affected revolution, and a clean later revolution must recover.
        reset_case();expect_seed(10000);
        push(35500,1,1);push(0,10000,50);
        for(n=0;n<80;n=n+1)push(0,10000,50);
        settle();
        for(n=1;n<72;n=n+1)begin push(n*500,10000+n,50);gap(7);end
        push(0,10000,50);settle();gap(100);
        if(dut.input_loss_count==0 || dut.completed!=0 || scan_frames!=0)
            $fatal(1,"FIFO overflow published a corrupt scan");
        for(n=1;n<72;n=n+1)begin push(n*500,10000+n,50);gap(7);end
        push(0,10000,50);settle();wait(scan_frames==6);
        $display("PASS overload rejection and subsequent clean scan recovery");

        // Parser errors and sparse coverage must still reject a revolution.
        // Reset with a blocked old scan also checks RAM is cleared by walking,
        // rather than relying on power-on contents or a whole-RAM reset.
        reset_case();ready=0;publish_seed(11000);wait(valid);
        reset_case();expect_seed(12000);
        push(35500,1,1);settle();
        for(n=0;n<72;n=n+1)begin
            if(n%6==5)expected[n]=0;
            else begin push(n*500+499,12000+n,50);gap(7);end
        end
        push(499,13000,50);settle();wait(scan_frames==6);gap(10);
        if(dut.last_coverage!=60)$fatal(1,"60-bin acceptance boundary changed");
        $display("PASS 60-bin boundary, angle-bin upper edges, cleared holes after reset");
        reset_case();expect_seed(12000);
        push(35500,1,1);settle();
        for(n=0;n<72;n=n+1)begin
            if(n==30)errors=1;
            push(n*500,12000+n,50);gap(7);
        end
        push(0,12000,50);settle();
        if(dut.completed!=0)$fatal(1,"error epoch did not reject scan");
        for(n=2;n<72;n=n+2)begin push(n*500,12000+n,50);gap(7);end
        push(0,12000,50);settle();
        if(dut.completed!=0)$fatal(1,"36-bin sparse scan was accepted");
        for(n=1;n<72;n=n+1)begin push(n*500,12000+n,50);gap(7);end
        push(0,12000,50);settle();wait(scan_frames==6);gap(10);
        $display("PASS parser-error epoch, coverage rejection and reset during blocked scan");
        $display("PASS lidar scan RAM regression");$finish;
    end
    initial begin #10000000;$fatal(1,"scan RAM regression timeout");end
endmodule
