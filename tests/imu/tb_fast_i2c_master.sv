`timescale 1ns/1ps
module tb_fast_i2c_master;
    reg clk=0,rst_n=0,req=0,rd=1,present=1;
    always #10 clk=~clk; // actual 50 MHz, Fast-mode Plus bus
    wire scl,sda;
    reg hold_scl=0,hold_sda=0;
    // Icarus does not implement delayed tri nets. Weak delayed pull drivers
    // retain low for 100ns after all strong open-drain sinks release.
    assign (pull1,pull0) #(100,0) scl=!(dut.scl_low || hold_scl);
    assign (pull1,pull0) #(100,0) sda=!(dut.sda_low || sensor.drive_low || hold_sda);
    assign scl=hold_scl ? 1'b0:1'bz;
    assign sda=hold_sda ? 1'b0:1'bz;
    reg [6:0] device=7'h69;
    reg [7:0] address=8'h75,wr=0;
    reg [5:0] length=1;
    wire busy,done;
    wire [3:0] error;
    wire [255:0] data;
    imu_fast_i2c_master #(.CLK_HZ(50000000),.I2C_HZ(1000000),.TIMEOUT_US(6000)) dut(
        .clk(clk),.rst_n(rst_n),.req(req),.read_not_write(rd),.device_addr(device),
        .reg_addr(address),.write_data(wr),.read_length(length),.busy(busy),.done(done),
        .error(error),.read_data(data),.scl(scl),.sda(sda));
    icm42688_model sensor(.scl(scl),.sda(sda),.present(present),.identity(8'h47),
        .corrupt_readback(1'b0),.invalid_data(1'b0),.freeze_samples(1'b1));
    integer checks=0,clear_edges=0;
    time previous_rise=0,previous_fall=0,low_width,high_width;
    reg check_timing=0;
    always @(posedge scl) begin
        if(check_timing && previous_fall!=0) begin
            low_width=$time-previous_fall;
            if(low_width<500) $fatal(1,"SCL low too short %0t",low_width);
        end
        previous_rise=$time;
    end
    always @(negedge scl) begin
        if(check_timing && previous_rise!=0) begin
            high_width=$time-previous_rise;
            if(high_width<260) $fatal(1,"SCL high too short %0t",high_width);
        end
        previous_fall=$time;
    end
    task transaction;
        input read_mode;
        input [7:0] regnum,write_byte;
        input [5:0] count;
        input [3:0] expected_error;
        begin
            @(negedge clk); rd=read_mode; address=regnum; wr=write_byte; length=count; req=1;
            @(negedge clk); req=0;
            wait(done); #1;
            if(error!==expected_error) $fatal(1,"error expected %0d got %0d",expected_error,error);
            if(busy) $fatal(1,"busy at done");
            checks=checks+1; repeat(5) @(negedge clk);
        end
    endtask
    initial begin
        repeat(8) @(negedge clk); rst_n=1; repeat(8) @(negedge clk); check_timing=1;
        transaction(1,8'h75,0,1,0); if(data[7:0]!==8'h47) $fatal(1,"WHO mismatch");
        transaction(0,8'h4f,8'h27,1,0);
        transaction(1,8'h4f,0,1,0); if(data[7:0]!==8'h27) $fatal(1,"write/read mismatch");
        transaction(1,8'h1d,0,14,0);
        if(data[111:0]!==112'h00840400f8002000800103e87fff) $fatal(1,"14-byte signed data %h",data[111:0]);
        if(sensor.burst_length!=14) $fatal(1,"burst interrupted");
        transaction(1,8'h00,0,32,0); if(sensor.burst_length!=32) $fatal(1,"32-byte extension failed");
        device=7'h68; transaction(1,8'h75,0,1,1); device=7'h69;
        fork
            transaction(0,8'h4f,8'h00,1,1);
            begin wait(sensor.state==1 && !sensor.ack_phase); @(negedge clk); present=0; end
        join
        present=1;
        fork
            transaction(0,8'h4f,8'h00,1,1);
            begin wait(sensor.state==2 && !sensor.ack_phase); @(negedge clk); present=0; end
        join
        present=1;
        transaction(1,8'h4f,0,1,0); if(data[7:0]!==8'h27) $fatal(1,"NACKed write committed");
        transaction(1,8'h75,0,0,4);
        transaction(1,8'h75,0,33,4);
        // Clock stretching during an actual high phase must not advance bits.
        fork
            transaction(1,8'h75,0,1,0);
            begin wait(dut.state==5 && dut.phase==1); hold_scl=1; #30000; hold_scl=0; end
        join
        if(data[7:0]!==8'h47) $fatal(1,"stretch corrupted data");
        check_timing=0;
        hold_sda=1;
        fork
            transaction(1,8'h75,0,1,0);
            begin
                wait(dut.state==21);
                repeat(9) @(posedge scl); hold_sda=0;
            end
        join
        if(data[7:0]!==8'h47) $fatal(1,"clear did not resume transaction");
        hold_sda=1; transaction(1,8'h75,0,1,3); hold_sda=0;
        hold_scl=1; transaction(1,8'h75,0,1,2); hold_scl=0;
        repeat(10) @(negedge clk);
        transaction(1,8'h75,0,1,0);
        // Abort a read asynchronously; next request must recover the slave bus.
        @(negedge clk); rd=1; address=8'h1d; length=14; req=1;
        @(negedge clk); req=0;
        wait(dut.state==12 && dut.phase==1); @(negedge clk); rst_n=0;
        #1; if(dut.scl_low || dut.sda_low || busy) $fatal(1,"reset failed to release master");
        repeat(8) @(negedge clk); rst_n=1; repeat(8) @(negedge clk);
        transaction(1,8'h75,0,1,0);
        if(data[7:0]!==8'h47) $fatal(1,"reset-abort recovery failed");
        if(sensor.master_nacks<6 || sensor.master_acks<44) $fatal(1,"ACK/NACK coverage missing");
        $display("PASS bus: %0d transactions, bursts 1/14/32, address/register/data NACK, stretching, clear, timeout, reset-abort recovery",checks);
        $finish;
    end
    initial begin #40000000; $fatal(1,"bus test timeout"); end
endmodule
