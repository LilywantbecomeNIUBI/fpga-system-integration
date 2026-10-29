`timescale 1ns/1ps
module tb_uart_system;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,rx=1;
    wire l1,l2,r1,r2,tx,m1,m2;
    always @(posedge clk) begin
        if(dut.legacy_m1_disable!==1'b0 || dut.legacy_m2_disable!==1'b0 ||
           dut.legacy_m1_pwm_disabled!==6'd0 || dut.legacy_m2_pwm_disabled!==6'd0 || dut.legacy_cal_disabled!==2'd0)
            $fatal(1,"Old BLDC enables/PWM/CAL must stay low in all states");
    end
    wire [7:0] led;
    brushed_motor_top #(.CLK_HZ(1000000),.BAUD(100000)) dut(
        .clk50m(clk),.reset_n(reset_n),.uart_rx(rx),
        .left_enc_a(1'b0),.left_enc_b(1'b0),.right_enc_a(1'b0),.right_enc_b(1'b0),
        .left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2),.uart_tx(tx),
        .legacy_m1_disable(m1),.legacy_m2_disable(m2),.led(led));
    wire [7:0] received;
    wire valid,frame_error;
    uart_rx_byte #(.CLK_HZ(1000000),.BAUD(100000)) monitor(
        .clk(clk),.reset_n(reset_n),.rx(tx),.data(received),.valid(valid),.framing_error(frame_error));
    reg [7:0] frame[0:15];
    reg [7:0] status[0:47];
    reg [15:0] status_crc=16'hffff;
    integer count=0,packets=0;
    function automatic [15:0] crc_byte(input [15:0] old,input [7:0] value);
        reg [15:0] c;integer j;
        begin c=old^{value,8'b0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
    endfunction
    always @(posedge clk) begin
        if(valid) begin
            status[count]=received;
            if(count<46) status_crc=crc_byte(status_crc,received);
            if(count==47) begin
                if(status[0]!=8'hc3 || status[1]!=8'h3c || status[2]!=1 || {status[47],status[46]}!=status_crc) $fatal(1,"telemetry frame");
                packets=packets+1;count=0;status_crc=16'hffff;
            end else count=count+1;
        end
        if(frame_error || m1 || m2) $fatal(1,"UART/legacy/output");
        if(!dut.core.armed && (l1||l2||r1||r2)) $fatal(1,"unarmed output");
    end
    task byte_send(input [7:0] value);integer i;begin
        @(negedge clk);rx=0;repeat(10) @(negedge clk);
        for(i=0;i<8;i=i+1)begin rx=value[i];repeat(10) @(negedge clk);end
        rx=1;repeat(10) @(negedge clk);
    end endtask
    task send(input [7:0] op,s,input signed [15:0] l,r,input corrupt);
        integer i;reg [15:0] crc;
        begin
            frame[0]=8'ha5;frame[1]=8'h5a;frame[2]=1;frame[3]=op;frame[4]=s;frame[5]=1;
            frame[6]=l[7:0];frame[7]=l[15:8];frame[8]=r[7:0];frame[9]=r[15:8];
            frame[10]=0;frame[11]=1;frame[12]=0;frame[13]=0;crc=16'hffff;
            for(i=0;i<14;i=i+1)crc=crc_byte(crc,frame[i]);
            frame[14]=crc[7:0];frame[15]=crc[15:8]^{7'b0,corrupt};
            for(i=0;i<16;i=i+1)byte_send(frame[i]);
            repeat(30) @(negedge clk);
        end
    endtask
    initial begin
        #1;repeat(3) @(negedge clk);reset_n=1;repeat(20) @(negedge clk);
        byte_send(8'ha5);byte_send(8'h5a);byte_send(1); // truncated frame
        repeat(21000) @(negedge clk);
        send(3,1,0,0,0);send(1,2,0,0,0);
        if(!dut.armed) $fatal(1,"UART arm");
        send(2,3,80,-60,1);if(dut.last_sequence!=2) $fatal(1,"corrupt CRC accepted");
        send(2,3,80,-60,0);repeat(150000) @(negedge clk);
        if(dut.left_applied!=80 || dut.right_applied!=-60) $fatal(1,"UART target path");
        if(packets<1 || status[4]!=1 || status[3]!=3 || dut.bad_frames<2) $fatal(1,"status snapshot/resync");
        send(2,3,100,100,0);if(dut.rejected_commands==0) $fatal(1,"UART replay");
        send(0,3,0,0,0);repeat(10) @(negedge clk);
        if(dut.armed||l1||l2||r1||r2) $fatal(1,"UART stop");
        $display("PASS UART system: real serial commands, CRC rejection, partial timeout, telemetry CRC, replay, STOP");$finish;
    end
    initial begin #3000000;$fatal(1,"timeout");end
endmodule
