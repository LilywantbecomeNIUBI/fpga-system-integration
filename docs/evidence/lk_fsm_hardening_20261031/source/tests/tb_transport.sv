`timescale 1ns/1ps
module tb_transport;
    reg clk=0;always #5 clk=~clk;
    reg reset_n=0,sample=0;
    reg [31:0] seq=0;
    wire [383:0] motor_frame,imu_frame;
    wire mv,iv,tx;
    wire [15:0] drops;
    motor_status_source #(.CLK_HZ(1000000)) motor(.clk(clk),.reset_n(reset_n),.armed(1'b0),.flags(8'd1),.last_sequence(8'd0),.faults(16'd1),
        .left_position(32'sd123),.right_position(-32'sd456),.left_velocity(32'sd7),.right_velocity(-32'sd8),.left_applied(16'sd0),.right_applied(16'sd0),
        .left_invalid(32'd0),.right_invalid(32'd0),.bad_frames(32'd0),.rejected_commands(32'd0),.frame(motor_frame),.valid(mv));
    imu_status_source #(.CLK_HZ(1000000)) imu(.clk(clk),.reset_n(reset_n),.sample_valid(sample),.ready(1'b1),.fault(1'b0),.sample_stale(1'b0),.temperature_valid(1'b1),
        .ax(16'sd0),.ay(16'sd0),.az(16'sd8192),.gx(16'sd0),.gy(16'sd0),.gz(16'sd0),.temperature_raw(16'sd0),
        .sample_seq(seq),.timestamp_us(seq*32'd1000),.now_us(seq*32'd1000),.who_am_i(8'h47),.last_error(8'd0),
        .error_count(16'd0),.invalid_count(16'd0),.observe_to_commit_us(16'd145),.sample_interval_us(16'd1000),.drop_count(drops),.frame(imu_frame),.valid(iv));
    frame_uart_mux #(.CLK_HZ(1000000),.BAUD(100000)) dut(.clk(clk),.reset_n(reset_n),.motor_valid(mv),.motor_frame(motor_frame),.imu_valid(iv),.imu_frame(imu_frame),.tx(tx),.imu_drops(drops));
    wire [7:0] byte_data;wire byte_valid,error;
    uart_rx_byte #(.CLK_HZ(1000000),.BAUD(100000)) monitor(.clk(clk),.reset_n(reset_n),.rx(tx),.data(byte_data),.valid(byte_valid),.framing_error(error));
    integer i,j,file,bytes=0;
    always @(posedge clk)begin
        if(byte_valid)begin $fwrite(file,"%c",byte_data);bytes=bytes+1;end
        if(error)$fatal(1,"UART framing");
    end
    initial begin
        file=$fopen("build/tests/mixed_uart.bin","wb");repeat(3)@(negedge clk);reset_n=1;
        // 150ms nominal acquisition, 100Hz IMU + 20Hz motor.
        for(i=0;i<150;i=i+1)begin sample=1;seq=seq+1;@(negedge clk);sample=0;repeat(999)@(negedge clk);end
        if(drops!=0)$fatal(1,"nominal IMU drop");
        // Overload IMU deliberately: queues must overwrite snapshots, never interleave packets.
        for(i=0;i<1000;i=i+1)begin sample=1;seq=seq+1;@(negedge clk);sample=0;repeat(9)@(negedge clk);end
        repeat(12000)@(negedge clk);
        if(drops==0 || bytes<48*10)$fatal(1,"congestion not exercised");
        $fclose(file);$display("PASS transport: nominal rates, congestion=%0d, whole-frame UART",drops);$finish;
    end
    initial begin #3000000;$fatal(1,"timeout");end
endmodule
