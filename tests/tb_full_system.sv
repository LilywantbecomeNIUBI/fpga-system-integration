`timescale 1ns/1ps
// Full production hierarchy, behavioral sensor/PLL, reduced frame buffer and
// post-pixel injection. Camera SCCB/DVP electrical behavior is not modeled.
module tb_full_system;
    reg clk=0,reset_n=0,pclk=0,rxc=0,estop=1;
    always #10 clk=~clk;always #6 pclk=~pclk;always #4 rxc=~rxc;
    tri1 scl,sda,csda,mdio;
    wire tx,l1,l2,r1,r2,m1,m2;
    // Accelerate motor/UART counters by 10x; sensor and camera/network clocks stay unchanged.
    system_top #(.CLK_HZ(5000000)) dut(.clk50m(clk),.reset_n(reset_n),.estop_n(estop),.uart_rx(1'b1),
        .left_enc_a(1'b0),.left_enc_b(1'b0),.right_enc_a(1'b0),.right_enc_b(1'b0),
        .rgmii_rxc(rxc),.rgmii_rxd(4'd0),.rgmii_rxctl(1'b0),.camera_pclk(pclk),.camera_vsync(1'b0),.camera_href(1'b0),.camera_data(8'd0),
        .camera_sdat(csda),.eth_mdio(mdio),.imu_i2c_scl(scl),.imu_i2c_sda(sda),.uart_tx(tx),
        .left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2),.legacy_m1_disable(m1),.legacy_m2_disable(m2));
    defparam dut.imu.POWERUP_US=100;
    defparam dut.imu.RESET_US=1200;
    defparam dut.imu.STARTUP_US=500;
    defparam dut.camera.stream.WIDTH=16;
    defparam dut.camera.stream.ACTIVE_HEIGHT=8;
    defparam dut.camera.stream.CANVAS_HEIGHT=16;
    reg present=1;
    icm42688_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(present),.identity(8'h47),.corrupt_readback(1'b0),.invalid_data(1'b0),.freeze_samples(1'b0));
    reg cmd=0;reg [7:0] op=0,seq=0;reg signed [15:0] target=0;
    reg os=0,oe=0,ov=0;reg [15:0] pixel=0,fid=0;
    integer samples=0,control_ticks=0,eth_bytes=0,moving_bytes=0,bytes=0,file,i,j;
    wire [7:0] byte_data;wire byte_valid,framing_error;
    uart_rx_byte #(.CLK_HZ(5000000)) monitor(.clk(clk),.reset_n(reset_n),.rx(tx),.data(byte_data),.valid(byte_valid),.framing_error(framing_error));
    task command(input [7:0] code,input signed [15:0] value);begin
        @(negedge clk);op=code;seq=seq+1;target=value;cmd=1;@(negedge clk);cmd=0;repeat(5)@(negedge clk);
    end endtask
    task image;begin
        @(negedge pclk);os=1;fid=fid+1;@(negedge pclk);os=0;
        for(j=0;j<128;j=j+1)begin ov=1;pixel=j;@(negedge pclk);end
        ov=0;oe=1;@(negedge pclk);oe=0;repeat(500)@(negedge pclk);
    end endtask
    always @(posedge clk)begin
        if(dut.imu_sample_valid)samples=samples+1;
        if(dut.core.control_tick)control_ticks=control_ticks+1;
        if(byte_valid)begin $fwrite(file,"%c",byte_data);bytes=bytes+1;end
        if(reset_n && (m1||m2||framing_error))$fatal(1,"legacy/UART fault");
    end
    always @(posedge dut.camera.clk125)if(dut.camera.tx_valid)begin eth_bytes=eth_bytes+1;if(dut.left_applied!=0)moving_bytes=moving_bytes+1;end
    initial begin
        force GSR.GSRI=1'b1;
        force dut.command_valid=cmd;force dut.opcode=op;force dut.sequence_id=seq;
        force dut.command_flags=0;force dut.left_target=target;force dut.right_target=target;
        force dut.kp=256;force dut.ki=0;
        file=$fopen("build/tests/full_system_uart.bin","wb");repeat(50)@(negedge clk);reset_n=1;
        wait(samples>=4);command(3,0);command(1,0);command(2,20);
        force dut.camera.cam_done=1;force dut.camera.phy_init=1;
        force dut.camera.os=os;force dut.camera.oe=oe;force dut.camera.ov=ov;force dut.camera.op=pixel;force dut.camera.ofid=fid;force dut.camera.og=1;
        force dut.camera.runtime_network.net.enabled=1;
        force dut.camera.runtime_network.net.target_ip=32'hc0a8320a;
        force dut.camera.runtime_network.net.lookup_valid=1;
        force dut.camera.runtime_network.net.found_mac=48'h001122334455;
        force dut.camera.runtime_network.net.wait_first=0;
        repeat(200)@(negedge clk);
        for(i=0;i<400;i=i+1)image();
        if(eth_bytes<500 || moving_bytes<500)$fatal(1,"no concurrent traffic %d moving %d",eth_bytes,moving_bytes);
        force pclk=0;force dut.camera.phy_init=0;
        #2000000;
        if(!dut.armed || dut.imu_fault || !dut.imu_ready)$fatal(1,"network/camera reset coupled to motor or IMU");
        force dut.camera.phy_init=1;
        #6000000;
        if(!dut.armed || dut.left_applied!=20 || samples<8 || control_ticks<6 || bytes<48*2)$fatal(1,"concurrent operation");
        present=0;wait(dut.imu_fault || dut.imu_stale);
        if(!dut.armed)$fatal(1,"observation-only IMU failure disarmed manual drive");
        #1;estop=0;#1;if(l1||l2||r1||r2)$fatal(1,"estop output");
        $fclose(file);$display("PASS full production top: IMU=%0d control_ticks=%0d Ethernet_bytes=%0d camera stop/network reset/IMU fault isolated",samples,control_ticks,eth_bytes);$finish;
    end
    initial begin #100000000;$fatal(1,"full system timeout");end
    initial forever begin #1000000;$display("PROGRESS time=%0t samples=%0d bytes=%0d",$time,samples,bytes);end
endmodule
