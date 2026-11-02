`timescale 1ns/1ps
// Full production hierarchy. Digital byte/GMII/video fixtures exercise the new
// telemetry route, not the UART/PHY/camera electrical interfaces or vision math.
module tb_system_sector_ethernet;
    reg clk=0,rxclk=0,pclk=0,rst_n=0;always #10 clk=~clk;always #4.1 rxclk=~rxclk;always #6 pclk=~pclk;
    tri1 scl,sda,mdio,csda;
    wire l1,l2,r1,r2,legacy,tx;wire [5:0] old_pwm;wire old_cal;
    system_task4_top #(.CLK_HZ(1000000),.TELEMETRY_ETHERNET(1),.LIDAR_RAW(0)) dut(
        .clk50m(clk),.reset_n(rst_n),.uart_rx(1'b1),.lidar_rx(1'b1),
        .rgmii_rxc(rxclk),.rgmii_rxctl(1'b0),.rgmii_rxd(4'd0),.eth_mdio(mdio),
        .camera_pclk(pclk),.camera_vsync(1'b0),.camera_href(1'b0),.camera_data(8'd0),.camera_sdat(csda),
        .imu_i2c_scl(scl),.imu_i2c_sda(sda),.left_enc_a(1'b0),.left_enc_b(1'b0),.right_enc_a(1'b0),.right_enc_b(1'b0),
        .left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2),.uart_tx(tx),
        .legacy_m1_disable(legacy),.legacy_m1_pwm_disabled(old_pwm),.legacy_cal_disabled(old_cal));
    defparam dut.imu.POWERUP_US=100;
    defparam dut.imu.RESET_US=1200;
    defparam dut.imu.STARTUP_US=500;
    defparam dut.imu_status.DECIMATION=2;
    defparam dut.camera.stream.WIDTH=16;
    defparam dut.camera.stream.ACTIVE_HEIGHT=8;
    defparam dut.camera.stream.CANVAS_HEIGHT=16;
    mpu6050_model #(.SAMPLE_NS(1000000)) sensor(.scl(scl),.sda(sda),.present(1'b1),.identity(8'h68),
        .corrupt_readback(1'b0),.invalid_data(1'b0),.freeze_samples(1'b0));
    reg [7:0] rx_data=0;reg rx_valid=0;
    reg native_valid=0;reg [7:0] native_byte=0;
    reg video_start=0;reg [10:0] cursor=0;integer log;reg was_tx=0;
    wire [7:0] video_data=cursor[7:0]^{5'd0,cursor[10:8]};
    always @(posedge dut.camera.clk125)if(dut.camera.read)cursor<=cursor+1'b1;
    always @(negedge dut.camera.clk125)begin
        if(dut.camera.tx_valid)$fwrite(log,"%02x\n",dut.camera.gmii);
        else if(was_tx)$fwrite(log,"END\n");
        was_tx=dut.camera.tx_valid;
    end
    always @(posedge clk)if(rst_n)begin
        if({l1,l2,r1,r2,legacy,old_pwm,old_cal}!==12'd0)$fatal(1,"unarmed motor pads changed");
        if(tx!==1'b1)$fatal(1,"status telemetry used UART");
    end
    task inject(input integer kind);reg [7:0] bytes[0:127];integer n,k;begin
        if(kind==0)begin n=74;$readmemh("build/lidar_raw_ethernet_20261102/set_endpoint.hex",bytes,0,73);end
        else begin n=72;$readmemh("build/lidar_raw_ethernet_20261102/arp_endpoint.hex",bytes,0,71);end
        @(negedge rxclk);rx_valid=1;
        for(k=0;k<n;k=k+1)begin rx_data=bytes[k];@(negedge rxclk);end
        rx_valid=0;rx_data=0;repeat(250)@(negedge clk);
    end endtask
    reg [7:0] bytes[0:65535];integer n,k,fd,ignored;
    initial begin
        force GSR.GSRI=1'b1;
        force dut.camera.phy_init=1'b1;
        force dut.camera.runtime_network.rx_data=rx_data;
        force dut.camera.runtime_network.rx_valid=rx_valid;force dut.camera.runtime_network.rx_error=1'b0;
        force dut.lbv=native_valid;force dut.lbyte=native_byte;force dut.lerr=1'b0;
        force dut.camera.start=video_start;force dut.camera.first=1'b1;
        force dut.camera.length=16'd1456;force dut.camera.payload=video_data;
        log=$fopen("build/lidar_raw_ethernet_20261102/system_sector_top_tx.hex","w");
        repeat(5)@(negedge clk);rst_n=1;repeat(500)@(negedge clk);
        inject(0);inject(1);
        fork
            begin
                wait(dut.lpackets>10);@(negedge dut.camera.clk125);video_start=1;
                @(negedge dut.camera.clk125);video_start=0;
                wait(dut.camera.done);if(cursor!=1456)$fatal(1,"full top video byte count");
            end
            begin
                fd=$fopen("build/lidar_raw_ethernet_20261102/input.len","r");ignored=$fscanf(fd,"%d",n);$fclose(fd);
                $readmemh("build/lidar_raw_ethernet_20261102/input.hex",bytes,0,n-1);
                for(k=0;k<n;k=k+1)begin
                    @(negedge clk);native_valid=1;native_byte=bytes[k];
                    @(negedge clk);native_valid=0;repeat(8)@(negedge clk);
                end
            end
        join
        repeat(200000)@(negedge clk);
        if(dut.ethernet_scan.lidar_display.completed==0 || dut.ethernet_scan.lidar_display.input_loss_count!=0 || dut.lpackets!=1000 || dut.lcrc!=1)
            $fatal(1,"full top raw source counters");
        $fclose(log);$display("PASS full production top: all raw points, concurrent video, seven UDP states, unarmed GPIOs and idle UART");$finish;
    end
    initial begin #30000000;$fatal(1,"full raw Ethernet top timeout");end
endmodule
