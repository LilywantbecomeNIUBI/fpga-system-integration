`timescale 1ns/1ps
// Production sensor-only top. PHY readiness and recovered GMII RX are digital
// fixtures; no claim is made about physical DDR, negotiation or signal integrity.
module tb_m2_sensor_ethernet;
    reg clk=0,rxclk=0,rst_n=0;always #10 clk=~clk;always #4.1 rxclk=~rxclk;
    tri1 scl,sda,mdio;
    wire l1,l2,r1,r2,legacy,tx;wire [5:0] old_pwm;wire old_cal;
    m2_sensor_eth_diag_top #(.CLK_HZ(1000000)) dut(.clk50m(clk),.reset_n(rst_n),.lidar_rx(1'b1),
        .rgmii_rxc(rxclk),.rgmii_rxctl(1'b0),.rgmii_rxd(4'd0),.eth_mdio(mdio),
        .imu_i2c_scl(scl),.imu_i2c_sda(sda),.left_enc_a(1'b0),.left_enc_b(1'b0),.right_enc_a(1'b0),.right_enc_b(1'b0),
        .left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2),.uart_tx(tx),
        .legacy_m1_disable(legacy),.legacy_m1_pwm_disabled(old_pwm),.legacy_cal_disabled(old_cal));
    defparam dut.imu.POWERUP_US=100;
    defparam dut.imu.RESET_US=1200;
    defparam dut.imu.STARTUP_US=500;
    defparam dut.imu_status.DECIMATION=2;
    mpu6050_model sensor(.scl(scl),.sda(sda),.present(1'b1),.identity(8'h68),
        .corrupt_readback(1'b0),.invalid_data(1'b0),.freeze_samples(1'b0));
    reg [7:0] rx_data=0;reg rx_valid=0;
    reg pv=0;reg [15:0] angle=0,distance=0;reg [7:0] intensity=77;
    integer log;reg was_tx=0;
    always @(negedge dut.network.clk125)begin
        if(dut.network.tx_valid)$fwrite(log,"%02x\n",dut.network.tx_data);
        else if(was_tx)$fwrite(log,"END\n");
        was_tx=dut.network.tx_valid;
    end
    always @(posedge clk)if(rst_n)begin
        if({l1,l2,r1,r2,legacy,old_pwm,old_cal}!==12'd0)$fatal(1,"motor pads changed");
        if(tx!==1'b1)$fatal(1,"status telemetry used UART");
    end
    task inject(input integer kind);reg [7:0] bytes[0:127];integer n,k;begin
        if(kind==0)begin n=74;$readmemh("build/lidar_eth360_20261102/set_endpoint.hex",bytes,0,73);end
        else begin n=72;$readmemh("build/lidar_eth360_20261102/arp_endpoint.hex",bytes,0,71);end
        @(negedge rxclk);rx_valid=1;
        for(k=0;k<n;k=k+1)begin rx_data=bytes[k];@(negedge rxclk);end
        rx_valid=0;rx_data=0;repeat(250)@(negedge clk);
    end endtask
    task point(input integer a,d);begin
        @(negedge clk);angle=a;distance=d;pv=1;
        @(negedge clk);pv=0;repeat(300)@(negedge clk);
    end endtask
    integer n;
    initial begin
        force GSR.GSRI=1'b1;
        force dut.network.phy_init=1'b1;
        force dut.network.rx_data=rx_data;force dut.network.rx_valid=rx_valid;force dut.network.rx_error=1'b0;
        force dut.lpv=pv;force dut.lang=angle;force dut.ldist=distance;force dut.lint=intensity;
        log=$fopen("build/lidar_eth360_20261102/static_top_tx.hex","w");
        repeat(5)@(negedge clk);rst_n=1;repeat(500)@(negedge clk);
        inject(0);inject(1);
        point(35900,1);point(37,1000);repeat(400)@(negedge clk);
        for(n=1;n<360;n=n+1)point(n*100+37,1000+n);
        point(37,2000);repeat(500000)@(negedge clk);
        if(dut.lidar_display.completed!=1 || dut.lidar_display.input_loss_count!=0)$fatal(1,"scan source failed");
        $fclose(log);$display("PASS actual M2 Ethernet top: sensor wiring, full 360 scan, static motor pads, UART idle");$finish;
    end
    initial begin #20000000;$fatal(1,"static Ethernet top timeout");end
endmodule
