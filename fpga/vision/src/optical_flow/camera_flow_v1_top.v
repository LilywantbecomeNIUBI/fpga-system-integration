module camera_flow_v1_top(
    input clk50m,reset_n,rgmii_rxc,input [3:0] rgmii_rxd,input rgmii_rxctl,
    output camera_sclk,inout camera_sdat,input camera_vsync,camera_href,camera_pclk,input [7:0] camera_data,
    output camera_rst_n,camera_pwdn,rgmii_gtxc,output [3:0] rgmii_txd,output rgmii_txen,
    output eth_mdc,inout eth_mdio,output eth_rst_n,output [7:0] led,output uart_tx,
    output m1_drv_en,m2_drv_en,lidar_moto_pwm
);
    wire [31:0] network_debug,pclk_cycles;
    camera_flow #(.STAGE(1)) camera(.clk50m(clk50m),.reset_n(reset_n),.rgmii_rxc(rgmii_rxc),.rgmii_rxd(rgmii_rxd),.rgmii_rxctl(rgmii_rxctl),
        .camera_sclk(camera_sclk),.camera_sdat(camera_sdat),.camera_vsync(camera_vsync),.camera_href(camera_href),.camera_pclk(camera_pclk),.camera_data(camera_data),
        .camera_rst_n(camera_rst_n),.camera_pwdn(camera_pwdn),.rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen),
        .eth_mdc(eth_mdc),.eth_mdio(eth_mdio),.eth_rst_n(eth_rst_n),.led(led),.network_debug(network_debug),
        .m1_drv_en(m1_drv_en),.m2_drv_en(m2_drv_en),.lidar_moto_pwm(lidar_moto_pwm));
    camera_clock_meter meter(.pclk(camera_pclk),.clk(clk50m),.rst_n(reset_n),.cycles(pclk_cycles));
    camera_status_uart #(.TELEMETRY(1),.NETWORK(1)) status_uart(.clk(clk50m),.rst_n(reset_n),.status(led),.pclk_cycles(pclk_cycles),.network_debug(network_debug),.tx(uart_tx));
endmodule
