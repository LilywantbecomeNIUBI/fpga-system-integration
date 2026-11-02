// Sensor-only Ethernet shell, sharing the production PHY/MAC/ARP implementation.
// Endpoint slot 1 is configured with the existing NCFG opcode 5; telemetry uses
// UDP 6205 at that endpoint. No camera or motor command ingress is instantiated.
module sensor_ethernet # (parameter CLK_HZ=50000000)(
    input clk50m,reset_n,rgmii_rxc,rgmii_rxctl,input [3:0] rgmii_rxd,
    output rgmii_gtxc,rgmii_txen,eth_mdc,eth_rst_n,output [3:0] rgmii_txd,
    inout eth_mdio,input [383:0] telemetry_frame,input telemetry_valid,
    output telemetry_ready,output [31:0] network_debug
);
    wire clk50,clk125,unused_xclk,locked,sys_rst_n,phy_init;
    wire src_rst_n,net_rst_n,rx_rst_n;
    wire [15:0] phy_status;
    ethernet_pll pll(.lock(locked),.clkout0(clk50),.clkout1(clk125),
        .clkout2(unused_xclk),.clkin(clk50m),.reset(~reset_n));
    reset_sync sys_reset(.clk(clk50),.arst_n(reset_n && locked),.rst_n(sys_rst_n));
    phy_reg_config phy(.Clk(clk50),.Rst_n(sys_rst_n),.Phy_rst_n(eth_rst_n),
        .Rd_Data(phy_status),.Phy_init_done(phy_init),.mdc(eth_mdc),.mdio(eth_mdio));
    wire common_reset_n=reset_n && locked && phy_init;
    reset_sync source_reset(.clk(clk50m),.arst_n(common_reset_n),.rst_n(src_rst_n));
    reset_sync network_reset(.clk(clk125),.arst_n(common_reset_n),.rst_n(net_rst_n));
    reset_sync receive_reset(.clk(rgmii_rxc),.arst_n(common_reset_n),.rst_n(rx_rst_n));
    wire [511:0] payload;wire payload_valid,payload_ready;
    telemetry_cdc #(.CLK_HZ(CLK_HZ)) telemetry_bridge(.src_clk(clk50m),.src_rst_n(src_rst_n),
        .src_valid(telemetry_valid),.src_frame(telemetry_frame),.src_ready(telemetry_ready),
        .dst_clk(clk125),.dst_rst_n(net_rst_n),.dst_ready(payload_ready),
        .dst_valid(payload_valid),.dst_payload(payload));
    wire [7:0] rx_data,tx_data;wire rx_valid,rx_error,tx_valid;
    rgmii_input receive_path(.rxc(rgmii_rxc),.rxd(rgmii_rxd),.rxctl(rgmii_rxctl),
        .data(rx_data),.valid(rx_valid),.error(rx_error));
    network_runtime_dual #(.DEFAULT_IP(0),.TELEMETRY_ENABLE(1)) net(
        .clk(clk125),.rst_n(net_rst_n),.rx_clk(rgmii_rxc),.rx_rst_n(rx_rst_n),
        .rx_data(rx_data),.rx_valid(rx_valid),.rx_error(rx_error),
        .video_start(1'b0),.video_first(1'b0),.video_length(16'd0),.video_data(8'd0),
        .video_read(),.video_done(),.tx_data(tx_data),.tx_valid(tx_valid),
        .target_ip(),.target_port(),.cache_hit(),.target_mac(),.enabled(),
        .arp_requests(),.rx_packets(),.rx_debug(network_debug),
        .target2_ip(),.target2_port(),.enabled2(),.cache2_hit(),.target2_mac(),
        .vision_valid(),.vision_payload(),.vision_status(256'd0),.vision_status_valid(1'b0),
        .event_valid(1'b0),.event_payload(256'd0),.event_ready(),
        .telemetry_valid(payload_valid),.telemetry_payload(payload),.telemetry_ready(payload_ready));
    rgmii_output transmit_path(.clk125(clk125),.data(tx_data),.enable(tx_valid),
        .rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen));
endmodule
