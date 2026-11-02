// RTL8211 TX timing uses the board's strap-selected internal RGMII delay.
module rgmii_output(
    input clk125, input [7:0] data, input enable,
    output rgmii_gtxc, output [3:0] rgmii_txd, output rgmii_txen
);
    genvar i;
    generate for (i=0;i<4;i=i+1) begin: tx_io
        ODDR data_ddr(.Q0(rgmii_txd[i]), .Q1(), .D0(data[i]), .D1(data[i+4]),
                      .TX(1'b0), .CLK(clk125));
    end endgenerate
    ODDR ctrl_ddr(.Q0(rgmii_txen), .Q1(), .D0(enable), .D1(enable), .TX(1'b0), .CLK(clk125));
    ODDR clk_ddr(.Q0(rgmii_gtxc), .Q1(), .D0(1'b1), .D1(1'b0), .TX(1'b0), .CLK(clk125));
endmodule
