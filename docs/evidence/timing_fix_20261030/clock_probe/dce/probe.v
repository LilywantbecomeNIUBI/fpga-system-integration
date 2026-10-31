module clock_probe(input clk50m,reset_n,camera_pclk,output reg led);
 wire clk50,clk125,raw125,unused,lock,pixel_clk;
 DCE tx_buffer(.CLKIN(raw125),.CE(1'b1),.CLKOUT(clk125));
 DCE pixel_buffer(.CLKIN(camera_pclk),.CE(1'b1),.CLKOUT(pixel_clk));
 reg [15:0] pc=0;always @(posedge pixel_clk)pc<=pc+1'b1;
 ethernet_pll pll(.lock(lock),.clkout0(clk50),.clkout1(raw125),.clkout2(unused),.clkin(clk50m),.reset(~reset_n));
 reg [9:0] wa=0,ra=0;reg [15:0] ram[0:1023];reg [15:0] q,qr;
 always @(posedge clk50)begin wa<=wa+1'b1;ram[wa]<={6'd0,wa};end
 always @(posedge clk125)begin ra<=ra+1'b1;q<=ram[ra];qr<=q;led<=(^qr)^(^pc);end
endmodule
