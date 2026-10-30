module clock_probe(input clk50m,reset_n,output reg led);
 wire clk50,clk125,unused,lock;
 ethernet_pll pll(.lock(lock),.clkout0(clk50),.clkout1(clk125),.clkout2(unused),.clkin(clk50m),.reset(~reset_n));
 reg [9:0] wa=0,ra=0;reg [15:0] ram[0:1023];reg [15:0] q,qr;
 always @(posedge clk50)begin wa<=wa+1'b1;ram[wa]<={6'd0,wa};end
 always @(posedge clk125)begin ra<=ra+1'b1;q<=ram[ra];qr<=q;led<=^qr;end
endmodule
