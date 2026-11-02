`timescale 1ns/1ps
// Digital clock source for integration tests only. Production uses the unchanged
// Gowin PLL wrapper. No lock acquisition/jitter/phase/electrical claim is made.
module ethernet_pll(output lock,clkout0,clkout1,clkout2,input clkin,reset);
 reg clk125=0;
 always #4 clk125=~clk125;
 assign clkout0=clkin;
 assign clkout1=reset ? 1'b0 : clk125;
 assign clkout2=1'b0;
 assign lock=~reset;
endmodule
