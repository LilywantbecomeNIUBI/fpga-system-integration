// PHY RX delay plus FPGA clock insertion require data/CTL deskew before IDDR.
// Arora V UG304: ~12.5 ps/tap; 128 nominal taps add about 1.6 ns to all five inputs.
module rgmii_input #(parameter RX_DATA_TAPS=128)(input rxc,input [3:0] rxd,input rxctl,
    output [7:0] data,output valid,output error);
    wire ctl_rise,ctl_fall;
    genvar n;
    generate for(n=0;n<4;n=n+1)begin:rx_io
        wire delayed;
        IODELAY #(.C_STATIC_DLY(RX_DATA_TAPS)) deskew(.DO(delayed),.DF(),.DI(rxd[n]),.DLYSTEP(8'd0),.SDTAP(1'b0),.VALUE(1'b0));
        IDDR sample(.Q0(data[n]),.Q1(data[n+4]),.D(delayed),.CLK(rxc));
    end endgenerate
    wire ctl_delayed;
    IODELAY #(.C_STATIC_DLY(RX_DATA_TAPS)) ctl_deskew(.DO(ctl_delayed),.DF(),.DI(rxctl),.DLYSTEP(8'd0),.SDTAP(1'b0),.VALUE(1'b0));
    IDDR control(.Q0(ctl_rise),.Q1(ctl_fall),.D(ctl_delayed),.CLK(rxc));
    assign valid=ctl_rise;
    assign error=ctl_rise^ctl_fall;
endmodule
