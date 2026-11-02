module uart_rx_byte #(parameter integer CLK_HZ=50000000, BAUD=115200)(
    input wire clk, reset_n, rx,
    output reg [7:0] data, output reg valid, framing_error
);
    localparam integer DIV=CLK_HZ/BAUD;
    (* syn_preserve=1 *) reg rx_meta, rx_sync;
    reg [1:0] state;
    integer count;
    reg [2:0] bit_index;
    reg [7:0] shift;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_meta<=1; rx_sync<=1; state<=0; count<=0; bit_index<=0;
            shift<=0; data<=0; valid<=0; framing_error<=0;
        end else begin
            rx_meta<=rx; rx_sync<=rx_meta; valid<=0; framing_error<=0;
            case(state)
                0: if (!rx_sync) begin state<=1; count<=DIV/2-1; end
                1: if (count!=0) count<=count-1;
                   else if (!rx_sync) begin state<=2; count<=DIV-1; bit_index<=0; end
                   else state<=0;
                2: if (count!=0) count<=count-1;
                   else begin
                       shift[bit_index]<=rx_sync; count<=DIV-1;
                       if (bit_index==7) state<=3; else bit_index<=bit_index+3'd1;
                   end
                3: if (count!=0) count<=count-1;
                   else begin
                       state<=0;
                       if (rx_sync) begin data<=shift; valid<=1; end
                       else framing_error<=1;
                   end
            endcase
        end
    end
endmodule

module uart_tx_byte #(parameter integer CLK_HZ=50000000, BAUD=115200)(
    input wire clk, reset_n, start, input wire [7:0] data,
    output wire tx, output reg busy
);
    localparam integer DIV=CLK_HZ/BAUD;
    reg [9:0] shift;
    integer count;
    reg [3:0] bit_index;
    assign tx=busy ? shift[0] : 1'b1;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin shift<=10'h3ff; count<=0; bit_index<=0; busy<=0; end
        else if (!busy) begin
            if (start) begin shift<={1'b1,data,1'b0}; busy<=1; count<=DIV-1; bit_index<=0; end
        end else if (count!=0) count<=count-1;
        else if (bit_index==9) busy<=0;
        else begin shift<={1'b1,shift[9:1]}; bit_index<=bit_index+4'd1; count<=DIV-1; end
    end
endmodule
