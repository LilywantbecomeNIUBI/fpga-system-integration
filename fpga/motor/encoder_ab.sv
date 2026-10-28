// AB x4 decoder. Positive sequence: 00 -> 01 -> 11 -> 10 -> 00.
module encoder_ab #(parameter integer FILTER_CYCLES=8)(
    input wire clk, reset_n, a, b, sample_tick, reverse,
    output reg signed [31:0] position,
    output reg signed [31:0] velocity,
    output reg [31:0] invalid_count
);
    (* syn_preserve=1 *) reg [1:0] ab_meta, ab_sync;
    reg [1:0] candidate, stable, previous;
    integer filter_count;
    reg initialized;
    reg signed [31:0] window_count;
    reg signed [31:0] step;
    always @* begin
        step=0;
        if (initialized) begin
            case ({previous,stable})
                4'b0001,4'b0111,4'b1110,4'b1000: step=1;
                4'b0010,4'b1011,4'b1101,4'b0100: step=-1;
                default: step=0;
            endcase
        end
        if (reverse) step=-step;
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ab_meta<=0; ab_sync<=0; candidate<=0; stable<=0; previous<=0;
            filter_count<=0; initialized<=0; position<=0; velocity<=0;
            window_count<=0; invalid_count<=0;
        end else begin
            ab_meta<={a,b}; ab_sync<=ab_meta;
            previous<=stable;
            if (ab_sync != candidate) begin candidate<=ab_sync; filter_count<=0; end
            else if (filter_count < FILTER_CYCLES) filter_count<=filter_count+1;
            else begin
                stable<=candidate;
                if(!initialized) previous<=candidate;
                initialized<=1;
            end
            if (initialized && ((previous ^ stable)==2'b11)) invalid_count<=invalid_count+1;
            position<=position+step;
            if (sample_tick) begin velocity<=window_count+step; window_count<=0; end
            else window_count<=window_count+step;
        end
    end
endmodule
