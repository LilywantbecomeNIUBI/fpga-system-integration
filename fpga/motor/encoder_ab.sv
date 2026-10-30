// A-edge x2 decoder, scaled to x4 units: +/-2 per A edge, +/-4 per cycle.
// Positive sequence remains 00 -> 01 -> 11 -> 10 -> 00.
// Sampled B determines direction; arbitrary missing phase is not recoverable.
module encoder_ab #(parameter integer FILTER_CYCLES=8, COUNT_B_EDGES=0,
    RECOVER_DIAGONALS=0, RECOVERY_MAX_GAP_CYCLES=500000)(
    input wire clk, reset_n, a, b, sample_tick, reverse,
    output reg signed [31:0] position,
    output reg signed [31:0] velocity,
    output reg [31:0] invalid_count,
    output reg [287:0] diagnostics,
    output wire [3:0] diagnostic_levels,
    output reg [255:0] transition_trace,
    output reg [31:0] transition_count
);
    (* syn_preserve=1 *) reg [1:0] ab_meta, ab_sync;
    reg [1:0] candidate, stable, previous;
    integer filter_count;
    reg initialized;
    reg signed [31:0] window_count;
    reg signed [31:0] step;
    reg [1:0] sync_previous;
    reg signed [31:0] legal_step;
    reg direction_valid,last_positive;
    reg [31:0] direction_age;
    assign diagnostic_levels={ab_sync,stable};
    always @* begin
        legal_step=0;
        if(initialized)case({previous,stable})
            4'b0001,4'b0111,4'b1110,4'b1000:legal_step=1;
            4'b0010,4'b1011,4'b1101,4'b0100:legal_step=-1;
            default:legal_step=0;
        endcase
        step=0;
        if (initialized) begin
            // Optional continuity assumption, evaluated on recorded AB first.
            // Without a recent legal transition a diagonal remains ambiguous.
            if(RECOVER_DIAGONALS)begin
                step=legal_step;
                if((previous^stable)==2'b11 && direction_valid && direction_age<RECOVERY_MAX_GAP_CYCLES)
                    step=last_positive ? 32'sd2 : -32'sd2;
            end else if(COUNT_B_EDGES) begin
                if(previous[0]!=stable[0])step=(stable[1]^stable[0]) ? 32'sd2 : -32'sd2;
            end else if(previous[1]!=stable[1])
                step=(stable[1]^stable[0]) ? -32'sd2 : 32'sd2;
        end
        if (reverse) step=-step;
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ab_meta<=0; ab_sync<=0; candidate<=0; stable<=0; previous<=0;
            filter_count<=0; initialized<=0; position<=0; velocity<=0;
            window_count<=0; invalid_count<=0;
            diagnostics<=0;sync_previous<=0;
            transition_trace<=0;transition_count<=0;
            direction_valid<=0;last_positive<=0;direction_age<=0;
        end else begin
            ab_meta<={a,b}; ab_sync<=ab_meta;
            sync_previous<=ab_sync;
            previous<=stable;
            if(legal_step!=0)begin
                direction_valid<=1;last_positive<=legal_step>0;direction_age<=0;
            end else if(direction_valid)begin
                if(direction_age>=RECOVERY_MAX_GAP_CYCLES-1)direction_valid<=0;
                else direction_age<=direction_age+32'd1;
            end
            if (ab_sync != candidate) begin candidate<=ab_sync; filter_count<=0; end
            else if (filter_count < FILTER_CYCLES) filter_count<=filter_count+1;
            else begin
                stable<=candidate;
                if(!initialized) previous<=candidate;
                initialized<=1;
            end
            // Explicitly selected mode: legacy invalid diagnostic is disabled.
            // Zero does not certify signal quality. Other core guards remain.
            invalid_count<=0;
            // Passive counters; never feed motor enable, faults or the PI.
            // Order: sync A/B, filtered A/B, A+/A-, B+/B-, diagonal.
            if(initialized) begin
                if(previous!=stable) begin
                    transition_trace<={transition_trace[251:0],previous,stable};
                    transition_count<=transition_count+32'd1;
                end
                if(sync_previous[1]!=ab_sync[1]) diagnostics[0+:32]<=diagnostics[0+:32]+32'd1;
                if(sync_previous[0]!=ab_sync[0]) diagnostics[32+:32]<=diagnostics[32+:32]+32'd1;
                if(previous[1]!=stable[1]) begin
                    diagnostics[64+:32]<=diagnostics[64+:32]+32'd1;
                    if(stable[1]==stable[0]) diagnostics[128+:32]<=diagnostics[128+:32]+32'd1;
                    else diagnostics[160+:32]<=diagnostics[160+:32]+32'd1;
                end
                if(previous[0]!=stable[0]) begin
                    diagnostics[96+:32]<=diagnostics[96+:32]+32'd1;
                    if(stable[1]!=stable[0]) diagnostics[192+:32]<=diagnostics[192+:32]+32'd1;
                    else diagnostics[224+:32]<=diagnostics[224+:32]+32'd1;
                end
                if((previous^stable)==2'b11) diagnostics[256+:32]<=diagnostics[256+:32]+32'd1;
            end
            position<=position+step;
            if (sample_tick) begin velocity<=window_count+step; window_count<=0; end
            else window_count<=window_count+step;
        end
    end
endmodule
