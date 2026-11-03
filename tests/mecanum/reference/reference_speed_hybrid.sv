// Frozen pre-optimization reference for arithmetic/behavior equivalence.
// 1kHz observer. Output is signed Q8 position-units / 10ms (NOT units / tick).
// Low speed: edge period, aged conservatively when edges cease. High speed:
// eight rolling 1ms buckets. No combinational divider; T uses 32 clocks.
module reference_speed_hybrid #(parameter integer CLK_HZ=50000000)(
    input wire clk,reset_n,clear,tick,
    input wire signed [31:0] position,
    output reg signed [31:0] velocity_q8,
    output wire motion_recent,
    output reg fault,
    output reg count_mode
);
    localparam integer STOP_CYCLES=CLK_HZ/20; // 50ms without an edge -> zero.
    localparam [31:0] PERIOD_NUMERATOR=(CLK_HZ/100)*256;
    reg signed [31:0] previous_position,bucket_delta,delta_history[0:7],rolling_delta;
    reg [15:0] bucket_events,event_history[0:7],rolling_events;
    reg [2:0] index;
    reg [31:0] timestamp,last_edge_time,edge_age,last_period;
    reg seen_edge,period_valid,last_negative;
    reg [1:0] last_step;
    wire signed [31:0] step=position-previous_position; // modulo-32 position wrap.
    wire edge_now=step!=0;
    wire bad_step=step>2 || step< -2;
    wire turning=edge_now && seen_edge && step[31]!=last_negative;
    wire [31:0] interval_cycles=timestamp-last_edge_time;
    wire [31:0] denominator=edge_age>last_period ? edge_age : last_period;
    wire signed [31:0] new_bucket=bucket_delta+step;
    wire signed [31:0] new_rolling=rolling_delta-delta_history[index]+new_bucket;
    wire [16:0] new_events={1'b0,rolling_events}-{1'b0,event_history[index]}+
                               {1'b0,bucket_events}+edge_now;
    // Hysteresis is in actual decoded events, so +/-1 and +/-2 decoders both work.
    wire use_count=count_mode ? new_events>=6 : new_events>=10;
    assign motion_recent=seen_edge && edge_age<STOP_CYCLES;
    reg dividing,divide_negative;
    reg [5:0] bits_left;
    reg [31:0] shift_value,divisor;
    reg [32:0] remainder_value;
    wire [32:0] trial={remainder_value[31:0],shift_value[31]};
    wire take_bit=trial>={1'b0,divisor};
    wire [31:0] next_shift={shift_value[30:0],take_bit};
    integer i;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            previous_position<=0;bucket_delta<=0;bucket_events<=0;rolling_delta<=0;rolling_events<=0;index<=0;
            timestamp<=0;last_edge_time<=0;edge_age<=STOP_CYCLES;last_period<=0;
            seen_edge<=0;period_valid<=0;last_negative<=0;last_step<=0;
            velocity_q8<=0;fault<=0;count_mode<=0;dividing<=0;bits_left<=0;
            shift_value<=0;divisor<=1;remainder_value<=0;divide_negative<=0;
            for(i=0;i<8;i=i+1)begin delta_history[i]<=0;event_history[i]<=0;end
        end else if(clear)begin
            previous_position<=position;bucket_delta<=0;bucket_events<=0;rolling_delta<=0;rolling_events<=0;index<=0;
            timestamp<=0;last_edge_time<=0;edge_age<=STOP_CYCLES;last_period<=0;
            seen_edge<=0;period_valid<=0;last_negative<=0;last_step<=0;
            velocity_q8<=0;fault<=0;count_mode<=0;dividing<=0;
            for(i=0;i<8;i=i+1)begin delta_history[i]<=0;event_history[i]<=0;end
        end else begin
            previous_position<=position;timestamp<=timestamp+1'b1;
            if(edge_age<32'hffffffff)edge_age<=edge_age+1'b1;
            if(dividing)begin
                shift_value<=next_shift;
                remainder_value<=take_bit ? trial-{1'b0,divisor} : trial;
                bits_left<=bits_left-1'b1;
                if(bits_left==1)begin
                    dividing<=0;
                    velocity_q8<=divide_negative ? -$signed(next_shift) : $signed(next_shift);
                end
            end
            if(edge_now)begin
                edge_age<=0;last_edge_time<=timestamp;last_negative<=step[31];
                last_step<=step[31] ? -step : step;
                seen_edge<=1;
                if(!seen_edge || turning || interval_cycles>STOP_CYCLES)period_valid<=0;
                else begin last_period<=interval_cycles;period_valid<=interval_cycles!=0;end
            end
            if(turning)begin
                // Never publish a quotient or rolling window from the old direction.
                velocity_q8<=0;dividing<=0;count_mode<=0;bucket_delta<=step;bucket_events<=1;
                rolling_delta<=0;rolling_events<=0;index<=0;
                for(i=0;i<8;i=i+1)begin delta_history[i]<=0;event_history[i]<=0;end
            end else if(tick)begin
                delta_history[index]<=new_bucket;event_history[index]<=bucket_events+edge_now;
                rolling_delta<=new_rolling;rolling_events<=new_events[15:0];index<=index+1'b1;
                bucket_delta<=0;bucket_events<=0;
                if(dividing)begin fault<=1;velocity_q8<=0;dividing<=0;end
                else if(edge_age>=STOP_CYCLES && !edge_now)begin
                    velocity_q8<=0;count_mode<=0;
                end else if(use_count)begin
                    // Eight ms / ten ms normalization: 256*10/8 = 320 exactly.
                    velocity_q8<=new_rolling*32'sd320;count_mode<=1;
                end else begin
                    count_mode<=0;
                    if(period_valid && denominator!=0)begin
                        shift_value<=last_step==2 ? PERIOD_NUMERATOR<<1 : PERIOD_NUMERATOR;
                        divisor<=denominator;remainder_value<=0;bits_left<=32;
                        divide_negative<=last_negative;dividing<=1;
                    end else velocity_q8<=0;
                end
            end else begin
                bucket_delta<=new_bucket;
                if(edge_now && bucket_events!=16'hffff)bucket_events<=bucket_events+1'b1;
            end
            if(bad_step || fault)begin fault<=1;velocity_q8<=0;dividing<=0;end
        end
    end
endmodule
