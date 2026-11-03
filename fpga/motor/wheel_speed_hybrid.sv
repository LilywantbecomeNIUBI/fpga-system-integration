// 1kHz observer. Output: signed Q8 position units / 10ms.
// Low speed uses a 32-clock period divider; high speed uses eight 1ms buckets.
// History is a fixed-tap shift queue: no variable-index 8:1 read mux.
module wheel_speed_hybrid #(parameter integer CLK_HZ=50000000)(
    input wire clk,reset_n,clear,tick,
    input wire signed [31:0] position,
    output reg signed [31:0] velocity_q8,
    output wire motion_recent,
    output reg fault,
    output reg count_mode
);
    localparam integer STOP_CYCLES=CLK_HZ/20;
    localparam [31:0] PERIOD_NUMERATOR=(CLK_HZ/100)*256;
    // At most one decoded +/-2 event per FPGA clock. These widths cover
    // every event in a complete 1ms bucket, not just expected motor speeds.
    localparam integer BUCKET_CYCLES=(CLK_HZ+999)/1000;
    localparam integer EVENT_W=BUCKET_CYCLES<2 ? 1 : $clog2(BUCKET_CYCLES+1);
    localparam integer DELTA_W=$clog2(2*BUCKET_CYCLES+1)+1;
    localparam integer ROLL_DW=DELTA_W+3,ROLL_EW=EVENT_W+3;
    reg signed [31:0] previous_position;
    reg signed [DELTA_W-1:0] bucket_delta,delta_history[0:7],captured_bucket,expired_delta;
    reg [EVENT_W-1:0] bucket_events,event_history[0:7],captured_events,expired_events;
    reg signed [ROLL_DW-1:0] rolling_delta,partial_delta,total_delta;
    reg [ROLL_EW-1:0] rolling_events,partial_events,total_events;
    reg [2:0] sample_state;
    reg sample_stopped,sample_period_valid,sample_negative;
    reg [1:0] sample_step;
    reg [31:0] sample_denominator;
    reg signed [31:0] count_x256,count_x64;
    reg [31:0] timestamp,last_edge_time,edge_age,last_period;
    reg seen_edge,period_valid,last_negative;
    reg [1:0] last_step;
    wire signed [31:0] step=position-previous_position;
    wire edge_now=step!=0;
    wire bad_step=step>2 || step< -2;
    wire turning=edge_now && seen_edge && step[31]!=last_negative;
    wire [31:0] interval_cycles=timestamp-last_edge_time;
    wire [31:0] denominator=edge_age>last_period ? edge_age : last_period;
    wire signed [DELTA_W:0] bucket_next=$signed(bucket_delta)+step;
    wire [EVENT_W:0] events_next={1'b0,bucket_events}+edge_now;
    wire bucket_overflow=bucket_next[DELTA_W]!=bucket_next[DELTA_W-1] || events_next[EVENT_W];
    wire use_count=count_mode ? total_events>=6 : total_events>=10;
    wire signed [31:0] count_extended=total_delta;
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
            previous_position<=0;bucket_delta<=0;bucket_events<=0;rolling_delta<=0;rolling_events<=0;
            captured_bucket<=0;expired_delta<=0;captured_events<=0;expired_events<=0;
            partial_delta<=0;partial_events<=0;total_delta<=0;total_events<=0;sample_state<=0;
            sample_stopped<=0;sample_period_valid<=0;sample_negative<=0;sample_step<=0;sample_denominator<=0;
            count_x256<=0;count_x64<=0;
            timestamp<=0;last_edge_time<=0;edge_age<=STOP_CYCLES;last_period<=0;
            seen_edge<=0;period_valid<=0;last_negative<=0;last_step<=0;
            velocity_q8<=0;fault<=0;count_mode<=0;dividing<=0;bits_left<=0;
            shift_value<=0;divisor<=1;remainder_value<=0;divide_negative<=0;
            for(i=0;i<8;i=i+1)begin delta_history[i]<=0;event_history[i]<=0;end
        end else if(clear)begin
            previous_position<=position;bucket_delta<=0;bucket_events<=0;rolling_delta<=0;rolling_events<=0;
            sample_state<=0;timestamp<=0;last_edge_time<=0;edge_age<=STOP_CYCLES;last_period<=0;
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
            // Every stage uses the tick snapshot. New encoder events continue
            // accumulating independently in the next bucket during calculation.
            case(sample_state)
                1:begin
                    partial_delta<=rolling_delta-expired_delta;
                    partial_events<=rolling_events-expired_events;sample_state<=2;
                end
                2:begin
                    total_delta<=partial_delta+captured_bucket;
                    total_events<=partial_events+captured_events;sample_state<=3;
                end
                3:begin
                    rolling_delta<=total_delta;rolling_events<=total_events;sample_state<=0;
                    if(sample_stopped)begin velocity_q8<=0;count_mode<=0;end
                    else if(use_count)begin
                        // 8ms -> 10ms: 256*10/8 = 256+64, no DSP multiplier.
                        count_x256<=count_extended<<<8;count_x64<=count_extended<<<6;
                        count_mode<=1;sample_state<=4;
                    end else begin
                        count_mode<=0;
                        if(sample_period_valid && sample_denominator!=0)begin
                            shift_value<=sample_step==2 ? PERIOD_NUMERATOR<<1 : PERIOD_NUMERATOR;
                            divisor<=sample_denominator;remainder_value<=0;bits_left<=32;
                            divide_negative<=sample_negative;dividing<=1;
                        end else velocity_q8<=0;
                    end
                end
                4:begin velocity_q8<=count_x256+count_x64;sample_state<=0;end
                default:begin end
            endcase
            if(edge_now)begin
                edge_age<=0;last_edge_time<=timestamp;last_negative<=step[31];
                last_step<=step[31] ? -step : step;seen_edge<=1;
                if(!seen_edge || turning || interval_cycles>STOP_CYCLES)period_valid<=0;
                else begin last_period<=interval_cycles;period_valid<=interval_cycles!=0;end
            end
            if(turning)begin
                // Cancel all in-flight stages and old-direction quotients.
                velocity_q8<=0;dividing<=0;sample_state<=0;count_mode<=0;
                bucket_delta<=step;bucket_events<=1;rolling_delta<=0;rolling_events<=0;
                for(i=0;i<8;i=i+1)begin delta_history[i]<=0;event_history[i]<=0;end
            end else if(tick)begin
                captured_bucket<=bucket_next[DELTA_W-1:0];captured_events<=events_next[EVENT_W-1:0];
                expired_delta<=delta_history[7];expired_events<=event_history[7];
                for(i=7;i>0;i=i-1)begin delta_history[i]<=delta_history[i-1];event_history[i]<=event_history[i-1];end
                delta_history[0]<=bucket_next[DELTA_W-1:0];event_history[0]<=events_next[EVENT_W-1:0];
                bucket_delta<=0;bucket_events<=0;
                sample_stopped<=edge_age>=STOP_CYCLES && !edge_now;
                sample_period_valid<=period_valid;sample_denominator<=denominator;
                sample_step<=last_step;sample_negative<=last_negative;
                if(dividing || sample_state!=0)begin fault<=1;velocity_q8<=0;dividing<=0;sample_state<=0;end
                else sample_state<=1;
            end else begin bucket_delta<=bucket_next[DELTA_W-1:0];bucket_events<=events_next[EVENT_W-1:0];end
            // A missed/late sampling tick must fault instead of truncating a
            // bucket; normal 1kHz operation fits the proven widths above.
            if(bad_step || fault || (!turning && bucket_overflow))begin
                fault<=1;velocity_q8<=0;dividing<=0;sample_state<=0;
            end
        end
    end
endmodule
