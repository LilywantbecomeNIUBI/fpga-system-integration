module mc_at8236_pwm #(parameter integer CLK_HZ=50000000, PWM_HZ=10000)(
    input wire clk, reset_n, enable,
    input wire signed [15:0] command,
    output wire in1, in2
);
    localparam integer PERIOD=CLK_HZ/PWM_HZ;
    localparam integer PHASE_BITS=PERIOD<2?1:$clog2(PERIOD);
    localparam integer DUTY_BITS=PERIOD<1?1:$clog2(PERIOD+1);
    localparam integer WAKE_LIMIT=CLK_HZ/1000+CLK_HZ/10000;
    localparam integer WAKE_BITS=WAKE_LIMIT<2?1:$clog2(WAKE_LIMIT);
    // Bounded counters only: retain every PWM/wake cycle and immediate raw
    // safety gating, but do not route enable across32 unnecessary phase bits.
    reg [PHASE_BITS-1:0] phase;
    reg [WAKE_BITS-1:0] wake_count;
    reg [1:0] power_state;
    wire waking=power_state==1;
    reg direction;
    reg [DUTY_BITS-1:0] duty;
    wire [15:0] magnitude=command[15] ? -command : command;
    // At 50MHz / 10kHz this is an exact x5, avoiding a divider chain.
    wire [31:0] scaled;
    generate if(PERIOD%1000==0) begin: exact_scale
        assign scaled=magnitude*(PERIOD/1000);
    end else begin: fractional_scale
        assign scaled=(magnitude*PERIOD)/1000;
    end endgenerate
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin phase<=0; duty<=0; direction<=0;power_state<=0;wake_count<=0;end
        else if (!enable || magnitude==0) begin phase<=0; duty<=0; direction<=0;power_state<=0;wake_count<=0;end
        else if(power_state==0) begin power_state<=1;wake_count<=0;end
        else if(waking) begin
            // Non-driving 11 brake holds an input high through the 1ms wake delay.
            // This also allows subsequent <5us PWM pulses at low duty.
            if(wake_count >= CLK_HZ/1000+CLK_HZ/10000-1) power_state<=2;
            else wake_count<=wake_count+1;
        end else if (phase==PERIOD-1) begin
            phase<=0; direction<=command[15];
            duty<=magnitude>=1000 ? PERIOD : scaled;
        end else phase<=phase+1;
    end
    // Match the board-verified diagnostic: positive=01, negative=10, idle PWM=11.
    // Hold one input high throughout slow decay so the driver stays awake.
    // Zero command, disable and reset immediately force 00, including during wake.
    wire active=reset_n && enable && magnitude!=0;
    wire driving=power_state==2 && phase<duty;
    assign in1=active && (!driving || direction);
    assign in2=active && (!driving || !direction);
endmodule
