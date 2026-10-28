module at8236_pwm #(parameter integer CLK_HZ=50000000, PWM_HZ=10000)(
    input wire clk, reset_n, enable,
    input wire signed [15:0] command,
    output wire in1, in2
);
    localparam integer PERIOD=CLK_HZ/PWM_HZ;
    integer phase,wake_count;
    reg [1:0] power_state;
    wire waking=power_state==1;
    reg direction;
    integer duty;
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
    // Fast decay after wake; disable immediately forces 00, including during wake.
    assign in1=enable && (waking || (power_state==2 && !direction && phase<duty));
    assign in2=enable && (waking || (power_state==2 && direction && phase<duty));
endmodule
