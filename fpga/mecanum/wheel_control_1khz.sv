// Fixed 1kHz PI. target: integer units/10ms, velocity: Q8 units/10ms.
// kp/ki retain legacy Q8 / 100Hz-reference semantics. Integral retains the
// factor ten in its accumulator, so small Ki is not rounded away each tick.
// Output conversion is bounded restoring division (32 clocks), not a wide /.
module mc_wheel_control_1khz #(
    parameter integer PWM_LIMIT=300, SLEW=2, REVERSE_TICKS=100,FRACTIONAL_TARGET=0
)(
    input wire clk,reset_n,tick,enable,motor_reverse,feedback_stopped,
    input wire signed [15:0] target,
    input wire target_fractional,
    input wire signed [31:0] velocity_q8,
    input wire [15:0] kp,ki,feedforward,
    output reg signed [15:0] applied
);
    localparam signed [47:0] I_BOUND=PWM_LIMIT*48'sd655360;
    localparam [31:0] OUTPUT_SCALE=32'd655360; // Q8 error * Q8 gain * ten.
    reg [3:0] state;
    reg signed [47:0] integral,integral_sum,next_integral,raw_output;
    // Exact signed operand widths: 25x14 and 25x12. Do not let the
    // accumulator's 48-bit context widen the multipliers.
    reg signed [38:0] p_product;
    reg signed [36:0] i_product,ff_signed;
    reg signed [42:0] p_scaled;
    reg signed [43:0] proportional_ff;
    wire signed [42:0] p_extended={{4{p_product[38]}},p_product};
    reg signed [24:0] error_value;
    reg signed [15:0] requested,demand;
    reg [12:0] kp_q;reg [10:0] ki_q;
    reg reverse_q,stopped_q,last_direction;
    reg [15:0] feedforward_q;
    wire signed [36:0] ff_scaled=$signed({1'b0,feedforward_q})*37'sd655360;
    integer quiet_ticks;
    wire signed [31:0] bounded_velocity=velocity_q8>8388352 ? 32'sd8388352 :
                                                velocity_q8< -8388352 ? -32'sd8388352 : velocity_q8;
    wire signed [31:0] integer_target_q8=$signed(target)*32'sd256;
    wire signed [31:0] target_q8=FRACTIONAL_TARGET && target_fractional ?
        $signed({{16{target[15]}},target}) : integer_target_q8;
    wire signed [15:0] electrical_demand=reverse_q ? -demand : demand;
    reg [31:0] shift_value;reg [20:0] remainder_value;
    reg [5:0] bits_left;reg divide_negative;
    wire [21:0] trial={remainder_value,shift_value[31]};
    wire take_bit=trial>=OUTPUT_SCALE;
    wire [31:0] next_shift={shift_value[30:0],take_bit};
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=0;integral<=0;p_product<=0;i_product<=0;integral_sum<=0;
            next_integral<=0;raw_output<=0;ff_signed<=0;p_scaled<=0;proportional_ff<=0;error_value<=0;requested<=0;demand<=0;
            kp_q<=0;ki_q<=0;feedforward_q<=0;reverse_q<=0;stopped_q<=0;last_direction<=0;quiet_ticks<=0;applied<=0;
            shift_value<=0;remainder_value<=0;bits_left<=0;divide_negative<=0;
        end else if(!enable)begin
            state<=0;integral<=0;quiet_ticks<=0;last_direction<=0;applied<=0;
        end else case(state)
            0:if(tick)begin
                error_value<=target_q8-bounded_velocity;requested<=target;
                kp_q<=kp[12:0];ki_q<=ki[10:0];feedforward_q<=feedforward;reverse_q<=motor_reverse;
                stopped_q<=feedback_stopped && velocity_q8<=256 && velocity_q8>=-256;state<=1;
            end
            1:begin
                p_product<=error_value*$signed({1'b0,kp_q});
                i_product<=error_value*$signed({1'b0,ki_q});
                ff_signed<=requested<0 ? -ff_scaled : ff_scaled;state<=2;
            end
            2:begin
                integral_sum<=integral+i_product;
                p_scaled<=(p_extended<<<3)+(p_extended<<<1);state<=3;
            end
            3:begin
                next_integral<=integral_sum>I_BOUND ? I_BOUND : integral_sum< -I_BOUND ? -I_BOUND : integral_sum;
                proportional_ff<=$signed({p_scaled[42],p_scaled})+
                    $signed({{7{ff_signed[36]}},ff_signed});state<=4;
            end
            4:begin raw_output<=proportional_ff+next_integral;state<=5;end
            5:begin
                if(requested==0)integral<=0;
                else if((raw_output<=I_BOUND && raw_output>=-I_BOUND) ||
                    (raw_output>I_BOUND && error_value<0) || (raw_output< -I_BOUND && error_value>0))
                    integral<=next_integral;
                remainder_value<=0;bits_left<=32;divide_negative<=raw_output[47];
                if(requested==0)shift_value<=0;
                else if(raw_output>I_BOUND || raw_output< -I_BOUND)shift_value<=I_BOUND[31:0];
                else shift_value<=raw_output[47] ? -raw_output : raw_output;
                state<=6;
            end
            6:begin
                shift_value<=next_shift;remainder_value<=take_bit ? trial-OUTPUT_SCALE : trial;
                bits_left<=bits_left-1'b1;
                if(bits_left==1)begin
                    demand<=divide_negative ? -$signed(next_shift[15:0]) : $signed(next_shift[15:0]);state<=7;
                end
            end
            7:begin
                state<=0;
                if(applied==0 && stopped_q)begin
                    if(quiet_ticks<REVERSE_TICKS)quiet_ticks<=quiet_ticks+1;
                end else quiet_ticks<=0;
                if(electrical_demand!=0 && electrical_demand[15]!=last_direction && quiet_ticks<REVERSE_TICKS)begin
                    if(applied>SLEW)applied<=applied-SLEW;
                    else if(applied< -SLEW)applied<=applied+SLEW;
                    else applied<=0;
                end else begin
                    if($signed({electrical_demand[15],electrical_demand})>$signed({applied[15],applied})+SLEW)applied<=applied+SLEW;
                    else if($signed({electrical_demand[15],electrical_demand})<$signed({applied[15],applied})-SLEW)applied<=applied-SLEW;
                    else applied<=electrical_demand;
                    if(electrical_demand!=0)last_direction<=electrical_demand[15];
                end
            end
            default:state<=0;
        endcase
    end
endmodule
