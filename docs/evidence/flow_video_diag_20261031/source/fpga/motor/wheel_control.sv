// Seven-clock PI pipeline; one update per control tick. All datapaths timed.
module wheel_control #(
    parameter integer PWM_LIMIT=300, SLEW=20, REVERSE_TICKS=10
)(
    input wire clk, reset_n, tick, enable, motor_reverse,
    input wire signed [15:0] target,
    input wire signed [31:0] velocity,
    input wire [15:0] kp, ki,
    output reg signed [15:0] applied
);
    localparam signed [15:0] STEP=SLEW;
    localparam signed [31:0] I_BOUND=PWM_LIMIT*256;
    reg [2:0] state;
    reg signed [31:0] integral,p_product,i_product,integral_sum,next_integral,raw_output;
    reg signed [16:0] error_value;
    reg signed [15:0] requested,demand;
    reg signed [13:0] kp_q;
    reg signed [11:0] ki_q;
    reg reverse_q,stopped_q,last_direction;
    integer quiet_ticks;
    wire signed [15:0] bounded_velocity=velocity>32767 ? 16'sd32767 : velocity< -32767 ? -16'sd32767 : velocity[15:0];
    wire signed [15:0] electrical_demand=reverse_q ? -demand : demand;
    function automatic signed [31:0] clamp_i(input signed [31:0] value);
        if(value>I_BOUND) clamp_i=I_BOUND;
        else if(value< -I_BOUND) clamp_i=-I_BOUND;
        else clamp_i=value;
    endfunction
    function automatic signed [15:0] clamp_pwm(input signed [31:0] value);
        if(value>PWM_LIMIT) clamp_pwm=PWM_LIMIT;
        else if(value< -PWM_LIMIT) clamp_pwm=-PWM_LIMIT;
        else clamp_pwm=value[15:0];
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=0;integral<=0;p_product<=0;i_product<=0;integral_sum<=0;next_integral<=0;
            raw_output<=0;error_value<=0;requested<=0;demand<=0;kp_q<=0;ki_q<=0;
            reverse_q<=0;stopped_q<=0;last_direction<=0;quiet_ticks<=0;applied<=0;
        end else if(!enable) begin state<=0;integral<=0;quiet_ticks<=0;last_direction<=0;applied<=0;end
        else case(state)
            0: if(tick) begin
                error_value<=$signed({target[15],target})-$signed({bounded_velocity[15],bounded_velocity});
                requested<=target;kp_q<=$signed({1'b0,kp[12:0]});ki_q<=$signed({1'b0,ki[10:0]});
                reverse_q<=motor_reverse;stopped_q<=velocity<=1 && velocity>=-1;state<=1;
            end
            1: begin p_product<=error_value*kp_q;i_product<=error_value*ki_q;state<=2;end
            2: begin integral_sum<=integral+i_product;state<=3;end
            3: begin next_integral<=clamp_i(integral_sum);state<=4;end
            4: begin raw_output<=p_product+next_integral;state<=5;end
            5: begin demand<=requested==0 ? 16'sd0 : clamp_pwm(raw_output>>>8);state<=6;end
            6: begin
                state<=0;
                if(requested==0) integral<=0;
                else if((raw_output<=I_BOUND && raw_output>=-I_BOUND) ||
                        (raw_output>I_BOUND && error_value<0) ||
                        (raw_output< -I_BOUND && error_value>0)) integral<=next_integral;
                if(applied==0 && stopped_q) begin
                    if(quiet_ticks<REVERSE_TICKS) quiet_ticks<=quiet_ticks+1;
                end else quiet_ticks<=0;
                if(electrical_demand!=0 && electrical_demand[15]!=last_direction && quiet_ticks<REVERSE_TICKS) begin
                    if(applied>STEP) applied<=applied-STEP;
                    else if(applied< -STEP) applied<=applied+STEP;
                    else applied<=0;
                end else begin
                    if($signed({electrical_demand[15],electrical_demand})>$signed({applied[15],applied})+SLEW) applied<=applied+STEP;
                    else if($signed({electrical_demand[15],electrical_demand})<$signed({applied[15],applied})-SLEW) applied<=applied-STEP;
                    else applied<=electrical_demand;
                    if(electrical_demand!=0) last_direction<=electrical_demand[15];
                end
            end
            default:state<=0;
        endcase
    end
endmodule
