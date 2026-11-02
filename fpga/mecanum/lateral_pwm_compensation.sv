// Feedforward for pure relative lateral moves. PI remains free to reduce PWM.
// Nominal GMR scale: 160 control counts is approximately 10mm of travel.
module mc_lateral_pwm_compensation #(
 parameter integer PWM_PERMILLE=150,TAPER_COUNTS=160
)(
 input wire clk,reset_n,active,arrived,input wire [1:0] mode,
 input wire signed [15:0] goal_x,goal_y,goal_angle,
 input wire signed [31:0] position_y,
 output wire [15:0] compensation
);
 function automatic integer gcd(input integer a,b);
  integer r;
  begin while(b!=0)begin r=a%b;a=b;b=r;end gcd=a;end
 endfunction
 localparam COMMON=gcd(PWM_PERMILLE,TAPER_COUNTS);
 localparam NUMERATOR=PWM_PERMILLE/COMMON,DENOMINATOR=TAPER_COUNTS/COMMON;
 reg signed [32:0] remaining;
 wire [32:0] magnitude=remaining<0 ? -remaining : remaining;
 reg [15:0] distance_counts,compensation_q;
 reg [31:0] taper_product;
 // Four clocks (<0.1us at 50MHz), far below the 10ms control window.
 // Reduce 150/160 to 15/16 so the final division is just a right shift.
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin remaining<=0;distance_counts<=0;taper_product<=0;compensation_q<=0;end
  else begin
   remaining<=$signed(goal_y)*33'sd65536-$signed(position_y);
   distance_counts<=magnitude>=(TAPER_COUNTS*33'd65536) ? TAPER_COUNTS : magnitude[31:16];
   taper_product<=distance_counts*NUMERATOR;
   compensation_q<=taper_product/DENOMINATOR;
  end
 end
 wire lateral=active && !arrived && mode==2 && goal_x==0 && goal_y!=0 && goal_angle==0;
 assign compensation=!lateral ? 16'd0 : compensation_q;
endmodule
