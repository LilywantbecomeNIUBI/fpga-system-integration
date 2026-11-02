// Relative yaw only: a six-axis IMU cannot observe absolute compass heading.
module mpu6050_yaw #(parameter CAL_LOG2=8,REVERSE=0)(
 input wire clk,reset_n,sample_valid,sensor_ok,wheels_still,recalibrate,
 input wire signed [15:0] gx,gy,gz,input wire [31:0] sample_us,
 output reg calibrated,output wire signed [31:0] yaw_q16,
 output reg signed [15:0] bias,output wire healthy
);
 localparam signed [63:0] PI_Q32=64'sd13493037705,TWO_PI_Q32=64'sd26986075409;
 reg [CAL_LOG2:0] count;
 reg signed [31:0] sum;
 reg [31:0] previous_us;
 reg have_previous;
 reg signed [63:0] angle;
 reg [2:0] state;
 reg signed [16:0] corrected;
 reg signed [31:0] dt;
 reg signed [63:0] product,delta,next_angle;
 wire stationary=wheels_still && gx<655 && gx> -655 && gy<655 && gy> -655 && gz<655 && gz> -655;
 wire [31:0] elapsed=sample_us-previous_us;
 assign yaw_q16=angle>>>16;
 assign healthy=calibrated && sensor_ok;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin count<=0;sum<=0;bias<=0;calibrated<=0;angle<=0;previous_us<=0;have_previous<=0;state<=0;corrected<=0;dt<=0;product<=0;delta<=0;next_angle<=0;end
  else if(!sensor_ok || recalibrate)begin count<=0;sum<=0;bias<=0;calibrated<=0;angle<=0;previous_us<=0;have_previous<=0;state<=0;end
  else begin
   case(state)
    0:if(sample_valid)begin
     previous_us<=sample_us;have_previous<=1;
     if(!calibrated)begin
      if(!stationary)begin count<=0;sum<=0;end
      else if(count==(1<<CAL_LOG2)-1)begin bias<=($signed(sum)+$signed(gz))>>>CAL_LOG2;calibrated<=1;angle<=0;count<=0;sum<=0;end
      else begin sum<=sum+$signed(gz);count<=count+1'b1;end
     end else if(!have_previous || elapsed>25000)begin
      // Resume at the new timestamp after a long gap. Unknown motion during
      // missing samples cannot be reconstructed; retain angle/calibration.
      state<=0;
     end else if(elapsed<5000 || gz>=32760 || gz<=-32760)begin
      // User-selected hold policy also skips short-interval/saturated samples.
      // Never erase a completed calibration or angle for an isolated sample.
      state<=0;
     end
     else begin
      corrected<=REVERSE ? $signed({bias[15],bias})-$signed({gz[15],gz}) : $signed({gz[15],gz})-$signed({bias[15],bias});
      dt<=$signed(elapsed);state<=1;
     end
    end
    1:begin product<=corrected*dt;state<=2;end
    // 9600323 / 2^24 rad-Q32 per raw LSB per microsecond (131 LSB/dps).
    2:begin product<=product*64'sd9600323;state<=3;end
    3:begin delta<=product>>>24;state<=4;end
    4:begin next_angle<=angle+delta;state<=5;end
    5:begin
     if(next_angle>PI_Q32)angle<=next_angle-TWO_PI_Q32;
     else if(next_angle< -PI_Q32)angle<=next_angle+TWO_PI_Q32;
     else angle<=next_angle;
     state<=0;
    end
    default:state<=0;
   endcase
  end
 end
endmodule
