// Keep raw high-resolution telemetry. Normalize control units with an exact
// signed remainder so slow pulses are accumulated, not discarded per window.
module mc_encoder_control_scale #(parameter SHIFT=4)(
 input wire clk,reset_n,sample_tick,
 input wire signed [31:0] raw_position,raw_velocity,
 output wire signed [31:0] control_position,control_velocity
);
 generate if(SHIFT==0)begin:passthrough
  assign control_position=raw_position;assign control_velocity=raw_velocity;
 end else begin:scaled
  localparam signed [35:0] FACTOR=36'sd1<<SHIFT;
  reg signed [31:0] previous,position,velocity;
  reg signed [35:0] remainder;
  // Subtraction in 32 bits deliberately accounts for raw position wrap.
  wire signed [31:0] raw_delta=raw_position-previous;
  wire signed [35:0] total=$signed(raw_delta)+remainder;
  wire signed [35:0] count=total/FACTOR; // signed divide truncates toward zero
  always @(posedge clk or negedge reset_n)begin
   if(!reset_n)begin previous<=0;position<=0;velocity<=0;remainder<=0;end
   else if(sample_tick)begin
    previous<=raw_position;position<=position+count;velocity<=count;
    remainder<=total-count*FACTOR;
   end
  end
  assign control_position=position;assign control_velocity=velocity;
 end endgenerate
endmodule
