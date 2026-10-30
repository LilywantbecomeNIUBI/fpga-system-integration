// Temporary open-loop diagnostic. No encoder or PI inputs exist in this design.
// Dedicated flags=0x22 and gains=0 prevent the normal speed-loop host from arming it.
module pwm_probe_core #(parameter integer CLK_HZ=50000000, RUN_MS=5000, WATCHDOG_MS=250, LIMIT=200)(
 input wire clk,reset_n,command_valid,
 input wire [7:0] opcode,sequence_id,command_flags,
 input wire signed [15:0] left_target,right_target,
 input wire [15:0] kp,ki,
 output reg armed,output reg [15:0] faults,
 output reg [7:0] last_sequence,
 output reg signed [15:0] left_applied,right_applied,
 output reg [31:0] rejected_commands
);
 localparam integer RUN_CYCLES=(CLK_HZ/1000)*RUN_MS;
 localparam integer WD_CYCLES=(CLK_HZ/1000)*WATCHDOG_MS;
 integer watchdog,elapsed;
 reg started;
 wire [7:0] delta=sequence_id-last_sequence;
 wire fresh=delta!=0 && delta<128;
 wire config_ok=command_flags==8'h22 && kp==0 && ki==0;
 wire targets_ok=left_target>=0 && left_target<=LIMIT && right_target>=0 && right_target<=LIMIT && (left_target==0 || right_target==0);
 always @(posedge clk or negedge reset_n) begin
  if(!reset_n)begin armed<=0;faults<=1;last_sequence<=0;left_applied<=0;right_applied<=0;watchdog<=0;elapsed<=0;started<=0;rejected_commands<=0;end
  else begin
   if(armed)watchdog<=watchdog+1;
   if(armed && started)elapsed<=elapsed+1;
   if(command_valid)begin
    if(opcode==0)begin armed<=0;left_applied<=0;right_applied<=0;end
    else if(opcode==3 && !armed && left_target==0 && right_target==0)begin faults<=0;watchdog<=0;end
    else if(opcode==1 && !armed && faults==0 && config_ok && left_target==0 && right_target==0)begin
     armed<=1;last_sequence<=sequence_id;left_applied<=0;right_applied<=0;watchdog<=0;elapsed<=0;started<=0;
    end else if(opcode==2 && armed && fresh && config_ok && targets_ok)begin
     last_sequence<=sequence_id;watchdog<=0;left_applied<=left_target;right_applied<=right_target;
     if(!started && (left_target!=0 || right_target!=0))begin started<=1;elapsed<=0;end
    end else rejected_commands<=rejected_commands+1;
   end
   // Neither repeated DRIVE nor fresh sequence can extend the one-shot window.
   if(armed && started && elapsed>=RUN_CYCLES-1)begin armed<=0;left_applied<=0;right_applied<=0;end
   if(armed && watchdog>=WD_CYCLES-1)begin armed<=0;faults[1]<=1;left_applied<=0;right_applied<=0;end
  end
 end
endmodule

// Slow decay: 01 drive, 11 brake. Duty means drive time, not IN1 high time.
module pwm_probe_output #(parameter integer CLK_HZ=50000000,PWM_HZ=10000)(
 input wire clk,reset_n,enable,input wire [15:0] duty_permille,
 output wire in1,in2
);
 localparam integer PERIOD=CLK_HZ/PWM_HZ;
 localparam integer WAKE_CYCLES=(CLK_HZ/10000)*11;
 integer phase,wake_count;
 wire active=reset_n && enable && duty_permille!=0;
 wire [31:0] high_cycles=duty_permille*(PERIOD/1000);
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin phase<=0;wake_count<=0;end
  else if(!active)begin phase<=0;wake_count<=0;end
  else if(wake_count<WAKE_CYCLES)begin phase<=0;wake_count<=wake_count+1;end
  else if(phase==PERIOD-1)phase<=0;
  else phase<=phase+1;
 end
 assign in1=active && !(wake_count>=WAKE_CYCLES && phase<high_cycles);
 assign in2=active;
endmodule

module pwm_probe_top #(parameter integer CLK_HZ=50000000,BAUD=115200)(
 input wire clk50m,reset_n,uart_rx,
 inout wire left_in1,left_in2,right_in1,right_in2,
 output wire uart_tx,
 output wire legacy_m1_disable,legacy_m2_disable,
 output wire [5:0] legacy_m1_pwm_disabled,legacy_m2_pwm_disabled,
 output wire [1:0] legacy_cal_disabled,output wire [7:0] led
);
 (* syn_preserve=1 *)reg reset_meta,reset_sync;
 always @(posedge clk50m or negedge reset_n)begin
  if(!reset_n)begin reset_meta<=0;reset_sync<=0;end
  else begin reset_meta<=1;reset_sync<=reset_meta;end
 end
 wire [7:0] data,opcode,seq,flags,last_seq;
 wire byte_valid,framing_error,valid,armed;
 wire signed [15:0] target_l,target_r,applied_l,applied_r;
 wire [15:0] kp,ki,faults;
 wire [31:0] bad_frames,rejected;
 uart_rx_byte #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) rx(.clk(clk50m),.reset_n(reset_sync),.rx(uart_rx),.data(data),.valid(byte_valid),.framing_error(framing_error));
 motor_protocol #(.CLK_HZ(CLK_HZ)) protocol(.clk(clk50m),.reset_n(reset_sync),.byte_valid(byte_valid),.framing_error(framing_error),.byte_data(data),.command_valid(valid),.opcode(opcode),.sequence_id(seq),.flags(flags),.left_target(target_l),.right_target(target_r),.kp(kp),.ki(ki),.bad_frames(bad_frames));
 pwm_probe_core #(.CLK_HZ(CLK_HZ)) core(.clk(clk50m),.reset_n(reset_sync),.command_valid(valid),.opcode(opcode),.sequence_id(seq),.command_flags(flags),.left_target(target_l),.right_target(target_r),.kp(kp),.ki(ki),.armed(armed),.faults(faults),.last_sequence(last_seq),.left_applied(applied_l),.right_applied(applied_r),.rejected_commands(rejected));
 wire logic_l1,logic_l2,logic_r1,logic_r2;wire [3:0] pad_sense;
 pwm_probe_output #(.CLK_HZ(CLK_HZ)) left_pwm(.clk(clk50m),.reset_n(reset_sync),.enable(armed),.duty_permille(applied_l),.in1(logic_l1),.in2(logic_l2));
 pwm_probe_output #(.CLK_HZ(CLK_HZ)) right_pwm(.clk(clk50m),.reset_n(reset_sync),.enable(armed),.duty_permille(applied_r),.in1(logic_r1),.in2(logic_r2));
 // IOBUF.O reads the electrical pad input. It has no path into PWM control.
 IOBUF pad_l1(.IO(left_in1),.O(pad_sense[0]),.I(logic_l1),.OEN(1'b0));
 IOBUF pad_l2(.IO(left_in2),.O(pad_sense[1]),.I(logic_l2),.OEN(1'b0));
 IOBUF pad_r1(.IO(right_in1),.O(pad_sense[2]),.I(logic_r1),.OEN(1'b0));
 IOBUF pad_r2(.IO(right_in2),.O(pad_sense[3]),.I(logic_r2),.OEN(1'b0));
 (* syn_preserve=1 *) reg [3:0] pad_meta,pad_sync;
 reg [3:0] pad_prev;
 reg [31:0] pad_edges_l,pad_edges_r,pad_high_l,pad_high_r;
 always @(posedge clk50m or negedge reset_sync)begin
  if(!reset_sync)begin pad_meta<=0;pad_sync<=0;pad_prev<=0;pad_edges_l<=0;pad_edges_r<=0;pad_high_l<=0;pad_high_r<=0;end
  else begin
   pad_meta<=pad_sense;pad_sync<=pad_meta;pad_prev<=pad_sync;
   if(!pad_sync[0]&&pad_sync[1]&&(pad_prev[0]||!pad_prev[1]))pad_edges_l<=pad_edges_l+1;
   if(!pad_sync[2]&&pad_sync[3]&&(pad_prev[2]||!pad_prev[3]))pad_edges_r<=pad_edges_r+1;
   if(!pad_sync[0]&&pad_sync[1])pad_high_l<=pad_high_l+1;
   if(!pad_sync[2]&&pad_sync[3])pad_high_r<=pad_high_r+1;
  end
 end
 reg prev_l,prev_r;reg [31:0] pulses_l,pulses_r;
 always @(posedge clk50m or negedge reset_sync)begin
  if(!reset_sync)begin prev_l<=0;prev_r<=0;pulses_l<=0;pulses_r<=0;end
  else begin prev_l<=(!logic_l1&&logic_l2);prev_r<=(!logic_r1&&logic_r2);if(!logic_l1&&logic_l2&&!prev_l)pulses_l<=pulses_l+1;if(!logic_r1&&logic_r2&&!prev_r)pulses_r<=pulses_r+1;end
 end
 // In diagnostic flags=0x22, position fields count internal PWM rising edges, NOT encoder motion.
 motor_telemetry #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) telemetry(.clk(clk50m),.reset_n(reset_sync),.armed(armed),.flags(8'h22),.last_sequence(last_seq),.faults(faults),.left_position(pulses_l),.right_position(pulses_r),.left_velocity(pad_high_l),.right_velocity(pad_high_r),.left_applied(applied_l),.right_applied(applied_r),.left_invalid(pad_edges_l),.right_invalid(pad_edges_r),.bad_frames(bad_frames),.rejected_commands(rejected),.tx(uart_tx));
 assign legacy_m1_disable=0,legacy_m2_disable=0,legacy_m1_pwm_disabled=0,legacy_m2_pwm_disabled=0,legacy_cal_disabled=0;
 assign led={3'b101,faults[1:0],applied_r!=0,applied_l!=0,armed};
endmodule
