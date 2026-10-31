// Navigation never issues CLEAR or ARM. Existing motor_core owns drive permission.
// UART STOP always wins; entering manual wheel mode while auto is active stops
// and requires the normal explicit re-arm. Internal commands get fresh core IDs.
module nav_command_mux #(parameter CLK_HZ=50000000)(
 input wire clk,rst_n,command_valid,input wire [7:0] opcode,sequence_id,command_flags,
 input wire signed [15:0] left_target,right_target,input wire [15:0] kp,ki,
 input wire armed,input wire [7:0] core_sequence,
 input wire nav_running,nav_permit,input wire signed [15:0] nav_left,nav_right,
 output reg core_valid,output reg [7:0] core_opcode,core_id,core_flags,
 output reg signed [15:0] core_left,core_right,output reg [15:0] core_kp,core_ki,
 output reg auto_owner,output wire inhibit);
 reg [7:0] saved_flags;reg [15:0] saved_kp,saved_ki;reg was_running;
 reg [31:0] divider;
 assign inhibit=auto_owner && (!nav_running || !nav_permit);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin core_valid<=0;core_opcode<=0;core_id<=0;core_flags<=0;core_left<=0;core_right<=0;
   core_kp<=0;core_ki<=0;auto_owner<=0;saved_flags<=0;saved_kp<=0;saved_ki<=0;was_running<=0;divider<=0;end
  else begin
   core_valid<=0;was_running<=nav_running;
   if(nav_running)auto_owner<=1;else if(!armed)auto_owner<=0;
   if(divider==CLK_HZ/100-1)divider<=0;else divider<=divider+1'b1;
   if(nav_running && divider==CLK_HZ/100-1)begin
    core_valid<=1;core_opcode<=2;core_id<=core_sequence+1'b1;
    core_flags<=saved_flags;core_kp<=saved_kp;core_ki<=saved_ki;
    core_left<=nav_permit?nav_left:16'sd0;core_right<=nav_permit?nav_right:16'sd0;
   end
   if(was_running && !nav_running)begin core_valid<=1;core_opcode<=0;core_left<=0;core_right<=0;end
   if(command_valid && (opcode<8'h40 || opcode==8'h44))begin
    core_valid<=1;core_opcode<=opcode;core_id<=sequence_id;core_flags<=command_flags;
    core_left<=left_target;core_right<=right_target;core_kp<=kp;core_ki<=ki;
    if(opcode==1 && !armed)begin saved_flags<=command_flags;saved_kp<=kp;saved_ki<=ki;end
    if(opcode==8'h44 || (auto_owner && opcode==2))begin core_opcode<=0;core_left<=0;core_right<=0;end
   end
  end
 end
endmodule
