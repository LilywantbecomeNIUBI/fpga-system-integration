// Eight outward normals. Stage 1 dot products (0/+1/-1 need no multiplier),
// stage 2 comparison flags, stage 3 commit. A changed input revokes old permit.
// This is final authorization, NOT post-sweep velocity clipping.
module mc_nav_direction_gate(input wire clk,rst_n,global_permit,
 input wire [7:0] danger_mask,input wire forward_flow_risk,rotation_requested,
 input wire signed [15:0] target_x,target_y,actual_x,actual_y,
 output wire permit,output reg [7:0] blocked_mask);
 reg [1:0] state;reg [2:0] index;reg valid,blocked;
 reg [7:0] mask;reg flow,rotation;
 reg signed [15:0] tx,ty,ax,ay;
 reg signed [18:0] target_dot,actual_dot;
 reg signed [18:0] target_dx,target_dy,actual_dx,actual_dy;
 reg signed [1:0] nx,ny;
 always @*begin
  nx=0;ny=0;
  case(index)
   0:nx=1;1:nx=-1;2:ny=1;3:ny=-1;
   4:begin nx=1;ny=1;end 5:begin nx=1;ny=-1;end
   6:begin nx=-1;ny=1;end 7:begin nx=-1;ny=-1;end
  endcase
  target_dx=nx==1?$signed(tx):nx==-1?-$signed({tx[15],tx}):19'sd0;
  target_dy=ny==1?$signed(ty):ny==-1?-$signed({ty[15],ty}):19'sd0;
  actual_dx=nx==1?$signed(ax):nx==-1?-$signed({ax[15],ax}):19'sd0;
  actual_dy=ny==1?$signed(ay):ny==-1?-$signed({ay[15],ay}):19'sd0;
 end
 wire same_inputs=mask==danger_mask && flow==forward_flow_risk && rotation==rotation_requested &&
  tx==target_x && ty==target_y && ax==actual_x && ay==actual_y;
 assign permit=global_permit && valid && same_inputs && blocked_mask==0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;index<=0;valid<=0;blocked<=1;blocked_mask<=255;mask<=0;flow<=0;rotation<=0;
   tx<=0;ty<=0;ax<=0;ay<=0;target_dot<=0;actual_dot<=0;
  end else case(state)
   0:if(!same_inputs || !valid)begin
    mask<=danger_mask;flow<=forward_flow_risk;rotation<=rotation_requested;tx<=target_x;ty<=target_y;ax<=actual_x;ay<=actual_y;
    index<=0;valid<=0;blocked_mask<=0;state<=1;
   end
   1:begin target_dot<=target_dx+target_dy;actual_dot<=actual_dx+actual_dy;state<=2;end
   2:begin blocked<=(mask[index] && (target_dot>0 || actual_dot>5 || rotation)) || (index==0 && flow && (tx>0 || ax>5 || rotation));state<=3;end
   3:begin blocked_mask[index]<=blocked;if(index==7)begin valid<=1;state<=0;end else begin index<=index+1'b1;state<=1;end end
  endcase
 end
endmodule
