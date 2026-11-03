// One immutable command mailbox, acknowledged after all 24 bytes are consumed.
// Both resets derive from network reset. No delayed command queue/retry.
module mc_command_cdc(
 input wire src_clk,src_reset_n,src_valid,input wire [191:0] src_frame,
 output wire src_ready,input wire dst_clk,dst_reset_n,
 output reg byte_valid,output reg [7:0] byte_data
);
 reg request,ack;
 (* async_reg="true",syn_preserve=1 *) reg ack_meta,ack_sync,req_meta,req_sync;
 reg [191:0] held,shift;
 reg busy;reg [4:0] index;
 assign src_ready=src_reset_n && request==ack_sync;
 always @(posedge src_clk or negedge src_reset_n)begin
  if(!src_reset_n)begin request<=0;ack_meta<=0;ack_sync<=0;held<=0;end
  else begin
   ack_meta<=ack;ack_sync<=ack_meta;
   // src_ready keeps reset gating externally. Reset already dominates this
   // branch; avoid routing that same reset back through the192-bit held CE.
   if(src_valid && request==ack_sync)begin held<=src_frame;request<=!request;end
  end
 end
 always @(posedge dst_clk or negedge dst_reset_n)begin
  if(!dst_reset_n)begin ack<=0;req_meta<=0;req_sync<=0;shift<=0;busy<=0;index<=0;byte_valid<=0;byte_data<=0;end
  else begin
   req_meta<=request;req_sync<=req_meta;byte_valid<=0;
   if(!busy && req_sync!=ack)begin shift<=held;busy<=1;index<=0;end
   else if(busy)begin
    byte_data<=shift[7:0];byte_valid<=1;shift<=shift>>8;
    if(index==23)begin busy<=0;ack<=req_sync;end
    else index<=index+1'b1;
   end
  end
 end
endmodule
