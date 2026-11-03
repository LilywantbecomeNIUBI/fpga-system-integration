// Serialized histogram RMW. Mirrored synchronous SDP RAM, no RAM reset.
module nav_histogram #(parameter SYNC_CLEAR=0)(input wire clk,rst_n,input wire [15:0] now_ms,
 input wire point_valid,input wire [15:0] angle_q100,distance_mm,input wire [7:0] intensity,
 input wire signed [15:0] angle_offset,input wire angle_reverse,
 input wire [15:0] timeout_ms,stop_mm,input wire [7:0] min_intensity,
 input wire [6:0] read_index,output reg [15:0] read_distance,output reg read_known,
 input wire clear_hazard,input wire [31:0] clear_sequence,
 output reg hazard,output reg [31:0] hazard_sequence,output reg [31:0] settled_sequence,output reg overflow,output reg ready,
 input wire sync_clear);
 localparam IDLE=0,FIFO_LATCH=1,BIN=2,OLD_REQ=3,OLD_LATCH=4,OLD_FLAGS=5,UPDATE=6,WRITE=7;
 (* syn_ramstyle="block_ram" *) reg [39:0] fifo[0:31];
 (* syn_ramstyle="block_ram" *) reg [31:0] sectors_read[0:127];
 (* syn_ramstyle="block_ram" *) reg [31:0] sectors_update[0:127];
 reg [39:0] fifo_q;reg [31:0] read_q,old_q;
 reg [5:0] wp,rp;reg [71:0] known;
 reg [6:0] init_index,bin,previous;reg previous_valid;
 reg [15:0] angle_work,distance_work,pending_min;reg good,tainted;reg [1:0] samples;
 reg [2:0] state;reg [15:0] old_distance,old_stamp;reg old_known;
 reg [31:0] commit_word;reg commit_known;
 reg [6:0] read_tag;reg read_tag_known,forward_q;reg [31:0] forward_word;
 reg signed [17:0] corrected;
 wire clearing=SYNC_CLEAR && sync_clear;
 wire full=wp-rp==32;
 wire front=(corrected<3000 || corrected>=33000);
 wire push=point_valid && ready && !full && !clearing;
 wire pop=ready && state==IDLE && rp!=wp && !clearing;
 wire write_en=(!ready || state==WRITE) && !clearing;
 wire [6:0] write_addr=!ready?init_index:bin;
 wire [31:0] write_data=!ready?32'd0:commit_word;
 wire write_known=ready && commit_known;
 wire [6:0] read_addr=read_index<72?read_index:0;
 wire [31:0] output_word=forward_q?forward_word:read_q;
 always @* begin
  corrected=(angle_reverse ? -$signed({1'b0,angle_q100}) : $signed({1'b0,angle_q100}))+$signed(angle_offset);
  if(corrected<0)corrected=corrected+36000;
  else if(corrected>=36000)corrected=corrected-36000;
 end
 // Each array has one synchronous read and one write port, no reset.
 always @(posedge clk)begin
  if(push)fifo[wp[4:0]]<={intensity,corrected[15:0],distance_mm};
  if(pop)fifo_q<=fifo[rp[4:0]];
  if(write_en)begin sectors_read[write_addr]<=write_data;sectors_update[write_addr]<=write_data;end
  read_q<=sectors_read[read_addr];
  if(state==OLD_REQ)old_q<=sectors_update[bin];
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   wp<=0;rp<=0;overflow<=0;hazard<=1;hazard_sequence<=0;settled_sequence<=0;ready<=0;
   init_index<=0;known<=0;state<=IDLE;bin<=0;previous<=0;previous_valid<=0;pending_min<=65535;
   tainted<=0;samples<=0;angle_work<=0;distance_work<=0;good<=0;
   old_distance<=0;old_stamp<=0;old_known<=0;read_distance<=0;read_known<=0;
   commit_word<=0;commit_known<=0;read_tag<=0;read_tag_known<=0;forward_q<=0;forward_word<=0;
  end else if(clearing)begin
   wp<=0;rp<=0;overflow<=0;hazard<=1;hazard_sequence<=0;settled_sequence<=0;ready<=0;
   init_index<=0;known<=0;state<=IDLE;previous_valid<=0;pending_min<=65535;tainted<=0;samples<=0;
   read_known<=0;read_tag_known<=0;forward_q<=0;commit_known<=0;
  end else begin
   read_tag<=read_index;
   read_tag_known<=read_index<72 && (write_en && write_addr==read_addr?write_known:known[read_addr]);
   forward_q<=write_en && write_addr==read_addr;forward_word<=write_data;
   read_distance<=output_word[15:0];
   read_known<=ready && read_tag<72 && read_tag_known && $unsigned(now_ms-output_word[31:16])<timeout_ms;
   if(ready && state==IDLE && rp==wp && !point_valid)settled_sequence<=hazard_sequence;
   if(clear_hazard && clear_sequence==hazard_sequence && !overflow)hazard<=0;
   // Raw hazards win over release even while FIFO acquisition is busy.
   if(point_valid && front && (distance_mm==0 || intensity<min_intensity || distance_mm<=stop_mm))begin
    hazard<=1;hazard_sequence<=hazard_sequence+1'b1;
   end
   if(point_valid && ready)begin
    if(full)begin overflow<=1;hazard<=1;end else wp<=wp+1'b1;
   end
   if(write_en)known[write_addr]<=write_known;
   if(!ready)begin
    if(init_index==71)ready<=1;else init_index<=init_index+1'b1;
   end else case(state)
    IDLE:if(pop)begin rp<=rp+1'b1;state<=FIFO_LATCH;end
    FIFO_LATCH:begin
     good<=fifo_q[39:32]>=min_intensity && fifo_q[15:0]!=0;
     angle_work<=fifo_q[31:16];distance_work<=fifo_q[15:0];bin<=0;state<=BIN;
    end
    BIN:if(angle_work>=500)begin angle_work<=angle_work-500;bin<=bin+1'b1;end else state<=OLD_REQ;
    OLD_REQ:state<=OLD_LATCH;
    OLD_LATCH:begin old_distance<=old_q[15:0];old_stamp<=old_q[31:16];state<=OLD_FLAGS;end
    OLD_FLAGS:begin old_known<=known[bin] && $unsigned(now_ms-old_stamp)<timeout_ms;state<=UPDATE;end
    UPDATE:begin
     previous<=bin;previous_valid<=1;state<=WRITE;
     commit_word<={old_stamp,old_distance};commit_known<=known[bin];
     if(!previous_valid || previous!=bin)begin
      samples<=good?1:0;tainted<=!good;pending_min<=good?distance_work:16'hffff;
      if(!good)begin commit_known<=0;commit_word<={old_stamp,16'd0};end
      else if(!old_known || distance_work<old_distance)begin commit_word<={old_stamp,distance_work};commit_known<=old_known;end
     end else begin
      if(good && samples<3)samples<=samples+1'b1;
      if(!good)begin tainted<=1;commit_known<=0;commit_word<={old_stamp,16'd0};end
      else begin
       if(distance_work<pending_min)pending_min<=distance_work;
       if(!tainted && samples>=1)begin
        commit_known<=1;commit_word<={now_ms,(distance_work<pending_min?distance_work:pending_min)};
       end else if(old_known && distance_work<old_distance)commit_word<={old_stamp,distance_work};
      end
     end
    end
    WRITE:state<=IDLE;
    default:begin state<=IDLE;overflow<=1;hazard<=1;end
   endcase
  end
 end
endmodule
