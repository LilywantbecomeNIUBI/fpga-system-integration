// UART burst decoupling + 72 synchronous RAM sectors. Unknown is never free.
// Commit after two valid points in a contiguous sector visit; retain old minima
// until the new visit qualifies. Each sector expires independently (modulo ms).
module nav_histogram(input wire clk,rst_n,input wire [15:0] now_ms,
 input wire point_valid,input wire [15:0] angle_q100,distance_mm,input wire [7:0] intensity,
 input wire signed [15:0] angle_offset,input wire angle_reverse,
 input wire [15:0] timeout_ms,stop_mm,input wire [7:0] min_intensity,
 input wire [6:0] read_index,output reg [15:0] read_distance,output reg read_known,
 input wire clear_hazard,input wire [31:0] clear_sequence,
 output reg hazard,output reg [31:0] hazard_sequence,output reg [31:0] settled_sequence,output reg overflow,output reg ready);
 reg [39:0] fifo[0:31];reg [5:0] wp,rp;
 reg [15:0] ranges[0:71],stamps[0:71];reg known[0:71];
 reg [6:0] init_index,bin,previous;reg previous_valid;
 reg [15:0] angle_work,distance_work,pending_min;reg good,tainted;reg [1:0] samples;
 reg [2:0] state;reg [15:0] old_distance,old_stamp;reg old_known;
 reg signed [17:0] corrected;
 wire full=wp-rp==32;
 wire front=(corrected<3000 || corrected>=33000);
 always @* begin
  corrected=(angle_reverse ? -$signed({1'b0,angle_q100}) : $signed({1'b0,angle_q100}))+$signed(angle_offset);
  if(corrected<0)corrected=corrected+36000;
  else if(corrected>=36000)corrected=corrected-36000;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin wp<=0;rp<=0;overflow<=0;hazard<=1;hazard_sequence<=0;settled_sequence<=0;ready<=0;
   init_index<=0;state<=0;bin<=0;previous<=0;previous_valid<=0;pending_min<=65535;
   tainted<=0;samples<=0;angle_work<=0;distance_work<=0;good<=0;
   old_distance<=0;old_stamp<=0;old_known<=0;read_distance<=0;read_known<=0;end
  else begin
   read_distance<=ranges[read_index<72 ? read_index : 0];
   read_known<=ready && read_index<72 && known[read_index<72 ? read_index : 0] &&
     $unsigned(now_ms-stamps[read_index<72 ? read_index : 0])<timeout_ms;
   if(ready && state==0 && rp==wp && !point_valid)settled_sequence<=hazard_sequence;
   if(clear_hazard && clear_sequence==hazard_sequence && !overflow)hazard<=0;
   // New hazards win over release, even while the acquisition FIFO is busy.
   if(point_valid && front && (distance_mm==0 || intensity<min_intensity || distance_mm<=stop_mm))begin
    hazard<=1;hazard_sequence<=hazard_sequence+1'b1;
   end
   if(point_valid && ready)begin
    if(full)begin overflow<=1;hazard<=1;end
    else begin fifo[wp[4:0]]<={intensity,corrected[15:0],distance_mm};wp<=wp+1'b1;end
   end
   if(!ready)begin
    ranges[init_index]<=0;stamps[init_index]<=0;known[init_index]<=0;
    if(init_index==71)ready<=1;else init_index<=init_index+1'b1;
   end else case(state)
    0:if(rp!=wp)begin
     {good,angle_work,distance_work}<={fifo[rp[4:0]][39:32]>=min_intensity && fifo[rp[4:0]][15:0]!=0,fifo[rp[4:0]][31:0]};
     rp<=rp+1'b1;bin<=0;state<=1;
    end
    1:if(angle_work>=500)begin angle_work<=angle_work-500;bin<=bin+1'b1;end
      else state<=2;
    2:begin old_distance<=ranges[bin];old_known<=known[bin] && $unsigned(now_ms-stamps[bin])<timeout_ms;old_stamp<=stamps[bin];state<=3;end
    3:begin
     previous<=bin;previous_valid<=1;state<=0;
     if(!previous_valid || previous!=bin)begin
      samples<=good?1:0;tainted<=!good;pending_min<=good?distance_work:16'hffff;
      if(!good)begin known[bin]<=0;ranges[bin]<=0;end
      else if(!old_known || distance_work<old_distance)begin ranges[bin]<=distance_work;known[bin]<=old_known;stamps[bin]<=old_stamp;end
     end else begin
      if(good && samples<3)samples<=samples+1'b1;
      if(!good)begin tainted<=1;known[bin]<=0;ranges[bin]<=0;end
      else begin
       if(distance_work<pending_min)pending_min<=distance_work;
       if(!tainted && samples>=1)begin
        known[bin]<=1;stamps[bin]<=now_ms;
        ranges[bin]<=distance_work<pending_min ? distance_work : pending_min;
       end else if(old_known && distance_work<old_distance)ranges[bin]<=distance_work;
      end
     end
    end
    default:begin state<=0;overflow<=1;hazard<=1;end
   endcase
  end
 end
endmodule
