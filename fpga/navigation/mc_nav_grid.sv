// Packed immutable foreground / serialized background RMW.
// Synchronous SDP inference: never reset RAM data or read asynchronously.
module mc_nav_grid #(parameter SYNC_CLEAR=0)(
 input wire clk,rst_n,input wire [31:0] now_ms,
 input wire scan_begin,scan_abort,publish,input wire planner_lock,
 input wire cell_valid,input wire [13:0] cell_address,input wire [1:0] cell_code,
 output wire cell_ready,output reg cell_done,
 input wire read_valid,input wire [13:0] read_address,
 output wire read_ready,output reg read_done,output reg [1:0] read_code,
 output wire map_valid,output reg [31:0] epoch,map_ms,
 output reg foreground,output wire initializing,output reg scan_bad,input wire sync_clear
);
 localparam BOOT=0,CLEAR=1,IDLE=2,ADDRESS=3,READ=4,LATCH=5,
            FLAGS=6,MERGE=7,WRITE=8,PUBLISH=9;
 reg [3:0] state;
 (* syn_ramstyle="block_ram" *) reg [31:0] words[0:2047];
 reg [31:0] ram_q,old_word,merged;
 reg [10:0] clear_address,update_address;
 reg [4:0] slot;
 reg [1:0] requested,old_code,resolved;
 reg active,available,publish_pending,read_tag;
 reg [4:0] read_slot;
 wire background=!foreground;
 wire clearing=SYNC_CLEAR && sync_clear;
 assign initializing=state==BOOT || state==CLEAR;
 assign cell_ready=!clearing && state==IDLE && active && !scan_bad && !publish_pending && !scan_begin;
 assign read_ready=!clearing && available && planner_lock && state!=READ && state!=PUBLISH && state!=BOOT;
 wire planner_read=read_valid && read_ready;
 wire updater_read=state==READ;
 wire [10:0] raddr=updater_read?update_address:{foreground,read_address[13:4]};
 wire write_en=state==BOOT || state==CLEAR || state==WRITE;
 wire [10:0] waddr=state==BOOT?clear_address:state==CLEAR?{background,clear_address[9:0]}:update_address;
 wire [31:0] wdata=state==WRITE?merged:32'b0;
 assign map_valid=available && !clearing && $unsigned(now_ms-map_ms)<250;
 always @(posedge clk)begin
  if(write_en)words[waddr]<=wdata;
  if(updater_read || planner_read)ram_q<=words[raddr];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=BOOT;clear_address<=0;foreground<=0;active<=0;available<=0;
   epoch<=0;map_ms<=0;scan_bad<=1;publish_pending<=0;cell_done<=0;
   read_done<=0;read_tag<=0;read_slot<=0;read_code<=0;
   update_address<=0;slot<=0;requested<=0;old_word<=0;old_code<=0;resolved<=0;merged<=0;
  end else if(clearing)begin
   state<=BOOT;clear_address<=0;foreground<=0;active<=0;available<=0;
   epoch<=0;map_ms<=0;scan_bad<=1;publish_pending<=0;cell_done<=0;
   read_done<=0;read_tag<=0;
  end else begin
   cell_done<=0;read_done<=read_tag;read_tag<=planner_read;
   if(planner_read)read_slot<={read_address[3:0],1'b0};
   if(read_tag)read_code<=(ram_q>>read_slot)&3;
   if(publish && active)publish_pending<=1;
   case(state)
    BOOT:if(clear_address==2047)begin clear_address<=0;state<=IDLE;end
         else clear_address<=clear_address+1'b1;
    CLEAR:if(clear_address==1023)begin clear_address<=0;state<=IDLE;end
          else clear_address<=clear_address+1'b1;
    IDLE:begin
     if(scan_begin)begin state<=CLEAR;clear_address<=0;active<=1;scan_bad<=0;publish_pending<=0;end
     else if(publish_pending)state<=PUBLISH;
     else if(cell_valid && cell_ready)begin
      update_address<={background,cell_address[13:4]};slot<={cell_address[3:0],1'b0};requested<=cell_code;state<=ADDRESS;
     end
    end
    ADDRESS:state<=READ;
    READ:state<=LATCH;
    LATCH:begin old_word<=ram_q;old_code<=(ram_q>>slot)&3;state<=FLAGS;end
    FLAGS:begin
     if(old_code[1])resolved<=old_code;
     else if(requested!=0)resolved<=requested;
     else resolved<=old_code;
     state<=MERGE;
    end
    MERGE:begin merged<=(old_word & ~(32'd3<<slot)) | ({30'd0,resolved}<<slot);state<=WRITE;end
    WRITE:begin cell_done<=1;state<=IDLE;end
    PUBLISH:if(!planner_lock && !read_tag && !read_done)begin
     if(active && !scan_bad && !scan_abort)begin foreground<=background;available<=1;epoch<=epoch+1'b1;map_ms<=now_ms;end
     active<=0;publish_pending<=0;state<=IDLE;
    end
    default:begin state<=BOOT;clear_address<=0;available<=0;active<=0;scan_bad<=1;end
   endcase
   if(scan_abort)scan_bad<=1;
  end
 end
endmodule
