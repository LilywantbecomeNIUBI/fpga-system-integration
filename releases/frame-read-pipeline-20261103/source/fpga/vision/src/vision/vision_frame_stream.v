// Two complete-frame slots; Gray-pointer ownership reused from camera_jpeg_stream.
// Capture never waits: when both slots are occupied, discard the entire new frame.
// Letterbox rows are generated from a completed immutable RGB565 image.
module vision_frame_stream #(
    parameter WIDTH=320,ACTIVE_HEIGHT=180,CANVAS_HEIGHT=320,PAD_PIXEL=16'h738e
)(
    input wr_clk,wr_rst_n,frame_start,frame_end,frame_good,
    input [15:0] frame_id,pixel,input pixel_valid,
    input rd_clk,rd_rst_n,payload_read,tx_done,
    output reg [7:0] payload_data,output [15:0] payload_length,packet_frame_id,
    output reg packet_first,output reg tx_start,output reg [31:0] dropped_frames
);
    localparam AB=$clog2(WIDTH*ACTIVE_HEIGHT),TOTAL=WIDTH*ACTIVE_HEIGHT;
    localparam TOP=(CANVAS_HEIGHT-ACTIVE_HEIGHT)/2;
    localparam [15:0] IMAGE_W=WIDTH,IMAGE_H=CANVAS_HEIGHT;
    reg [15:0] descriptors[0:1];
    reg [1:0] wptr,wgray,rptr,rgray,rs0,rs1,ws0,ws1;
    wire [1:0] wn=wptr+1'b1,rn=rptr+1'b1;
    wire [1:0] allocated_gray=wptr^(wptr>>1);
    wire full=allocated_gray==~rs1,empty=rgray==ws1;
    reg active,drop;reg [AB:0] written;reg [15:0] write_fid;
    // Two aligned command stages. Four independent address banks keep each
    // registered write-enable/address/data group near its own BSRAMs.
    localparam BANK_BITS=AB>=2 ? 2 : AB;
    localparam BANKS=1<<BANK_BITS,BANK_AB=AB+1-BANK_BITS;
    localparam BANK_SEL_BITS=BANK_BITS>0 ? BANK_BITS : 1;
    reg wr_pending;reg [AB:0] wr_address;reg [15:0] wr_pixel;
    reg [AB-1:0] frame_address;
    wire [AB:0] read_address={rptr[0],frame_address};
    wire [BANKS*16-1:0] read_words;
    reg [BANK_SEL_BITS-1:0] read_bank,read_bank_pipe;
    wire [15:0] ram_word=read_words[read_bank_pipe*16+:16];
    always @(posedge wr_clk)begin
        wr_address<={wptr[0],written[AB-1:0]};wr_pixel<=pixel;
    end
    always @(posedge wr_clk or negedge wr_rst_n)
        if(!wr_rst_n)wr_pending<=0;
        else wr_pending<=pixel_valid && active && !drop && written<TOTAL;
    always @(posedge rd_clk)begin
        read_bank<=read_address>>BANK_AB;
        read_bank_pipe<=read_bank;
    end
    genvar bank;
    generate for(bank=0;bank<BANKS;bank=bank+1)begin:storage_bank
        (* syn_ramstyle="block_ram" *) reg [15:0] memory[0:(1<<BANK_AB)-1];
        (* syn_preserve=1, syn_keep=1 *) reg write_enable;
        reg [BANK_AB-1:0] write_address;
        reg [15:0] write_pixel,read_pixel;
        // Preserve a separate register in each bank before the shared mux:
        // BSRAM -> local register -> bank mux -> cached_word. No reset is
        // needed on payload flops; the fill-valid pipeline masks stale data.
        (* syn_preserve=1, syn_keep=1 *) reg [15:0] read_pixel_pipe;
        wire selected=wr_pending && (wr_address>>BANK_AB)==bank;
        always @(posedge wr_clk or negedge wr_rst_n)
            if(!wr_rst_n)write_enable<=0;else write_enable<=selected;
        always @(posedge wr_clk)begin
            if(selected)begin write_address<=wr_address[BANK_AB-1:0];write_pixel<=wr_pixel;end
            if(write_enable)memory[write_address]<=write_pixel;
        end
        always @(posedge rd_clk)begin
            read_pixel<=memory[read_address[BANK_AB-1:0]];
            read_pixel_pipe<=read_pixel;
        end
        assign read_words[bank*16+:16]=read_pixel_pipe;
    end endgenerate
    // Allocation advances immediately, but publish ownership only after the
    // final pipelined RAM write. This also covers back-to-back tiny frames and
    // arbitrary destination/source clock ratios.
    reg [1:0] commit_valid,commit_gray0,commit_gray1;
    wire frame_commit=frame_end && active && frame_good && !drop && written==TOTAL;
    always @(posedge wr_clk or negedge wr_rst_n)begin
        if(!wr_rst_n)begin commit_valid<=0;commit_gray0<=0;commit_gray1<=0;end
        else begin
            commit_valid<={commit_valid[0],frame_commit};
            if(frame_commit)commit_gray0<=wn^(wn>>1);
            commit_gray1<=commit_gray0;
        end
    end
    always @(posedge wr_clk or negedge wr_rst_n)begin
        if(!wr_rst_n)begin wptr<=0;wgray<=0;rs0<=0;rs1<=0;active<=0;drop<=0;written<=0;write_fid<=0;dropped_frames<=0;end
        else begin
            rs0<=rgray;rs1<=rs0;
            if(commit_valid[1])wgray<=commit_gray1;
            if(frame_start)begin active<=1;drop<=full;written<=0;write_fid<=frame_id;end
            if(pixel_valid && active && !drop)begin
                if(written<TOTAL)written<=written+1'b1;else drop<=1;
            end
            if(frame_end && active)begin
                active<=0;
                if(frame_good && !drop && written==TOTAL)begin descriptors[wptr[0]]<=write_fid;wptr<=wn;end
                else dropped_frames<=dropped_frames+1'b1;
            end
        end
    end
    localparam IDLE=0,LOAD=1,ACTIVE=2,GAP=3,FILL=4,DRAIN=5;
    localparam CB=$clog2(WIDTH);
    reg [2:0] state;reg [4:0] gap;reg [15:0] fid,row,index;
    reg [15:0] cached_word;
    reg [15:0] line_ram[0:(1<<CB)-1];reg [15:0] line_word,line_ram_dout_r;
    reg [CB:0] fill_count;reg [CB-1:0] fill_index0,fill_index1,fill_index2,word_index;
    reg [2:0] fill_valid;reg central,pixel_phase;
    wire fill_request=state==FILL;
    // Keep the next word at the RAM output throughout both byte phases. The
    // extra output register advances only after the current high byte is read,
    // so even a stall between the two bytes cannot replace the current pixel.
    wire [CB-1:0] line_prefetch=word_index+pixel_phase;
    always @(posedge rd_clk)begin
        cached_word<=ram_word;
        if(fill_valid[2])line_ram[fill_index2]<=cached_word;
        line_word<=line_ram[line_prefetch];
        if(!pixel_phase || (payload_read && index[0]))line_ram_dout_r<=line_word;
    end
    assign packet_frame_id=fid;assign payload_length=16+WIDTH*2;
    reg [7:0] header;
    wire [15:0] next_index=index+1'b1;
    wire next_phase=pixel_phase || index==15;
    wire [15:0] next_word=pixel_phase && index[0] ? line_word : line_ram_dout_r;
    always @*begin
        case(next_index[3:0])
            0:header="C";1:header="V";2:header=2;3:header=1;
            4:header=fid[7:0];5:header=fid[15:8];6:header=row[7:0];7:header=row[15:8];
            8:header=0;9:header=0;10:header=IMAGE_W[7:0];11:header=IMAGE_W[15:8];
            12:header=IMAGE_W[7:0];13:header=IMAGE_W[15:8];14:header=IMAGE_H[7:0];15:header=IMAGE_H[15:8];
        endcase
    end
    // The RAM output register separates the BSRAM read from the UDP byte mux.
    // One-byte lookahead stage. Consume replaces the byte; a stall retains it.
    // LOAD primes byte zero before the receiver sees tx_start.
    always @(posedge rd_clk)
        if(state==LOAD)payload_data<="C";
        else if(payload_read)payload_data<=!next_phase ? header : central ?
            (next_index[0] ? next_word[15:8] : next_word[7:0]) :
            (next_index[0] ? PAD_PIXEL[15:8] : PAD_PIXEL[7:0]);
    always @(posedge rd_clk or negedge rd_rst_n)begin
        if(!rd_rst_n)begin
            rptr<=0;rgray<=0;ws0<=0;ws1<=0;state<=IDLE;gap<=0;fid<=0;row<=0;index<=0;frame_address<=0;
            tx_start<=0;packet_first<=0;central<=0;pixel_phase<=0;fill_count<=0;fill_index0<=0;fill_index1<=0;fill_index2<=0;fill_valid<=0;word_index<=0;
        end
        else begin
            ws0<=wgray;ws1<=ws0;tx_start<=0;
            fill_valid<={fill_valid[1:0],fill_request};
            fill_index0<=fill_count[CB-1:0];fill_index1<=fill_index0;fill_index2<=fill_index1;
            if(fill_request)frame_address<=frame_address+1'b1;
            if(payload_read)begin
                index<=index+1'b1;if(index==15)pixel_phase<=1;
                if(pixel_phase && index[0])word_index<=word_index+1'b1;
            end
            case(state)
                IDLE:if(!empty)begin
                    packet_first<=1;
                    fid<=descriptors[rptr[0]];row<=0;index<=0;word_index<=0;pixel_phase<=0;central<=TOP==0;frame_address<=0;fill_count<=0;
                    state<=TOP==0 ? FILL : LOAD;
                end
                FILL:if(fill_count==WIDTH-1)state<=DRAIN;else fill_count<=fill_count+1'b1;
                DRAIN:if(fill_valid==0)state<=LOAD;
                LOAD:begin tx_start<=1;state<=ACTIVE;end
                ACTIVE:if(tx_done)begin state<=GAP;gap<=0;end
                GAP:if(gap==15)begin
                    if(row==CANVAS_HEIGHT-1)begin rptr<=rn;rgray<=rn^(rn>>1);state<=IDLE;end
                    else begin
                        packet_first<=0;
                        row<=row+1'b1;index<=0;word_index<=0;pixel_phase<=0;fill_count<=0;
                        central<=row>=TOP-1 && row<TOP+ACTIVE_HEIGHT-1;
                        state<=row>=TOP-1 && row<TOP+ACTIVE_HEIGHT-1 ? FILL : LOAD;
                    end
                end else gap<=gap+1'b1;
            endcase
        end
    end
endmodule
