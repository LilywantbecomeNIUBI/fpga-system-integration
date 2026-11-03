// Read-only display tap. Navigation consumes every parser point independently.
// One complete revolution -> 360 one-degree nearest returns, measured angle, at most 10 Hz.
// Sixty CRC-protected V2 chunks travel over Ethernet, never the 115200 UART.
module lidar_scan_360_source #(parameter integer CLK_HZ=50000000)(
    input wire clk,rst_n,byte_valid,framing_error,point_valid,
    input wire [15:0] angle,distance,
    input wire [7:0] intensity,
    input wire [31:0] packets,errors,crc_errors,format_errors,timeouts,
    input wire ready,
    output reg [383:0] frame,
    output reg valid
);
    // A single synchronous read port is shared by point updates and packing.
    // Bank 0 uses 0..359, bank 1 uses 512..871. Never reset the RAM itself.
    (* syn_ramstyle = "block_ram" *) reg [39:0] scan_ram [0:1023];
    reg bank_sel,snapshot_bank,clear_active;
    reg [8:0] clear_index;
    reg [39:0] ram_q;
    reg ram_re,ram_we;
    reg [9:0] ram_raddr,ram_waddr;
    reg [39:0] ram_wdata;

    // The parser emits twelve consecutive point_valid cycles per UART packet.
    // Queue them while the three-stage update/360-cycle bank clear is busy.
    // Store arrival time and error/loss epochs with each point, so queueing
    // cannot move an error into the wrong revolution. No FIFO data reset.
    (* syn_ramstyle = "block_ram" *) reg [135:0] point_fifo [0:31];
    reg [135:0] fifo_q,point_record;
    reg [4:0] fifo_wr,fifo_rd;
    reg [5:0] fifo_count;
    reg [31:0] input_loss_count,processed_loss,loss_base;
    wire input_point=point_valid && angle<16'd36000;
    wire fifo_push=input_point && fifo_count<6'd32;

    localparam [2:0] P_IDLE=0,P_LATCH=1,P_WAIT_CLEAR=2,P_READ=3,P_WRITE=4,P_BUFFER=5;
    reg [2:0] point_state;
    wire fifo_pop=point_state==P_IDLE && !clear_active && fifo_count!=0;
    wire [31:0] point_ms=point_record[135:104];
    wire [31:0] point_loss=point_record[103:72];
    wire [31:0] point_errors=point_record[71:40];
    wire [15:0] point_angle=point_record[39:24];
    wire [33:0] bin_product=point_angle*18'd167773;
    wire [15:0] point_bin=bin_product>>24;
    reg [8:0] bin_idx;
    reg [15:0] point_distance;
    reg [7:0] point_intensity;
    reg point_old_seen;

    reg [359:0] seen;
    reg [8:0] coverage,last_coverage;
    reg collecting,scan_bad,have_previous,transmitting,diag_pending;
    reg [15:0] previous_angle,scan_sequence;
    reg [31:0] ms_div,now_ms,last_point_ms,error_base,snapshot_ms,last_upload_ms;
    reg [31:0] byte_count,framing_count,completed,skipped;
    reg [5:0] chunk;
    reg [7:0] diag_ms;
    wire [16:0] delta=point_angle>=previous_angle ?
        {1'b0,point_angle}-{1'b0,previous_angle} :
        17'd36000+{1'b0,point_angle}-{1'b0,previous_angle};
    wire wrap=have_previous && point_angle<previous_angle &&
        previous_angle-point_angle>16'd18000;
    wire [31:0] age=now_ms-snapshot_ms;

    localparam [1:0] PK_IDLE=0,PK_READ=1,PK_CRC=2,PK_SEND=3;
    reg [1:0] pack_state;
    reg [8:0] pack_base,pack_addr;
    reg [3:0] pack_issued,pack_taken;
    reg [239:0] payload_shift;
    reg [127:0] pack_header;
    reg ram_pack_valid;
    reg [367:0] body;
    reg capture,is_scan;
    wire [383:0] checked_frame;
    wire checked_valid;
    wire point_read=point_state==P_READ;
    wire pack_read=pack_state==PK_READ && pack_issued<4'd6 &&
        transmitting && age<32'd400;
    wire pack_grant=pack_read && !point_read;
    // A timed-out CRC must drain before its snapshot bank can be reused.
    wire snapshot_busy=transmitting || (is_scan && pack_state!=PK_IDLE);
    wire point_update=point_state==P_WRITE && point_distance!=0 && point_intensity!=0 &&
        (!point_old_seen || ram_q[31:16]==0 || point_distance<ram_q[31:16]);

    frame_crc crc(.clk(clk),.reset_n(rst_n),.capture(capture),.body_in(body),
        .frame(checked_frame),.valid(checked_valid));

    // Canonical simple dual-port RAM templates: one write and one registered
    // read per clock, no asynchronous RAM read and no whole-array assignment.
    always @(posedge clk)begin
        if(rst_n)begin
            if(ram_we)scan_ram[ram_waddr]<=ram_wdata;
            if(ram_re)ram_q<=scan_ram[ram_raddr];
            if(fifo_push)point_fifo[fifo_wr]<={now_ms,input_loss_count,errors,angle,distance,intensity};
            if(fifo_pop)fifo_q<=point_fifo[fifo_rd];
            if(point_state==P_BUFFER)point_record<=fifo_q;
        end
    end

    always @*begin
        ram_re=0;ram_raddr=0;
        if(point_read)begin
            ram_re=1;ram_raddr={bank_sel,bin_idx};
        end else if(pack_grant)begin
            ram_re=1;ram_raddr={snapshot_bank,pack_addr};
        end
        ram_we=0;ram_waddr=0;ram_wdata=0;
        if(clear_active)begin
            ram_we=1;ram_waddr={bank_sel,clear_index};
        end else if(point_update)begin
            ram_we=1;ram_waddr={bank_sel,bin_idx};
            ram_wdata={point_intensity,point_distance,point_angle};
        end
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            fifo_wr<=0;fifo_rd<=0;fifo_count<=0;input_loss_count<=0;
        end else begin
            if(fifo_push)fifo_wr<=fifo_wr+1'b1;
            if(fifo_pop)fifo_rd<=fifo_rd+1'b1;
            case({fifo_push,fifo_pop})
                2'b10:fifo_count<=fifo_count+1'b1;
                2'b01:fifo_count<=fifo_count-1'b1;
                default:begin end
            endcase
            // With no input ready port, overload must invalidate a scan rather
            // than silently publish it. Full+pop also drops, avoiding a RAM
            // read/write collision at the same FIFO address.
            if(input_point && !fifo_push)input_loss_count<=input_loss_count+1'b1;
        end
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            bank_sel<=0;snapshot_bank<=0;clear_active<=1;clear_index<=0;
            point_state<=P_IDLE;bin_idx<=0;point_distance<=0;point_intensity<=0;point_old_seen<=0;
            processed_loss<=0;loss_base<=0;
            seen<=0;coverage<=0;last_coverage<=0;collecting<=0;scan_bad<=0;have_previous<=0;
            transmitting<=0;diag_pending<=0;previous_angle<=0;scan_sequence<=0;chunk<=0;
            ms_div<=0;diag_ms<=0;now_ms<=0;last_point_ms<=0;error_base<=0;snapshot_ms<=0;
            last_upload_ms<=32'hffffff9c;byte_count<=0;framing_count<=0;completed<=0;skipped<=0;
            pack_state<=PK_IDLE;pack_base<=0;pack_addr<=0;pack_issued<=0;pack_taken<=0;
            payload_shift<=0;pack_header<=0;ram_pack_valid<=0;
            body<=0;capture<=0;is_scan<=0;frame<=0;valid<=0;
        end else begin
            capture<=0;
            ram_pack_valid<=pack_grant;
            if(ms_div==CLK_HZ/1000-1)begin
                ms_div<=0;now_ms<=now_ms+1'b1;
                if(diag_ms==199)begin diag_ms<=0;diag_pending<=1;end
                else diag_ms<=diag_ms+1'b1;
            end else ms_div<=ms_div+1'b1;
            if(byte_valid)byte_count<=byte_count+1'b1;
            if(framing_error)framing_count<=framing_count+1'b1;

            if(clear_active)begin
                if(clear_index==9'd359)begin clear_active<=0;clear_index<=0;end
                else clear_index<=clear_index+1'b1;
            end
            // Do not time out points already accepted into the input queue.
            if(!input_point && fifo_count==0 && point_state==P_IDLE &&
                    have_previous && now_ms-last_point_ms>32'd250)begin
                collecting<=0;have_previous<=0;scan_bad<=1;
            end

            case(point_state)
                P_IDLE:if(fifo_pop)point_state<=P_BUFFER;
                P_BUFFER:point_state<=P_LATCH;
                P_LATCH:begin
                    // Stage 1: divide only into a register; no RAM mux or
                    // nearest-return comparator follows the divider this cycle.
                    bin_idx<=point_bin[8:0];
                    point_distance<=point_record[23:8];point_intensity<=point_record[7:0];
                    previous_angle<=point_angle;have_previous<=1;
                    last_point_ms<=point_ms;processed_loss<=point_loss;
                    if(wrap)begin
                        if(collecting && !scan_bad && point_errors==error_base &&
                                point_loss==loss_base && delta<=17'd1000 && coverage>=9'd300)begin
                            completed<=completed+1'b1;last_coverage<=coverage;
                            if(!snapshot_busy && point_ms-last_upload_ms>=32'd100)begin
                                // Publish by exchanging ownership, never by
                                // copying 360 words. A blocked snapshot keeps
                                // its bank; non-published turns reuse the writer.
                                snapshot_bank<=bank_sel;bank_sel<=!bank_sel;
                                snapshot_ms<=point_ms;last_upload_ms<=point_ms;
                                scan_sequence<=scan_sequence+1'b1;
                                chunk<=0;pack_base<=0;transmitting<=1;
                            end else skipped<=skipped+1'b1;
                        end
                        collecting<=1;scan_bad<=0;error_base<=point_errors;loss_base<=point_loss;
                        seen<=0;coverage<=0;clear_active<=1;clear_index<=0;
                        // Preserve the wrap point until the new writer is clear.
                        point_state<=P_WAIT_CLEAR;
                    end else if(collecting)begin
                        if(delta>17'd1000 || point_errors!=error_base || point_loss!=loss_base)scan_bad<=1;
                        point_state<=P_READ;
                    end else point_state<=P_IDLE;
                end
                P_WAIT_CLEAR:if(!clear_active)point_state<=P_READ;
                P_READ:begin
                    // Stage 2: synchronous read of the old bin, on the RAM port.
                    point_old_seen<=seen[bin_idx];point_state<=P_WRITE;
                end
                P_WRITE:begin
                    // Stage 3: compare ram_q and conditionally write one word.
                    // Serial updates also handle consecutive hits in one bin.
                    if(!point_old_seen)begin seen[bin_idx]<=1;coverage<=coverage+1'b1;end
                    point_state<=P_IDLE;
                end
                default:point_state<=P_IDLE;
            endcase

            case(pack_state)
                PK_IDLE:begin
                    if(diag_pending)begin
                        // V2 06: same counters, 16-bit coverage now up to 360.
                        body<=0;body[31:0]<=32'h02063cc3;body[47:32]<=scan_sequence;
                        body[55:48]<={3'b0,(completed!=0),transmitting,
                            (scan_bad || input_loss_count!=processed_loss),collecting,
                            (have_previous && now_ms-last_point_ms<32'd250)};
                        body[95:64]<=now_ms;body[127:96]<=byte_count;body[159:128]<=framing_count;
                        body[191:160]<=packets;body[223:192]<=crc_errors;body[255:224]<=format_errors;
                        body[287:256]<=timeouts;body[319:288]<=completed;body[351:320]<=skipped;
                        body[367:352]<={7'd0,last_coverage};
                        capture<=1;is_scan<=0;diag_pending<=0;pack_state<=PK_CRC;
                    end else if(transmitting && age<32'd400)begin
                        pack_header<={8'd1,8'd6,snapshot_ms,age[15:0],8'd60,{2'd0,chunk},scan_sequence,32'h02053cc3};
                        payload_shift<=0;pack_addr<=pack_base;pack_issued<=0;pack_taken<=0;
                        is_scan<=1;pack_state<=PK_READ;
                    end
                end
                PK_READ:begin
                    if(pack_grant)begin
                        pack_issued<=pack_issued+1'b1;pack_addr<=pack_addr+1'b1;
                    end
                    // One-cycle read tag distinguishes pack data from point
                    // data. Point reads may interrupt the six read grants.
                    if(ram_pack_valid)begin
                        payload_shift<={ram_q,payload_shift[239:40]};
                        if(pack_taken==4'd5)begin
                            body<={ram_q,payload_shift[239:40],pack_header};
                            capture<=1;pack_state<=PK_CRC;
                        end else pack_taken<=pack_taken+1'b1;
                    end
                end
                PK_CRC:if(checked_valid)begin
                    if(!is_scan || (transmitting && age<32'd400))begin
                        frame<=checked_frame;valid<=1;pack_state<=PK_SEND;
                    end else pack_state<=PK_IDLE;
                end
                PK_SEND:if(valid && ready)begin
                    valid<=0;pack_state<=PK_IDLE;
                    if(is_scan)begin
                        if(chunk==6'd59)transmitting<=0;
                        else begin chunk<=chunk+1'b1;pack_base<=pack_base+9'd6;end
                    end
                end
            endcase

            // Abort reads/stalled output, but let an in-flight CRC finish and
            // discard it. Otherwise a late checked_valid can revive stale data.
            if(transmitting && age>=32'd400)begin
                transmitting<=0;
                if(is_scan)begin
                    valid<=0;
                    if(pack_state==PK_READ || pack_state==PK_SEND)begin
                        capture<=0;pack_state<=PK_IDLE;
                    end
                end
            end
        end
    end
endmodule
