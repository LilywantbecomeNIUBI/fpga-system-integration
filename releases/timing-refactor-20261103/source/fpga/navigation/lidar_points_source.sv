// All twelve accepted parser points are forwarded unchanged, including invalid
// returns. Two six-point CRC frames per native packet; no angular binning.
// Eight packet slots, 32 words each. Only eighteen 32-bit words are used per
// slot; narrow synchronous RAM avoids a wide/shallow flip-flop FIFO.
module lidar_points_source #(parameter integer CLK_HZ=50000000)(
    input wire clk,rst_n,byte_valid,framing_error,point_valid,
    input wire [3:0] point_index,
    input wire [15:0] angle,distance,input wire [7:0] intensity,
    input wire [31:0] packets,errors,crc_errors,format_errors,timeouts,
    input wire ready,output reg [383:0] frame,output reg valid
);
    (* syn_ramstyle="block_ram" *) reg [31:0] packet_ram[0:255];
    reg [2:0] wr_slot,rd_slot;reg [3:0] queue_count;
    reg [31:0] ms_div,now_ms,byte_count,framing_count,dropped,forwarded;
    reg [7:0] diag_ms;reg diag_pending;
    reg [31:0] previous_errors,capture_sequence,capture_ms;
    reg [479:0] capture_points,write_points;
    reg capture_active,capture_error;
    reg [3:0] expected_index;
    reg writer_busy;reg [4:0] write_index;
    reg [31:0] write_sequence,write_ms;reg write_error;
    wire commit=writer_busy && write_index==17;
    reg ram_re;reg [7:0] ram_addr;reg [31:0] ram_q;
    wire [7:0] write_addr={wr_slot,write_index};
    wire [31:0] write_data=write_index==0 ? write_sequence :
        write_index==1 ? write_ms : write_index==2 ? {31'd0,write_error} : write_points[31:0];
    localparam R_IDLE=0,R_READ=1,R_TAKE=2,R_BUILD=3,R_CRC=4,R_SEND=5;
    reg [2:0] reader_state;reg [4:0] read_index;
    reg [31:0] active_sequence,active_ms;reg active_error,half;
    reg [479:0] active_points;
    wire pop=reader_state==R_TAKE && read_index==17;
    wire [31:0] age=now_ms-active_ms;
    reg capture_crc,is_diagnostic;
    reg [367:0] body;wire [383:0] checked_frame;wire checked_valid;
    wire sample_matches=point_index==expected_index && packets==capture_sequence && angle<36000;
    wire abandoned_input=point_valid && capture_active &&
        ((point_index==0) || (point_index!=0 && !sample_matches));
    wire overloaded_input=point_valid && capture_active && point_index==11 && sample_matches &&
        (writer_busy || queue_count>=8);
    wire expired_packet=(reader_state==R_BUILD && age>=400) ||
        (reader_state==R_CRC && checked_valid && !is_diagnostic && age>=400) ||
        (reader_state==R_SEND && valid && !ready && !is_diagnostic && age>=400);
    wire [2:0] drop_events={2'd0,abandoned_input}+{2'd0,overloaded_input}+{2'd0,expired_packet};
    frame_crc crc(.clk(clk),.reset_n(rst_n),.capture(capture_crc),.body_in(body),
        .frame(checked_frame),.valid(checked_valid));
    always @*begin
        ram_re=reader_state==R_READ;ram_addr={rd_slot,read_index};
    end
    always @(posedge clk)begin
        if(rst_n)begin
            if(writer_busy)packet_ram[write_addr]<=write_data;
            if(ram_re)ram_q<=packet_ram[ram_addr];
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            wr_slot<=0;rd_slot<=0;queue_count<=0;
        end else begin
            if(commit)wr_slot<=wr_slot+1'b1;
            if(pop)rd_slot<=rd_slot+1'b1;
            case({commit,pop})
                2'b10:queue_count<=queue_count+1'b1;
                2'b01:queue_count<=queue_count-1'b1;
                default:begin end
            endcase
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            ms_div<=0;now_ms<=0;byte_count<=0;framing_count<=0;dropped<=0;forwarded<=0;
            diag_ms<=0;diag_pending<=0;previous_errors<=0;capture_sequence<=0;capture_ms<=0;
            capture_points<=0;write_points<=0;capture_active<=0;capture_error<=0;expected_index<=0;
            writer_busy<=0;write_index<=0;write_sequence<=0;write_ms<=0;write_error<=0;
            reader_state<=R_IDLE;read_index<=0;active_sequence<=0;active_ms<=0;
            active_error<=0;half<=0;active_points<=0;capture_crc<=0;is_diagnostic<=0;
            body<=0;frame<=0;valid<=0;
        end else begin
            capture_crc<=0;
            if(ms_div==CLK_HZ/1000-1)begin
                ms_div<=0;now_ms<=now_ms+1'b1;
                if(diag_ms==199)begin diag_ms<=0;diag_pending<=1;end
                else diag_ms<=diag_ms+1'b1;
            end else ms_div<=ms_div+1'b1;
            if(byte_valid)byte_count<=byte_count+1'b1;
            if(framing_error)framing_count<=framing_count+1'b1;
            if(writer_busy)begin
                if(write_index>=3)write_points<=write_points>>32;
                if(commit)begin writer_busy<=0;write_index<=0;end
                else write_index<=write_index+1'b1;
            end
            if(point_valid)begin
                if(point_index==0)begin
                    capture_points<={{intensity,distance,angle},440'd0};
                    capture_active<=1;expected_index<=1;capture_sequence<=packets;capture_ms<=now_ms;
                    capture_error<=errors!=previous_errors;previous_errors<=errors;
                end else if(capture_active && point_index==expected_index && packets==capture_sequence && angle<36000)begin
                    capture_points<={{intensity,distance,angle},capture_points[479:40]};
                    expected_index<=expected_index+1'b1;
                    if(point_index==11)begin
                        capture_active<=0;
                        if(!writer_busy && queue_count<8)begin
                            write_points<={{intensity,distance,angle},capture_points[479:40]};
                            write_sequence<=capture_sequence;write_ms<=capture_ms;
                            write_error<=capture_error || errors!=previous_errors;
                            writer_busy<=1;write_index<=0;
                        end
                    end
                end else if(capture_active)capture_active<=0;
            end
            case(reader_state)
                R_IDLE:begin
                    if(diag_pending)begin
                        // 06 v3: last words are forwarded/dropped packet counts;
                        // the final uint16 is queued complete packets (0..8).
                        body<=0;body[31:0]<=32'h03063cc3;
                        body[47:32]<=packets[15:0];body[55:48]<=8'd1;
                        body[95:64]<=now_ms;body[127:96]<=byte_count;body[159:128]<=framing_count;
                        body[191:160]<=packets;body[223:192]<=crc_errors;body[255:224]<=format_errors;
                        body[287:256]<=timeouts;body[319:288]<=forwarded;body[351:320]<=dropped;
                        body[367:352]<={12'd0,queue_count};
                        is_diagnostic<=1;capture_crc<=1;diag_pending<=0;reader_state<=R_CRC;
                    end else if(queue_count!=0)begin read_index<=0;reader_state<=R_READ;end
                end
                R_READ:reader_state<=R_TAKE;
                R_TAKE:begin
                    case(read_index)
                        0:active_sequence<=ram_q;
                        1:active_ms<=ram_q;
                        2:active_error<=ram_q[0];
                        default:active_points<={ram_q,active_points[479:32]};
                    endcase
                    if(read_index==17)begin half<=0;reader_state<=R_BUILD;end
                    else begin read_index<=read_index+1'b1;reader_state<=R_READ;end
                end
                R_BUILD:begin
                    if(age>=400)reader_state<=R_IDLE;
                    else begin
                        body<={half?active_points[479:240]:active_points[239:0],
                            {7'd0,active_error},{7'd0,half},active_ms,age[15:0],active_sequence,32'h01073cc3};
                        is_diagnostic<=0;capture_crc<=1;reader_state<=R_CRC;
                    end
                end
                R_CRC:if(checked_valid)begin
                    if(is_diagnostic || age<400)begin frame<=checked_frame;valid<=1;reader_state<=R_SEND;end
                    else reader_state<=R_IDLE;
                end
                R_SEND:begin
                    if(valid && ready)begin
                        valid<=0;
                        if(is_diagnostic)reader_state<=R_IDLE;
                        else if(!half)begin half<=1;reader_state<=R_BUILD;end
                        else begin forwarded<=forwarded+1'b1;reader_state<=R_IDLE;end
                    end else if(!is_diagnostic && age>=400)begin
                        valid<=0;reader_state<=R_IDLE;
                    end
                end
                default:reader_state<=R_IDLE;
            endcase
            if(drop_events!=0)dropped<=dropped+drop_events;
        end
    end
endmodule
