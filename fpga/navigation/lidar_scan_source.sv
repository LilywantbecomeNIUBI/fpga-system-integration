// Read-only display tap. Navigation consumes every parser point independently.
// One complete revolution -> 72 nearest-return 5-degree bins, at most 5 Hz.
// Six CRC-protected 48-byte chunks fit the existing 115200-baud mixed UART.
module lidar_scan_source #(parameter integer CLK_HZ=50000000)(
    input wire clk,rst_n,byte_valid,framing_error,point_valid,
    input wire [15:0] angle,distance,
    input wire [7:0] intensity,
    input wire [31:0] packets,errors,crc_errors,format_errors,timeouts,
    input wire ready,
    output reg [383:0] frame,
    output reg valid
);
    reg [23:0] working[0:71],snapshot[0:71];
    reg [71:0] seen;
    reg [6:0] coverage,last_coverage;
    reg collecting,scan_bad,have_previous,transmitting,diag_pending;
    reg [15:0] previous_angle,scan_sequence;
    reg [31:0] ms_div,now_ms,last_point_ms,error_base,snapshot_ms,last_upload_ms;
    reg [31:0] byte_count,framing_count,completed,skipped;
    reg [2:0] chunk;reg [7:0] diag_ms;
    reg [367:0] body;
    reg capture,crc_busy,is_scan;
    wire [383:0] checked_frame;wire checked_valid;
    wire [6:0] bin=angle/16'd500;
    wire [16:0] delta=angle>=previous_angle ? {1'b0,angle}-{1'b0,previous_angle} :
        17'd36000+{1'b0,angle}-{1'b0,previous_angle};
    wire wrap=have_previous && angle<previous_angle && previous_angle-angle>16'd18000;
    wire [31:0] age=now_ms-snapshot_ms;
    integer i;
    frame_crc crc(.clk(clk),.reset_n(rst_n),.capture(capture),.body_in(body),.frame(checked_frame),.valid(checked_valid));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            seen<=0;coverage<=0;last_coverage<=0;collecting<=0;scan_bad<=0;have_previous<=0;
            transmitting<=0;diag_pending<=0;previous_angle<=0;scan_sequence<=0;chunk<=0;
            ms_div<=0;diag_ms<=0;now_ms<=0;last_point_ms<=0;error_base<=0;snapshot_ms<=0;
            last_upload_ms<=32'hffffff38;byte_count<=0;framing_count<=0;completed<=0;skipped<=0;
            body<=0;capture<=0;crc_busy<=0;is_scan<=0;frame<=0;valid<=0;
            for(i=0;i<72;i=i+1)begin working[i]<=0;snapshot[i]<=0;end
        end else begin
            capture<=0;
            if(ms_div==CLK_HZ/1000-1)begin
                ms_div<=0;now_ms<=now_ms+1'b1;
                if(diag_ms==199)begin diag_ms<=0;diag_pending<=1;end
                else diag_ms<=diag_ms+1'b1;
            end else ms_div<=ms_div+1'b1;
            if(byte_valid)byte_count<=byte_count+1'b1;
            if(framing_error)framing_count<=framing_count+1'b1;
            if(!point_valid && have_previous && now_ms-last_point_ms>250)begin
                collecting<=0;have_previous<=0;scan_bad<=1;
            end
            if(point_valid && angle<36000)begin
                previous_angle<=angle;have_previous<=1;last_point_ms<=now_ms;
                if(wrap)begin
                    if(collecting && !scan_bad && errors==error_base && delta<=1000 && coverage>=60)begin
                        completed<=completed+1'b1;last_coverage<=coverage;
                        if(!transmitting && now_ms-last_upload_ms>=200)begin
                            for(i=0;i<72;i=i+1)snapshot[i]<=working[i];
                            snapshot_ms<=now_ms;last_upload_ms<=now_ms;
                            scan_sequence<=scan_sequence+1'b1;chunk<=0;transmitting<=1;
                        end else skipped<=skipped+1'b1;
                    end
                    collecting<=1;scan_bad<=0;error_base<=errors;seen<=0;coverage<=1;
                    for(i=0;i<72;i=i+1)working[i]<=0;
                    seen[bin]<=1;
                    working[bin]<=distance!=0 && intensity!=0 ? {intensity,distance} : 24'd0;
                end else if(collecting)begin
                    if(delta>1000 || errors!=error_base)scan_bad<=1;
                    if(!seen[bin])begin seen[bin]<=1;coverage<=coverage+1'b1;end
                    if(distance!=0 && intensity!=0 && (!seen[bin] || working[bin][15:0]==0 || distance<working[bin][15:0]))
                        working[bin]<={intensity,distance};
                end
            end
            if(valid && ready)begin
                valid<=0;
                if(is_scan)begin
                    if(chunk==5)transmitting<=0;
                    else chunk<=chunk+1'b1;
                end
            end
            if(!crc_busy && !valid)begin
                if(diag_pending)begin
                    // 06: flags, device time and independent reception-stage counters.
                    body<=0;body[31:0]<=32'h01063cc3;body[47:32]<=scan_sequence;
                    body[55:48]<={3'b0,(completed!=0),transmitting,scan_bad,collecting,
                        (have_previous && now_ms-last_point_ms<250)};
                    body[95:64]<=now_ms;body[127:96]<=byte_count;body[159:128]<=framing_count;
                    body[191:160]<=packets;body[223:192]<=crc_errors;body[255:224]<=format_errors;
                    body[287:256]<=timeouts;body[319:288]<=completed;body[351:320]<=skipped;
                    body[367:352]<={9'd0,last_coverage};
                    capture<=1;crc_busy<=1;is_scan<=0;diag_pending<=0;
                end else if(transmitting && age<400)begin
                    body<=0;body[31:0]<=32'h01053cc3;body[47:32]<=scan_sequence;
                    body[55:48]<={5'd0,chunk};body[63:56]<=8'd6;body[79:64]<=age[15:0];
                    for(i=0;i<12;i=i+1)body[80+i*24+:24]<=snapshot[chunk*12+i];
                    capture<=1;crc_busy<=1;is_scan<=1;
                end
            end
            if(checked_valid)begin frame<=checked_frame;valid<=1;crc_busy<=0;end
            // A stopped consumer cannot retain an old scan indefinitely.
            if(transmitting && age>=400)begin
                transmitting<=0;
                if(is_scan)valid<=0;
            end
        end
    end
endmodule
