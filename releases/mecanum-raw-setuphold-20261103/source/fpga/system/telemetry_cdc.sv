// Lossless, backpressured bundled-data mailbox. One immutable complete frame.
// Both domains reset from the same network reset condition. The held bus stays
// stable until the Ethernet consumer acknowledges it; only toggles are sampled.
module telemetry_cdc #(parameter integer CLK_HZ=50000000)(
    input wire src_clk,src_rst_n,src_valid,
    input wire [383:0] src_frame,output wire src_ready,
    input wire dst_clk,dst_rst_n,dst_ready,
    output reg dst_valid,output reg [511:0] dst_payload
);
    reg request,ack;
    (* async_reg="true", syn_preserve=1 *) reg ack_meta,ack_sync,request_meta,request_sync;
    reg [511:0] held;
    reg [31:0] sequence_id,device_ms,ms_div;
    wire [383:0] wire_frame;
    genvar i;
    generate for(i=0;i<48;i=i+1)begin:byte_order
        assign wire_frame[383-i*8-:8]=src_frame[i*8+:8];
    end endgenerate
    assign src_ready=src_rst_n && request==ack_sync;
    always @(posedge src_clk or negedge src_rst_n)begin
        if(!src_rst_n)begin
            request<=0;ack_meta<=0;ack_sync<=0;held<=0;
            sequence_id<=0;device_ms<=0;ms_div<=0;
        end else begin
            ack_meta<=ack;ack_sync<=ack_meta;
            if(ms_div==CLK_HZ/1000-1)begin ms_div<=0;device_ms<=device_ms+1'b1;end
            else ms_div<=ms_div+1'b1;
            if(src_valid && src_ready)begin
                // TLM1, version, three reserved bytes, BE sequence/uptime, then
                // the unchanged 48-byte CRC-protected mixed telemetry frame.
                held<={32'h544c4d31,8'd1,24'd0,sequence_id,device_ms,wire_frame};
                sequence_id<=sequence_id+1'b1;request<=!request;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n)begin
        if(!dst_rst_n)begin
            ack<=0;request_meta<=0;request_sync<=0;dst_valid<=0;dst_payload<=0;
        end else begin
            request_meta<=request;request_sync<=request_meta;
            if(!dst_valid && request_sync!=ack)begin dst_payload<=held;dst_valid<=1;end
            if(dst_valid && dst_ready)begin dst_valid<=0;ack<=request_sync;end
        end
    end
endmodule
