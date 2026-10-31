// Diagnostic 32-byte packets. Sequence numbers expose telemetry coalescing/loss.
// Control decisions never depend on receiving every diagnostic vector packet.
module flow_event_source #(parameter STAGE=2,WIDTH=160,HEIGHT=90)(
    input clk,rst_n,frame_begin,sample_valid,sample_good,status_valid,
    input [15:0] frame_id,x,y,input signed [15:0] u,v,
    input [2:0] state,hazard,input stop_request,input signed [1:0] steer,input [7:0] speed_limit,
    input [15:0] left_valid,center_valid,right_valid,left_risk,center_risk,right_risk,
    input [31:0] cycles,dropped_frames,
    output reg event_valid,output reg [255:0] event_payload
);
    reg [15:0] sequence_index;
    localparam [7:0] VERSION=1,STAGE_BYTE=STAGE;
    localparam [15:0] W=WIDTH,H=HEIGHT;
    wire [7:0] flags={3'd0,hazard,state!=0,stop_request};
    wire [7:0] steering={{6{steer[1]}},steer};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin sequence_index<=0;event_valid<=0;event_payload<=0;end
        else begin
            event_valid<=0;
            if(frame_begin)sequence_index<=0;
            if(sample_valid)begin
                event_valid<=1;sequence_index<=sequence_index+1'b1;
                event_payload<={32'h4f464c57,VERSION,STAGE_BYTE,7'd0,sample_good,8'd0,
                    frame_id,x,y,u,v,sequence_index,W,H,cycles,dropped_frames};
            end
            if(status_valid)begin
                event_valid<=1;
                event_payload<={32'h4f465354,VERSION,STAGE_BYTE,flags,5'd0,state,
                    frame_id,left_valid,center_valid,right_valid,left_risk,center_risk,right_risk,speed_limit,steering,cycles,dropped_frames};
            end
        end
    end
endmodule
