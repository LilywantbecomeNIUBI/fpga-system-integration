// Network-domain freshness gate and latest-state event producer. This clock
// continues when camera PCLK stops, so a stopped camera cannot freeze a valid target.
module vision_v23_bridge #(parameter CLK_HZ=125000000,TIMEOUT_MS=500)(
    input clk,rst_n,stream_ready,semantic_update,semantic_valid,
    input [255:0] semantic_payload,input observation_valid,input [255:0] observation_payload,
    input event_ready,output reg event_valid,output reg [255:0] event_payload,
    output reg [3:0] led
);
    localparam LIMIT=(CLK_HZ/1000)*TIMEOUT_MS;
    localparam AGE_BITS=LIMIT<2 ? 1 : $clog2(LIMIT+1);
    reg [255:0] latest;reg have_observation,dirty,expired,ready_d,semantic_d;
    reg [15:0] semantic_fid;reg [AGE_BITS-1:0] age;reg deadline_due;
    wire expiring=!expired && deadline_due;
    wire change=observation_valid || semantic_update || expiring || ready_d!=stream_ready || semantic_d!=semantic_valid;
    wire [255:0] selected=observation_valid ? observation_payload : have_observation ? latest : 256'd0;
    wire [15:0] expected=semantic_update ? semantic_payload[207:192] : semantic_fid;
    wire usable=stream_ready && semantic_valid && (!(expired || expiring) || observation_valid) &&
        (have_observation || observation_valid) && selected[175:160]==expected;
    reg [255:0] masked;
    always @*begin
        masked=selected;masked[255:216]={32'h46535441,8'd2};
        if(!usable)begin
            masked[215:208]=(selected[215:208]&8'hf8)|8'h10;
            masked[199:192]=0;
            if((expired || expiring) && !observation_valid)masked[207:200]=9;
            else if(selected[175:160]!=expected)masked[207:200]=10;
        end
    end
    always @(posedge clk)begin
        if(observation_valid)latest<=observation_payload;
        if((!event_valid || event_ready) && (dirty || change))event_payload<=masked;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            have_observation<=0;dirty<=1;expired<=0;ready_d<=0;semantic_d<=0;semantic_fid<=0;age<=0;deadline_due<=0;event_valid<=0;led<=0;
        end else begin
            ready_d<=stream_ready;semantic_d<=semantic_valid;
            deadline_due<=age>=LIMIT-2;
            led<={masked[208] && masked[199:192]==3,masked[208] && masked[199:192]==2,
                masked[208] && masked[199:192]==1,masked[208]};
            if(!deadline_due)age<=age+1'b1;
            if(expiring)expired<=1;
            if(observation_valid)begin have_observation<=1;age<=0;expired<=0;deadline_due<=0;end
            if(semantic_update)semantic_fid<=semantic_payload[207:192];
            if(change)dirty<=1;
            if(event_valid && event_ready)event_valid<=0;
            if((!event_valid || event_ready) && (dirty || change))begin
                event_valid<=1;dirty<=0;
            end
        end
    end
endmodule
