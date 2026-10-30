// Independent network-clock watchdog. Continues to stop/mask control when PCLK stops.
// One immutable output plus newest diagnostic vector. OFST status has priority.
module flow_event_bridge #(parameter CLK_HZ=125000000,TIMEOUT_MS=100,STAGE=2)(
    input clk,rst_n,stream_ready,observation_valid,input [255:0] observation_payload,
    input event_ready,output reg event_valid,output reg [255:0] event_payload,
    output stop_request,output signed [7:0] steer,output [7:0] speed_limit,output [2:0] control_state
);
    localparam LIMIT=(CLK_HZ/1000)*TIMEOUT_MS;
    localparam [7:0] STAGE_BYTE=STAGE;
    reg [255:0] latest,vector;reg have_status,expired,status_dirty,vector_dirty,ready_d;
    reg [31:0] age;reg deadline_due;
    wire status_in=observation_valid && observation_payload[255:224]==32'h4f465354;
    wire vector_in=observation_valid && observation_payload[255:224]==32'h4f464c57;
    wire expiring=!expired && deadline_due;
    wire pending=status_dirty || status_in || expiring || ready_d!=stream_ready || vector_dirty || vector_in;
    wire [255:0] selected=status_in ? observation_payload : latest;
    wire usable=stream_ready && (have_status || status_in) && (!(expired || expiring) || status_in) && selected[201];
    reg [255:0] masked;
    always @*begin
        masked=selected;
        if(!usable)begin
            masked[207:200]=8'h21; // stop + stale/unavailable, zero hazard/confidence
            masked[199:192]=0;masked[79:64]=0;
        end
    end
    assign stop_request=!usable || selected[200];
    assign steer=usable ? selected[71:64] : 8'd0;
    assign speed_limit=usable ? selected[79:72] : 8'd0;
    assign control_state=usable ? selected[194:192] : 3'd0;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            latest<={32'h4f465354,8'd1,STAGE_BYTE,8'h21,8'd0,192'd0};vector<=0;
            have_status<=0;expired<=1;status_dirty<=1;vector_dirty<=0;ready_d<=0;age<=0;deadline_due<=0;event_valid<=0;event_payload<=0;
        end else begin
            ready_d<=stream_ready;
            // Register the comparator one tick early, preserving the deadline
            // while removing a counter-to-256-bit-mask combinational path.
            deadline_due<=age>=LIMIT-2;
            if(!deadline_due)age<=age+1'b1;
            if(expiring)begin expired<=1;status_dirty<=1;end
            if(ready_d!=stream_ready)status_dirty<=1;
            if(status_in)begin latest<=observation_payload;have_status<=1;expired<=0;age<=0;deadline_due<=0;status_dirty<=1;end
            if(vector_in)begin vector<=observation_payload;vector_dirty<=1;end
            // Keep ready-to-valid logic independent of the wide payload enable.
            event_valid<=(event_valid && !event_ready) || pending;
            if(!event_valid || event_ready)begin
                if(status_dirty || status_in || expiring || ready_d!=stream_ready)begin
                    event_payload<=masked;status_dirty<=0;
                end else if(vector_dirty || vector_in)begin
                    event_payload<=vector_in ? observation_payload : vector;vector_dirty<=0;
                end
            end
        end
    end
endmodule
