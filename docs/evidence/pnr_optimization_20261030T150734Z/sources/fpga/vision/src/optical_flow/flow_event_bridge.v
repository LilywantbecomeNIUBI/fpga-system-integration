// Independent network-clock watchdog. Continues to stop/mask control when PCLK stops.
// One immutable output plus newest diagnostic vector. OFST status has priority.
module flow_event_bridge #(parameter CLK_HZ=125000000,TIMEOUT_MS=100,STAGE=2)(
    input clk,rst_n,stream_ready,observation_valid,input [255:0] observation_payload,
    input event_ready,output event_valid,output [255:0] event_payload,
    output stop_request,output signed [7:0] steer,output [7:0] speed_limit,output [2:0] control_state
);
    localparam LIMIT=(CLK_HZ/1000)*TIMEOUT_MS;
    localparam [7:0] STAGE_BYTE=STAGE;
    reg [255:0] latest,vector;reg have_status,expired,status_dirty,vector_dirty,ready_d;
    localparam AGE_BITS=LIMIT<2 ? 1 : $clog2(LIMIT+1);
    reg [AGE_BITS-1:0] age;reg deadline_due;
    wire status_in=observation_valid && observation_payload[255:224]==32'h4f465354;
    wire vector_in=observation_valid && observation_payload[255:224]==32'h4f464c57;
    wire expiring=!expired && deadline_due;
    wire pending=status_dirty || status_in || expiring || ready_d!=stream_ready || vector_dirty || vector_in;
    wire [255:0] selected=status_in ? observation_payload : have_status ? latest :
        {32'h4f465354,8'd1,STAGE_BYTE,8'h21,8'd0,192'd0};
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
    // Two-entry conservative elastic FIFO. Push credit depends only on local
    // registered occupancy; downstream ready never drives a wide write CE.
    reg [1:0] count;reg write_bank;
    reg [7:0] read_bank /* synthesis syn_preserve=1 */;
    reg [255:0] slot0,slot1;
    wire push=count<2 && pending,pop=event_valid && event_ready;
    wire status_pending=status_dirty || status_in || expiring || ready_d!=stream_ready;
    wire [255:0] push_data=status_pending ? masked : vector_in ? observation_payload : vector;
    assign event_valid=count!=0;
    genvar g;
    generate for(g=0;g<8;g=g+1)begin:payload_group
        always @(posedge clk or negedge rst_n)
            if(!rst_n)read_bank[g]<=0;else if(pop)read_bank[g]<=!read_bank[g];
        assign event_payload[g*32+:32]=read_bank[g] ? slot1[g*32+:32] : slot0[g*32+:32];
    end endgenerate
    always @(posedge clk)begin
        if(status_in)latest<=observation_payload;
        if(vector_in)vector<=observation_payload;
        if(push && !write_bank)slot0<=push_data;
        if(push && write_bank)slot1<=push_data;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin count<=0;write_bank<=0;end
        else begin
            case({push,pop})2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:begin end endcase
            if(push)write_bank<=!write_bank;
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            have_status<=0;expired<=1;status_dirty<=1;vector_dirty<=0;ready_d<=0;age<=0;deadline_due<=0;
        end else begin
            ready_d<=stream_ready;
            // Register the comparator one tick early, preserving the deadline
            // while removing a counter-to-256-bit-mask combinational path.
            deadline_due<=age>=LIMIT-2;
            if(!deadline_due)age<=age+1'b1;
            if(expiring)begin expired<=1;status_dirty<=1;end
            if(ready_d!=stream_ready)status_dirty<=1;
            if(status_in)begin have_status<=1;expired<=0;age<=0;deadline_due<=0;status_dirty<=1;end
            if(vector_in)vector_dirty<=1;
            // Keep ready-to-valid logic independent of the wide payload enable.
            if(push)begin
                if(status_pending)begin
                    status_dirty<=0;
                end else if(vector_dirty || vector_in)begin
                    vector_dirty<=0;
                end
            end
        end
    end
endmodule
