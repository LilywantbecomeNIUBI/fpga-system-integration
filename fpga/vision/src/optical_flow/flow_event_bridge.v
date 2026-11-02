// Independent fast watchdog plus a registered, immutable event queue.
module flow_event_bridge #(parameter CLK_HZ=125000000,TIMEOUT_MS=100,STAGE=2)(
    input clk,rst_n,stream_ready,observation_valid,input [255:0] observation_payload,
    input event_ready,output event_valid,output [255:0] event_payload,
    output stop_request,output signed [7:0] steer,output [7:0] speed_limit,output [2:0] control_state
);
    localparam LIMIT=(CLK_HZ/1000)*TIMEOUT_MS;
    localparam AGE_BITS=LIMIT<2 ? 1 : $clog2(LIMIT+1);
    localparam [7:0] STAGE_BYTE=STAGE;
    reg [AGE_BITS-1:0] age;reg deadline_due;
    reg [255:0] control_latest,latest,vector,observation_sample;
    reg have_status,have_latest,expired,status_dirty,vector_dirty,ready_d;
    reg fast_epoch,sample_epoch,latest_epoch;
    (* syn_preserve=1, syn_keep=1 *) reg [7:0] status_sample,vector_sample,ready_sample;
    wire status_in=observation_valid && observation_payload[255:224]==32'h4f465354;
    wire vector_in=observation_valid && (observation_payload[255:224]==32'h4f464c57 || observation_payload[255:224]==32'h4f464447);
    wire expiring=!expired && deadline_due;
    wire [255:0] selected=status_in ? observation_payload : control_latest;
    wire usable=stream_ready && (have_status || status_in) && (!(expired || expiring) || status_in) && selected[201];
    assign stop_request=!usable || selected[200];
    assign steer=usable ? selected[71:64] : 8'd0;
    assign speed_limit=usable ? selected[79:72] : 8'd0;
    assign control_state=usable ? selected[194:192] : 3'd0;
    wire status_pending=status_dirty || status_sample[0] || expiring || ready_d!=stream_ready;
    wire pending=status_pending || vector_dirty || vector_sample[0];
    always @(posedge clk)begin
        observation_sample<=observation_payload;sample_epoch<=!fast_epoch;
        if(status_in)begin
            control_latest<=0;control_latest[207:192]<=observation_payload[207:192];
            control_latest[79:64]<=observation_payload[79:64];
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            have_status<=0;have_latest<=0;expired<=1;status_dirty<=1;vector_dirty<=0;ready_d<=0;
            age<=0;deadline_due<=0;fast_epoch<=0;latest_epoch<=0;
        end else begin
            ready_d<=stream_ready;
            deadline_due<=age>=LIMIT-2;
            if(!deadline_due)age<=age_incremented;
            if(expiring)expired<=1;
            if(status_in)begin have_status<=1;expired<=0;age<=0;deadline_due<=0;fast_epoch<=!fast_epoch;end
            if(status_sample[0])begin have_latest<=1;latest_epoch<=sample_epoch;end
            if(issue)begin
                if(status_pending)status_dirty<=0;else vector_dirty<=0;
            end
            if((status_sample[0] || expiring) && !(issue && status_pending))status_dirty<=1;
            if(ready_d!=stream_ready)status_dirty<=1;
            if(vector_sample[0] && !(issue && !status_pending))vector_dirty<=1;
        end
    end

    // Byte adders use registered look-ahead carries. Carry state advances only
    // with this counter; synchronous clear wins, preserving every deadline.
    wire [AGE_BITS-1:0] age_incremented;
    genvar age_lane;
    generate for(age_lane=0;age_lane<(AGE_BITS+7)/8;age_lane=age_lane+1)begin:age_segments
        localparam PART=(AGE_BITS-age_lane*8)<8 ? AGE_BITS-age_lane*8 : 8;
        if(age_lane==0)begin
            assign age_incremented[age_lane*8+:PART]=age[age_lane*8+:PART]+1'b1;
        end else begin
            (* syn_preserve=1 *) reg carry;
            always @(posedge clk or negedge rst_n)
                if(!rst_n)carry<=0;
                else if(status_in)carry<=0;
                else if(!deadline_due)carry<=age[age_lane*8-1:0]=={{(age_lane*8-1){1'b1}},1'b0};
            assign age_incremented[age_lane*8+:PART]=age[age_lane*8+:PART]+carry;
        end
    end endgenerate

    // Reserve at issue, publish only after the registered payload is written.
    // Downstream ready affects registered occupancy, never a wide write CE.
    reg [1:0] count;reg write_bank;
    (* syn_preserve=1, syn_keep=1 *) reg [7:0] read_bank,stage_write,stage_bank;
    reg [255:0] slot0,slot1;
    wire pop=event_valid && event_ready;
    wire issue=({1'b0,count}+stage_write[0]<3'd2) && pending;
    assign event_valid=count!=0;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin count<=0;write_bank<=0;end
        else begin
            case({stage_write[0],pop})
                2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:begin end
            endcase
            if(issue)write_bank<=!write_bank;
        end
    end

    reg [255:0] status_stage,vector_stage;
    (* syn_preserve=1, syn_keep=1 *) reg [7:0] stage_kind;
    genvar g;
    generate for(g=0;g<8;g=g+1)begin:payload_group
        wire [255:0] data_in=status_sample[g] ? observation_sample : have_latest ? latest :
            {32'h4f465354,8'd1,STAGE_BYTE,8'h21,8'd0,192'd0};
        wire epoch_in=status_sample[g] ? sample_epoch : latest_epoch;
        wire valid_in=ready_sample[g] && have_status && !(expired || expiring) &&
            epoch_in==fast_epoch && data_in[201];
        reg [255:0] masked;
        always @*begin
            masked=data_in;
            if(!valid_in)begin masked[207:200]=8'h21;masked[199:192]=0;masked[79:64]=0;end
        end
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin status_sample[g]<=0;vector_sample[g]<=0;ready_sample[g]<=0;stage_kind[g]<=0;end
            else begin
                status_sample[g]<=status_in;vector_sample[g]<=vector_in;
                ready_sample[g]<=stream_ready;stage_kind[g]<=status_pending;
            end
        end
        wire [31:0] write_data=stage_kind[g] ? status_stage[g*32+:32] : vector_stage[g*32+:32];
        always @(posedge clk)begin
            if(status_sample[g])latest[g*32+:32]<=observation_sample[g*32+:32];
            if(vector_sample[g])vector[g*32+:32]<=observation_sample[g*32+:32];
            status_stage[g*32+:32]<=masked[g*32+:32];
            vector_stage[g*32+:32]<=vector_sample[g] ? observation_sample[g*32+:32] : vector[g*32+:32];
            if(stage_write[g] && !stage_bank[g])slot0[g*32+:32]<=write_data;
            if(stage_write[g] && stage_bank[g])slot1[g*32+:32]<=write_data;
        end

        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin read_bank[g]<=0;stage_write[g]<=0;stage_bank[g]<=0;end
            else begin
                if(pop)read_bank[g]<=!read_bank[g];
                stage_write[g]<=issue;stage_bank[g]<=write_bank;
            end
        end
        assign event_payload[g*32+:32]=read_bank[g] ? slot1[g*32+:32] : slot0[g*32+:32];
    end endgenerate
endmodule
