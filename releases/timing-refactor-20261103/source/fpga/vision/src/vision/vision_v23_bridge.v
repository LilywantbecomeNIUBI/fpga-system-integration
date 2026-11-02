// Network-clock freshness gate; fast LED validity retains exact deadlines.
module vision_v23_bridge #(parameter CLK_HZ=125000000,TIMEOUT_MS=500)(
    input clk,rst_n,stream_ready,semantic_update,semantic_valid,
    input [255:0] semantic_payload,input observation_valid,input [255:0] observation_payload,
    input event_ready,output event_valid,output [255:0] event_payload,
    output reg [3:0] led
);
    localparam LIMIT=(CLK_HZ/1000)*TIMEOUT_MS;
    localparam AGE_BITS=LIMIT<2 ? 1 : $clog2(LIMIT+1);
    reg [AGE_BITS-1:0] age;reg deadline_due;
    reg have_observation,expired,ready_d,semantic_d,dirty;
    reg [15:0] semantic_fid;
    reg [255:0] fast_latest,latest,observation_sample;
    reg fast_epoch,sample_epoch,latest_epoch,have_latest;
    (* syn_preserve=1, syn_keep=1 *) reg [7:0] observation_sample_valid,ready_sample;
    wire expiring=!expired && deadline_due;
    wire [255:0] selected=observation_valid ? observation_payload : have_observation ? fast_latest : 256'd0;
    wire [15:0] expected=semantic_update ? semantic_payload[207:192] : semantic_fid;
    wire usable=stream_ready && semantic_valid && (!(expired || expiring) || observation_valid) &&
        (have_observation || observation_valid) && selected[175:160]==expected;
    wire pending=dirty || observation_sample_valid[0] || expiring ||
        ready_d!=stream_ready || semantic_d!=semantic_valid || semantic_update;
    always @(posedge clk)begin
        observation_sample<=observation_payload;sample_epoch<=!fast_epoch;
        if(observation_valid)begin
            fast_latest<=0;fast_latest[215:192]<=observation_payload[215:192];
            fast_latest[175:160]<=observation_payload[175:160];
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            have_observation<=0;expired<=0;ready_d<=0;semantic_d<=0;semantic_fid<=0;
            age<=0;deadline_due<=0;led<=0;fast_epoch<=0;have_latest<=0;latest_epoch<=0;dirty<=1;
        end else begin
            ready_d<=stream_ready;semantic_d<=semantic_valid;
            deadline_due<=age>=LIMIT-2;
            led<=usable ? {selected[208] && selected[199:192]==3,
                selected[208] && selected[199:192]==2,selected[208] && selected[199:192]==1,selected[208]} : 4'd0;
            if(!deadline_due)age<=age_incremented;
            if(expiring)expired<=1;
            if(observation_valid)begin have_observation<=1;age<=0;expired<=0;deadline_due<=0;fast_epoch<=!fast_epoch;end
            if(semantic_update)semantic_fid<=semantic_payload[207:192];
            if(observation_sample_valid[0])begin have_latest<=1;latest_epoch<=sample_epoch;end
            if(issue)dirty<=0;
            // These changes are newer than the registered event snapshot.
            if(semantic_update || ready_d!=stream_ready || semantic_d!=semantic_valid)dirty<=1;
            if((observation_sample_valid[0] || expiring) && !issue)dirty<=1;
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
                else if(observation_valid)carry<=0;
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

    reg [255:0] stage_payload;
    (* syn_preserve=1, syn_keep=1 *) reg [7:0] stage_usable,stage_expired,stage_mismatch;
    genvar g;
    generate for(g=0;g<8;g=g+1)begin:payload_group
        wire [255:0] data_in=observation_sample_valid[g] ? observation_sample : have_latest ? latest : 256'd0;
        wire epoch_in=observation_sample_valid[g] ? sample_epoch : latest_epoch;
        wire valid_in=ready_sample[g] && semantic_d && have_observation &&
            !(expired || expiring) && epoch_in==fast_epoch && data_in[175:160]==semantic_fid;
        reg [255:0] masked;
        always @*begin
            masked=stage_payload;masked[255:216]={32'h46535441,8'd2};
            if(!stage_usable[g])begin
                masked[215:208]=(stage_payload[215:208]&8'hf8)|8'h10;masked[199:192]=0;
                if(stage_expired[g])masked[207:200]=9;
                else if(stage_mismatch[g])masked[207:200]=10;
            end
        end
        always @(posedge clk or negedge rst_n)
            if(!rst_n)begin observation_sample_valid[g]<=0;ready_sample[g]<=0;end
            else begin observation_sample_valid[g]<=observation_valid;ready_sample[g]<=stream_ready;end
        always @(posedge clk)begin
            if(observation_sample_valid[g])latest[g*32+:32]<=observation_sample[g*32+:32];
            stage_payload[g*32+:32]<=data_in[g*32+:32];
            stage_usable[g]<=valid_in;stage_expired[g]<=expired || expiring;
            stage_mismatch[g]<=data_in[175:160]!=semantic_fid;
            if(stage_write[g] && !stage_bank[g])slot0[g*32+:32]<=masked[g*32+:32];
            if(stage_write[g] && stage_bank[g])slot1[g*32+:32]<=masked[g*32+:32];
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
