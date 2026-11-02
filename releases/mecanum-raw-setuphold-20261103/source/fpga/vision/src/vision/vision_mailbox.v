// Immutable bundled-data/toggle handshake, adapted from network_runtime_dual.
// One in-flight message and one newest pending message; intermediate updates coalesce.
// Both resets must be asserted together; each domain releases synchronously.
module vision_mailbox #(parameter WIDTH=256, SPLIT_RESET=0)(
    input src_clk,src_rst_n,src_valid,input [WIDTH-1:0] src_data,
    input dst_clk,dst_rst_n,output reg dst_valid,output reg [WIDTH-1:0] dst_data,
    input [3:0] src_reset_bank,input [1:0] dst_reset_bank
);
    reg req,ack,as0,as1,qs0,qs1,pending;
    reg [WIDTH-1:0] held,newest;
    always @(posedge src_clk or negedge src_rst_n)begin
        if(!src_rst_n)begin req<=0;as0<=0;as1<=0;pending<=0;end
        else begin
            as0<=ack;as1<=as0;
            if(src_valid)begin pending<=1;end
            if(req==as1 && (pending || src_valid))begin
                req<=~req;pending<=0;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n)begin
        if(!dst_rst_n)begin ack<=0;qs0<=0;qs1<=0;dst_valid<=0;end
        else begin
            qs0<=req;qs1<=qs0;dst_valid<=0;
            if(qs1!=ack)begin dst_valid<=1;ack<=qs1;end
        end
    end
    // Keep payload reset loads below 200 per release stage. Control and payload
    // banks assert together and release on the same third clock edge.
    genvar g;
    generate for(g=0;g<4;g=g+1)begin:source_bank
        localparam LO=g*((WIDTH+3)/4);
        localparam N=(WIDTH-LO)<((WIDTH+3)/4) ? WIDTH-LO : ((WIDTH+3)/4);
        wire reset_n=SPLIT_RESET ? src_reset_bank[g] : src_rst_n;
        if(N>0)always @(posedge src_clk or negedge reset_n)
            if(!reset_n)begin held[LO+:N]<=0;newest[LO+:N]<=0;end
            else begin
                if(src_valid)newest[LO+:N]<=src_data[LO+:N];
                if(req==as1 && (pending || src_valid))held[LO+:N]<=src_valid ? src_data[LO+:N] : newest[LO+:N];
            end
    end
    for(g=0;g<2;g=g+1)begin:destination_bank
        localparam LO=g*((WIDTH+1)/2);
        localparam N=(WIDTH-LO)<((WIDTH+1)/2) ? WIDTH-LO : ((WIDTH+1)/2);
        wire reset_n=SPLIT_RESET ? dst_reset_bank[g] : dst_rst_n;
        if(N>0)always @(posedge dst_clk or negedge reset_n)
            if(!reset_n)dst_data[LO+:N]<=0;
            else if(qs1!=ack)dst_data[LO+:N]<=held[LO+:N];
    end endgenerate
endmodule
