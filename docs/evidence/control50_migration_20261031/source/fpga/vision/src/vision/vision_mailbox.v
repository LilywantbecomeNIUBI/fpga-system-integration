// Immutable bundled-data/toggle handshake, adapted from network_runtime_dual.
// One in-flight message and one newest pending message; intermediate updates coalesce.
// Both resets must be asserted together; each domain releases synchronously.
module vision_mailbox #(parameter WIDTH=256)(
    input src_clk,src_rst_n,src_valid,input [WIDTH-1:0] src_data,
    input dst_clk,dst_rst_n,output reg dst_valid,output reg [WIDTH-1:0] dst_data
);
    reg req,ack,as0,as1,qs0,qs1,pending;
    reg [WIDTH-1:0] held,newest;
    always @(posedge src_clk or negedge src_rst_n)begin
        if(!src_rst_n)begin req<=0;as0<=0;as1<=0;pending<=0;held<=0;newest<=0;end
        else begin
            as0<=ack;as1<=as0;
            if(src_valid)begin newest<=src_data;pending<=1;end
            if(req==as1 && (pending || src_valid))begin
                held<=src_valid ? src_data : newest;req<=~req;pending<=0;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n)begin
        if(!dst_rst_n)begin ack<=0;qs0<=0;qs1<=0;dst_valid<=0;dst_data<=0;end
        else begin
            qs0<=req;qs1<=qs0;dst_valid<=0;
            if(qs1!=ack)begin dst_data<=held;dst_valid<=1;ack<=qs1;end
        end
    end
endmodule
