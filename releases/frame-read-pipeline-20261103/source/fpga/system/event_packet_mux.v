// Fair packet arbitration with an immutable output while the network stalls.
// Each producer owns its pending snapshot until valid && ready.
module event_packet_mux(
    input clk,rst_n,a_valid,b_valid,input [255:0] a_payload,b_payload,
    output a_ready,b_ready,input out_ready,
    output reg out_valid,output reg [255:0] out_payload
);
    reg prefer_b;
    wire available=!out_valid || out_ready;
    wire choose_b=b_valid && (!a_valid || prefer_b);
    assign a_ready=available && !choose_b;
    assign b_ready=available && choose_b;
    always @(posedge clk)if(available && (a_valid || b_valid))
        out_payload<=choose_b ? b_payload : a_payload;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin out_valid<=0;prefer_b<=0;end
        else if(available)begin
            out_valid<=a_valid || b_valid;
            if(a_valid || b_valid)begin
                prefer_b<=!choose_b;
            end
        end
    end
endmodule
