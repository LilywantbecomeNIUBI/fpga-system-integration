// Single producer/consumer, synchronous dual-port RAM, Gray pointer CDC.
// Both resets derive from ONE system reset, with local synchronous release.
// Camera/config resets must never reset either pointer independently.
module vision_async_fifo #(parameter WIDTH=46,AB=8)(
    input wr_clk,wr_rst_n,wr_en,input [WIDTH-1:0] wr_data,output full,
    input rd_clk,rd_rst_n,rd_en,output reg [WIDTH-1:0] rd_data,
    output reg rd_valid,output empty
);
    reg [WIDTH-1:0] memory[0:(1<<AB)-1];
    reg [AB:0] wbin,wgray,rbin,rgray;
    reg [AB:0] rs0,rs1,ws0,ws1 /* synthesis syn_preserve=1 */;
    assign full=wgray=={~rs1[AB:AB-1],rs1[AB-2:0]};
    assign empty=rgray==ws1;
    wire [AB:0] wn=wbin+1'b1,rn=rbin+1'b1;
    always @(posedge wr_clk)if(wr_en && !full)memory[wbin[AB-1:0]]<=wr_data;
    always @(posedge rd_clk)if(rd_en && !empty)rd_data<=memory[rbin[AB-1:0]];
    always @(posedge wr_clk or negedge wr_rst_n)begin
        if(!wr_rst_n)begin wbin<=0;wgray<=0;rs0<=0;rs1<=0;end
        else begin
            rs0<=rgray;rs1<=rs0;
            if(wr_en && !full)begin wbin<=wn;wgray<=wn^(wn>>1);end
        end
    end
    always @(posedge rd_clk or negedge rd_rst_n)begin
        if(!rd_rst_n)begin rbin<=0;rgray<=0;ws0<=0;ws1<=0;rd_valid<=0;end
        else begin
            ws0<=wgray;ws1<=ws0;rd_valid<=rd_en && !empty;
            if(rd_en && !empty)begin rbin<=rn;rgray<=rn^(rn>>1);end
        end
    end
endmodule
