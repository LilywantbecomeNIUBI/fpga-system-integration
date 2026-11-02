// V0: one 5x5 LK solve at each fixed grid point. Invalid samples are explicit.
module flow_grid #(parameter WIDTH=160,HEIGHT=90,STEP=8,MARGIN=8)(
    input clk,rst_n,start,invalidate,input [15:0] frame_id,
    output reg busy,frame_done,sample_valid,sample_good,
    output reg [15:0] result_id,x,y,output reg signed [15:0] u,v,
    output reg [31:0] cycles,
    output read_enable,read_current,output [15:0] read_address,input [7:0] read_data
);
    localparam IDLE=0,LAUNCH=1,WAIT_PATCH=2,NEXT=3;
    reg [1:0] state;reg [15:0] px,py;reg discarded,patch_start;
    wire pd,pg;wire signed [15:0] pu,pv;
    flow_lk_patch #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) patch(.clk(clk),.rst_n(rst_n),.start(patch_start),
        .coarse(1'b0),
        .center_x(px),.center_y(py),.initial_u(16'sd0),.initial_v(16'sd0),
        .busy(),.done(pd),.valid(pg),.u(pu),.v(pv),.read_enable(read_enable),.read_current(read_current),
        .read_address(read_address),.read_data(read_data));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;px<=0;py<=0;discarded<=0;patch_start<=0;busy<=0;frame_done<=0;sample_valid<=0;sample_good<=0;
            result_id<=0;x<=0;y<=0;u<=0;v<=0;cycles<=0;
        end else begin
            frame_done<=0;sample_valid<=0;patch_start<=0;
            if(busy)cycles<=cycles+1'b1;
            if(invalidate && busy)discarded<=1;
            case(state)
                IDLE:if(start)begin busy<=1;discarded<=0;result_id<=frame_id;px<=MARGIN;py<=MARGIN;cycles<=0;state<=LAUNCH;end
                LAUNCH:begin patch_start<=1;state<=WAIT_PATCH;end
                WAIT_PATCH:if(pd)begin
                    x<=px;y<=py;u<=pu;v<=pv;sample_valid<=!discarded && !invalidate;sample_good<=pg;state<=NEXT;
                end
                NEXT:begin
                    if(px+STEP>WIDTH-MARGIN)begin
                        px<=MARGIN;
                        if(py+STEP>HEIGHT-MARGIN)begin busy<=0;frame_done<=1;state<=IDLE;end
                        else begin py<=py+STEP;state<=LAUNCH;end
                    end else begin px<=px+STEP;state<=LAUNCH;end
                end
            endcase
        end
    end
endmodule
