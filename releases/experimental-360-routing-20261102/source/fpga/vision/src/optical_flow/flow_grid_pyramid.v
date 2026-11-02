// Two scales: 3 coarse LK iterations, x2 initial flow, 2 fine residual iterations.
// One shared solver and random-read port; a point's motion is local to its patch.
module flow_grid_pyramid #(parameter WIDTH=160,HEIGHT=90,STEP=8,MARGIN=8)(
    input clk,rst_n,start,invalidate,input [15:0] frame_id,
    output reg busy,frame_done,sample_valid,sample_good,
    output reg [15:0] result_id,x,y,output reg signed [15:0] u,v,
    output reg [31:0] cycles,
    output read_enable,read_current,read_coarse,output [15:0] read_address,input [7:0] read_data,
    output [143:0] debug_status
);
    localparam IDLE=0,LAUNCH=1,WAIT_PATCH=2,PUBLISH=3,NEXT=4;
    reg [2:0] state;reg [15:0] px,py;reg discarded,patch_start,coarse,good;
    reg [1:0] iteration;reg signed [15:0] seed_u,seed_v;
    wire pd,pg;wire signed [15:0] pu,pv;
    wire [39:0] patch_debug;
    wire [11:0] patch_fault;
    reg [2:0] debug_previous_state;reg [15:0] debug_wait;
    reg [7:0] debug_starts,debug_completions;
    wire [7:0] debug_state=state;
    wire [7:0] debug_iteration={patch_fault[11],patch_fault[9:5],iteration};
    assign debug_status={patch_fault[11:10],patch_fault[4:0],busy,debug_state,result_id,px,py,
        patch_debug[39:16],debug_iteration,patch_debug[15:0],debug_wait,debug_starts,debug_completions};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin debug_previous_state<=IDLE;debug_wait<=0;debug_starts<=0;debug_completions<=0;end
        else begin
            debug_previous_state<=state;
            if(state!=debug_previous_state || state==IDLE)debug_wait<=0;
            else if(debug_wait!=16'hffff)debug_wait<=debug_wait+1'b1;
            if(patch_start)debug_starts<=debug_starts+1'b1;
            if(pd)debug_completions<=debug_completions+1'b1;
        end
    end
    wire [15:0] patch_x=coarse ? px>>1 : px,patch_y=coarse ? py>>1 : py;
    assign read_coarse=coarse;
    flow_lk_patch #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.PYRAMID(1)) patch(.clk(clk),.rst_n(rst_n),.start(patch_start),.coarse(coarse),
        .center_x(patch_x),.center_y(patch_y),.initial_u(seed_u),.initial_v(seed_v),
        .busy(),.done(pd),.valid(pg),.u(pu),.v(pv),.read_enable(read_enable),.read_current(read_current),
        .read_address(read_address),.read_data(read_data),.debug_status(patch_debug),.debug_fault(patch_fault));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;px<=0;py<=0;discarded<=0;patch_start<=0;coarse<=0;good<=0;iteration<=0;seed_u<=0;seed_v<=0;
            busy<=0;frame_done<=0;sample_valid<=0;sample_good<=0;result_id<=0;x<=0;y<=0;u<=0;v<=0;cycles<=0;
        end else begin
            frame_done<=0;sample_valid<=0;patch_start<=0;
            if(busy)cycles<=cycles+1'b1;
            if(invalidate && busy)discarded<=1;
            case(state)
                IDLE:if(start)begin
                    busy<=1;discarded<=0;result_id<=frame_id;px<=MARGIN;py<=MARGIN;cycles<=0;
                    coarse<=1;iteration<=0;seed_u<=0;seed_v<=0;state<=LAUNCH;
                end
                LAUNCH:begin patch_start<=1;state<=WAIT_PATCH;end
                WAIT_PATCH:if(pd)begin
                    if(!pg)begin good<=0;seed_u<=0;seed_v<=0;state<=PUBLISH;end
                    else if(coarse && iteration==2)begin
                        if(pu>1024 || pu< -1024 || pv>1024 || pv< -1024)begin good<=0;seed_u<=0;seed_v<=0;state<=PUBLISH;end
                        else begin seed_u<=pu<<<1;seed_v<=pv<<<1;coarse<=0;iteration<=0;state<=LAUNCH;end
                    end else if(!coarse && iteration==1)begin good<=1;seed_u<=pu;seed_v<=pv;state<=PUBLISH;end
                    else begin seed_u<=pu;seed_v<=pv;iteration<=iteration+1'b1;state<=LAUNCH;end
                end
                PUBLISH:begin
                    x<=px;y<=py;u<=seed_u;v<=seed_v;sample_valid<=!discarded && !invalidate;sample_good<=good;state<=NEXT;
                end
                NEXT:begin
                    coarse<=1;iteration<=0;seed_u<=0;seed_v<=0;
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
