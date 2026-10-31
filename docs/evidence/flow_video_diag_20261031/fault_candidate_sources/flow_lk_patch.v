// Sparse inverse-compositional LK patch, following Bouguet section 2.3.
// Architecture reference: rothej/optical-flow-fpga (MIT, see reference/optical_flow).
// Pixels are UNSIGNED; forward flow is right/down, in signed Q7 fine pixels.
// One start -> exactly one done, including texture/boundary/range rejection.
module flow_lk_patch #(parameter WIDTH=160,HEIGHT=90,STEP_LIMIT=512,FLOW_LIMIT=2048,PYRAMID=0)(
    input clk,rst_n,start,coarse,input [15:0] center_x,center_y,
    input signed [15:0] initial_u,initial_v,
    output reg busy,done,valid,output reg signed [15:0] u,v,
    output reg read_enable,read_current,output reg [15:0] read_address,
    input [7:0] read_data,output [39:0] debug_status
);
    localparam IDLE=0,CHECK=1,REQUEST=2,WAIT_RAM=3,LATCH=4,MIX_X=5,MIX_Y=6,
        GRADIENT=7,PRODUCTS=8,ACCUMULATE=9,MATRIX=10,DIFFERENCES=11,
        QUALITY=12,DIV_U=13,WAIT_U=14,DIV_V=15,WAIT_V=16,FINISH=17;
    reg [4:0] state;
    reg signed [16:0] cx,cy,sx,sy,wx,wy;
    reg signed [15:0] seed_u,seed_v;
    reg [6:0] fx,fy;
    reg signed [3:0] dx,dy;
    reg [3:0] sample_index;
    reg [7:0] p0,pl,pr,pt,pb,c00,c10,c01,c11,warped;
    reg [15:0] mix_top,mix_bottom;
    reg [22:0] mix_total;
    reg signed [8:0] gx,gy,gt;
    // Keep these small products in fabric (Gowin SUG550 section 5.6).
    // V1.9.12.04 DSP absorption across their conditional registers loses the
    // subsequent tensor feedback; post-synthesis simulation must cover this.
    reg signed [17:0] xx /* synthesis syn_dspstyle="logic" */;
    reg signed [17:0] yy /* synthesis syn_dspstyle="logic" */;
    reg signed [17:0] xy /* synthesis syn_dspstyle="logic" */;
    reg signed [17:0] xt /* synthesis syn_dspstyle="logic" */;
    reg signed [17:0] yt /* synthesis syn_dspstyle="logic" */;
    reg signed [23:0] axx,ayy,axy,axt,ayt;
    reg signed [47:0] d0,d1,nu0,nu1,nv0,nv1;
    reg signed [48:0] det,nu,nv;
    reg [49:0] trace_square;
    wire [24:0] trace_value={1'b0,axx[23:0]}+{1'b0,ayy[23:0]};
    reg div_start;reg [63:0] dividend,divisor;
    wire div_done,div_busy;wire [7:0] div_count;wire [63:0] quotient;
    reg signed [16:0] du,dv;
    reg range_ok;
    // A patch owns its scale from start through done. Keep this control local
    // to address arithmetic rather than routing the grid's scale across DSPs.
    reg coarse_q;
    reg [4:0] debug_previous_state;
    reg [15:0] debug_wait;
    wire [7:0] debug_state=state;
    // Observation only. No watchdog/recovery is inserted into the solver.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin debug_previous_state<=IDLE;debug_wait<=0;end
        else begin
            debug_previous_state<=state;
            if(state!=debug_previous_state || state==IDLE)debug_wait<=0;
            else if(debug_wait!=16'hffff)debug_wait<=debug_wait+1'b1;
        end
    end
    assign debug_status={debug_state,div_count,
        coarse_q,range_ok,valid,done,busy,div_start,div_done,div_busy,debug_wait};
    always @(posedge clk or negedge rst_n)
        if(!rst_n)coarse_q<=0;else if(start && state==IDLE)coarse_q<=coarse;
    wire [15:0] image_width=PYRAMID && coarse_q ? WIDTH/2 : WIDTH;
    wire [15:0] image_height=PYRAMID && coarse_q ? HEIGHT/2 : HEIGHT;
    wire signed [17:0] result_u=$signed(seed_u)+$signed(du);
    wire signed [17:0] result_v=$signed(seed_v)+$signed(dv);
    wire signed [8:0] diff_x=$signed({1'b0,pr})-$signed({1'b0,pl});
    wire signed [8:0] diff_y=$signed({1'b0,pb})-$signed({1'b0,pt});
    vision_divider #(.WIDTH(64)) divide(.clk(clk),.rst_n(rst_n),.start(div_start),
        .numerator(dividend),.denominator(divisor),.busy(div_busy),.done(div_done),.quotient(quotient),.debug_count(div_count));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;busy<=0;done<=0;valid<=0;u<=0;v<=0;read_enable<=0;read_current<=0;read_address<=0;
            cx<=0;cy<=0;sx<=0;sy<=0;wx<=0;wy<=0;seed_u<=0;seed_v<=0;fx<=0;fy<=0;dx<=0;dy<=0;sample_index<=0;
            p0<=0;pl<=0;pr<=0;pt<=0;pb<=0;c00<=0;c10<=0;c01<=0;c11<=0;warped<=0;
            mix_top<=0;mix_bottom<=0;mix_total<=0;gx<=0;gy<=0;gt<=0;xx<=0;yy<=0;xy<=0;xt<=0;yt<=0;
            axx<=0;ayy<=0;axy<=0;axt<=0;ayt<=0;d0<=0;d1<=0;nu0<=0;nu1<=0;nv0<=0;nv1<=0;
            det<=0;nu<=0;nv<=0;trace_square<=0;div_start<=0;dividend<=0;divisor<=0;du<=0;dv<=0;range_ok<=0;
        end else begin
            done<=0;read_enable<=0;div_start<=0;
            case(state)
                IDLE:if(start)begin
                    busy<=1;valid<=0;u<=0;v<=0;cx<=$signed({1'b0,center_x});cy<=$signed({1'b0,center_y});
                    seed_u<=initial_u;seed_v<=initial_v;fx<=initial_u[6:0];fy<=initial_v[6:0];
                    sx<=$signed({1'b0,center_x})-2;sy<=$signed({1'b0,center_y})-2;
                    wx<=$signed({1'b0,center_x})-2+($signed(initial_u)>>>7);
                    wy<=$signed({1'b0,center_y})-2+($signed(initial_v)>>>7);
                    dx<=-2;dy<=-2;sample_index<=0;axx<=0;ayy<=0;axy<=0;axt<=0;ayt<=0;
                    range_ok<=1;state<=CHECK;
                end
                CHECK:begin
                    // Entire patch plus central derivatives and bilinear neighbours must fit.
                    if(cx<3 || cy<3 || cx>=image_width-3 || cy>=image_height-3 ||
                       wx<0 || wy<0 || wx+5>=image_width || wy+5>=image_height)begin range_ok<=0;state<=FINISH;end
                    else state<=REQUEST;
                end
                REQUEST:begin
                    read_enable<=1;read_current<=sample_index>=5;
                    case(sample_index)
                        0:read_address<=sy*image_width+sx;
                        1:read_address<=sy*image_width+sx-1;
                        2:read_address<=sy*image_width+sx+1;
                        3:read_address<=(sy-1)*image_width+sx;
                        4:read_address<=(sy+1)*image_width+sx;
                        5:read_address<=wy*image_width+wx;
                        6:read_address<=wy*image_width+wx+1;
                        7:read_address<=(wy+1)*image_width+wx;
                        default:read_address<=(wy+1)*image_width+wx+1;
                    endcase
                    state<=WAIT_RAM;
                end
                WAIT_RAM:state<=LATCH;
                LATCH:begin
                    case(sample_index)
                        0:p0<=read_data;1:pl<=read_data;2:pr<=read_data;3:pt<=read_data;4:pb<=read_data;
                        5:c00<=read_data;6:c10<=read_data;7:c01<=read_data;8:c11<=read_data;
                    endcase
                    if(sample_index==8)state<=MIX_X;
                    else begin sample_index<=sample_index+1'b1;state<=REQUEST;end
                end
                MIX_X:begin
                    mix_top<=c00*(16'd128-{9'd0,fx})+c10*{9'd0,fx};
                    mix_bottom<=c01*(16'd128-{9'd0,fx})+c11*{9'd0,fx};state<=MIX_Y;
                end
                MIX_Y:begin mix_total<=mix_top*(23'd128-{16'd0,fy})+mix_bottom*{16'd0,fy};state<=GRADIENT;end
                GRADIENT:begin
                    gx<=diff_x>>>1;gy<=diff_y>>>1;
                    gt<=$signed({1'b0,p0})-$signed({1'b0,mix_total[21:14]});state<=PRODUCTS;
                end
                PRODUCTS:begin xx<=gx*gx;yy<=gy*gy;xy<=gx*gy;xt<=gx*gt;yt<=gy*gt;state<=ACCUMULATE;end
                ACCUMULATE:begin
                    axx<=axx+xx;ayy<=ayy+yy;axy<=axy+xy;axt<=axt+xt;ayt<=ayt+yt;
                    sample_index<=0;
                    if(dx==2 && dy==2)state<=MATRIX;
                    else begin
                        if(dx==2)begin dx<=-2;dy<=dy+1'b1;sx<=cx-2;sy<=sy+1'b1;wx<=cx-2+($signed(seed_u)>>>7);wy<=wy+1'b1;end
                        else begin dx<=dx+1'b1;sx<=sx+1'b1;wx<=wx+1'b1;end
                        state<=REQUEST;
                    end
                end
                MATRIX:begin
                    d0<=axx*ayy;d1<=axy*axy;nu0<=ayy*axt;nu1<=axy*ayt;nv0<=axx*ayt;nv1<=axy*axt;
                    trace_square<=trace_value*trace_value;state<=DIFFERENCES;
                end
                DIFFERENCES:begin
                    det<=$signed({d0[47],d0})-$signed({d1[47],d1});
                    nu<=$signed({nu0[47],nu0})-$signed({nu1[47],nu1});
                    nv<=$signed({nv0[47],nv0})-$signed({nv1[47],nv1});state<=QUALITY;
                end
                QUALITY:begin
                    if(trace_value<512 || det<=4096 || $unsigned(det)<=(trace_square>>6))begin range_ok<=0;state<=FINISH;end
                    else begin divisor<={15'd0,det};state<=DIV_U;end
                end
                DIV_U:begin
                    dividend<=nu<0 ? ((-$signed({{15{nu[48]}},nu}))<<7) : ({15'd0,nu}<<7);
                    div_start<=1;state<=WAIT_U;
                end
                WAIT_U:if(div_done)begin
                    if(quotient>STEP_LIMIT)begin range_ok<=0;state<=FINISH;end
                    else begin du<=nu<0 ? -$signed({1'b0,quotient[15:0]}) : $signed({1'b0,quotient[15:0]});state<=DIV_V;end
                end
                DIV_V:begin
                    dividend<=nv<0 ? ((-$signed({{15{nv[48]}},nv}))<<7) : ({15'd0,nv}<<7);
                    div_start<=1;state<=WAIT_V;
                end
                WAIT_V:if(div_done)begin
                    if(quotient>STEP_LIMIT)range_ok<=0;
                    dv<=nv<0 ? -$signed({1'b0,quotient[15:0]}) : $signed({1'b0,quotient[15:0]});state<=FINISH;
                end
                FINISH:begin
                    done<=1;busy<=0;state<=IDLE;
                    valid<=range_ok && result_u>=-FLOW_LIMIT && result_u<=FLOW_LIMIT && result_v>=-FLOW_LIMIT && result_v<=FLOW_LIMIT;
                    if(range_ok && result_u>=-FLOW_LIMIT && result_u<=FLOW_LIMIT && result_v>=-FLOW_LIMIT && result_v<=FLOW_LIMIT)begin u<=result_u[15:0];v<=result_v[15:0];end
                    else begin u<=0;v<=0;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
