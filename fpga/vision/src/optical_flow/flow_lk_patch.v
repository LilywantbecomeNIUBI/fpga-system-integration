// Sparse inverse-compositional LK patch, following Bouguet section 2.3.
// Architecture reference: rothej/optical-flow-fpga (MIT, see reference/optical_flow).
// Pixels are UNSIGNED; forward flow is right/down, in signed Q7 fine pixels.
// One start -> exactly one done, including texture/boundary/range rejection.
module flow_lk_patch #(parameter WIDTH=160,HEIGHT=90,STEP_LIMIT=512,FLOW_LIMIT=2048,PYRAMID=0)(
    input clk,rst_n,start,coarse,input [15:0] center_x,center_y,
    input signed [15:0] initial_u,initial_v,
    output reg busy,done,valid,output reg signed [15:0] u,v,
    output reg read_enable,read_current,output reg [15:0] read_address,
    input [7:0] read_data,output [39:0] debug_status,output reg [11:0] debug_fault
);
    // Explicit Gray codes; preserve the feedback register, audit actual FF count.
    localparam [4:0] IDLE=5'd0,
        CHECK=5'd1,
        REQUEST=5'd3,
        WAIT_RAM=5'd2,
        LATCH=5'd6,
        MIX_X=5'd7,
        MIX_Y=5'd5,
        GRADIENT=5'd4,
        PRODUCTS=5'd12,
        ACCUMULATE=5'd13,
        MATRIX=5'd15,
        DIFFERENCES=5'd14,
        QUALITY=5'd10,
        DIV_U=5'd11,
        WAIT_U=5'd9,
        DIV_V=5'd8,
        WAIT_V=5'd24,
        FINISH=5'd25,
        ADDRESS=5'd27,ISSUE=5'd26,PRODUCT_SUM=5'd30;
    reg [4:0] state /* synthesis syn_preserve=1 syn_maxfan=100000 */;
    reg signed [16:0] address_row_y,address_column_base;
    reg [15:0] address_delta,row_product,column_sum;
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
    reg signed [13:0] xx_low /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] xx_high /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] yy_low /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] yy_high /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] xy_low /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] xy_high /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] xt_low /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] xt_high /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] yt_low /* synthesis syn_dspstyle="logic" */;
    reg signed [13:0] yt_high /* synthesis syn_dspstyle="logic" */;
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
    // Data-control registers are outside the sole feedback state bank.
    // Look ahead from next_state so the registered decode remains aligned,
    // adding no externally observable state-machine cycle.
    reg [4:0] next_state;
    reg [31:0] data_phase /* synthesis syn_preserve=1 */;
    function [31:0] phase_decode;
        input [4:0] code;
        begin
            case(code)
                IDLE:phase_decode=32'd1<<IDLE;
                CHECK:phase_decode=32'd1<<CHECK;
                REQUEST:phase_decode=32'd1<<REQUEST;
                WAIT_RAM:phase_decode=32'd1<<WAIT_RAM;
                LATCH:phase_decode=32'd1<<LATCH;
                MIX_X:phase_decode=32'd1<<MIX_X;
                MIX_Y:phase_decode=32'd1<<MIX_Y;
                GRADIENT:phase_decode=32'd1<<GRADIENT;
                PRODUCTS:phase_decode=32'd1<<PRODUCTS;
                ACCUMULATE:phase_decode=32'd1<<ACCUMULATE;
                MATRIX:phase_decode=32'd1<<MATRIX;
                DIFFERENCES:phase_decode=32'd1<<DIFFERENCES;
                QUALITY:phase_decode=32'd1<<QUALITY;
                DIV_U:phase_decode=32'd1<<DIV_U;
                WAIT_U:phase_decode=32'd1<<WAIT_U;
                DIV_V:phase_decode=32'd1<<DIV_V;
                WAIT_V:phase_decode=32'd1<<WAIT_V;
                FINISH:phase_decode=32'd1<<FINISH;
                ADDRESS:phase_decode=32'd1<<ADDRESS;
                ISSUE:phase_decode=32'd1<<ISSUE;
                PRODUCT_SUM:phase_decode=32'd1<<PRODUCT_SUM;
                default:phase_decode=0;
            endcase
        end
    endfunction
    always @* begin
        next_state=state;
        case(state)
            IDLE:if(busy)next_state=FINISH;else if(start)next_state=CHECK;
            CHECK:if(cx<3 || cy<3 || cx>=image_width-3 || cy>=image_height-3 ||
                wx<0 || wy<0 || wx+5>=image_width || wy+5>=image_height)next_state=FINISH;
                else next_state=REQUEST;
            REQUEST:next_state=ADDRESS;
            ADDRESS:next_state=ISSUE;
            ISSUE:next_state=WAIT_RAM;
            WAIT_RAM:next_state=LATCH;
            LATCH:next_state=sample_index==8 ? MIX_X : REQUEST;
            MIX_X:next_state=MIX_Y;
            MIX_Y:next_state=GRADIENT;
            GRADIENT:next_state=PRODUCTS;
            PRODUCTS:next_state=PRODUCT_SUM;
            PRODUCT_SUM:next_state=ACCUMULATE;
            ACCUMULATE:next_state=dx==2 && dy==2 ? MATRIX : REQUEST;
            MATRIX:next_state=DIFFERENCES;
            DIFFERENCES:next_state=QUALITY;
            QUALITY:next_state=trace_value<512 || det<=4096 || $unsigned(det)<=(trace_square>>6) ? FINISH : DIV_U;
            DIV_U:next_state=WAIT_U;
            WAIT_U:if(div_done)next_state=quotient>STEP_LIMIT ? FINISH : DIV_V;
            DIV_V:next_state=WAIT_V;
            WAIT_V:if(div_done)next_state=FINISH;
            FINISH:next_state=IDLE;
            default:next_state=FINISH;
        endcase
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=IDLE;data_phase<=phase_decode(IDLE);end
        else begin state<=next_state;data_phase<=phase_decode(next_state);end
    end
    reg [4:0] debug_previous_state;
    reg [15:0] debug_wait;
    wire [7:0] debug_state=state;
    // Diagnostic counters observe state; the existing fault exits remain below.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin debug_previous_state<=0;debug_wait<=0;end
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
            busy<=0;done<=0;valid<=0;u<=0;v<=0;read_enable<=0;read_current<=0;read_address<=0;
            cx<=0;cy<=0;sx<=0;sy<=0;wx<=0;wy<=0;seed_u<=0;seed_v<=0;fx<=0;fy<=0;dx<=0;dy<=0;sample_index<=0;
            p0<=0;pl<=0;pr<=0;pt<=0;pb<=0;c00<=0;c10<=0;c01<=0;c11<=0;warped<=0;
            mix_top<=0;mix_bottom<=0;mix_total<=0;gx<=0;gy<=0;gt<=0;xx<=0;yy<=0;xy<=0;xt<=0;yt<=0;
            axx<=0;ayy<=0;axy<=0;axt<=0;ayt<=0;d0<=0;d1<=0;nu0<=0;nu1<=0;nv0<=0;nv1<=0;
            det<=0;nu<=0;nv<=0;trace_square<=0;div_start<=0;dividend<=0;divisor<=0;du<=0;dv<=0;range_ok<=0;
            debug_fault<=0;
        end else begin
            done<=0;read_enable<=0;div_start<=0;
            case(1'b1)
                data_phase[IDLE]:if(busy)begin
                    // Captured on hardware: state=IDLE, busy=1, no done.
                    // Abort this inconsistent in-flight patch through FINISH.
                    if(!debug_fault[11])debug_fault<={1'b1,1'b1,5'd0,debug_previous_state};
                    range_ok<=0;
                end else if(start)begin
                    busy<=1;valid<=0;u<=0;v<=0;cx<=$signed({1'b0,center_x});cy<=$signed({1'b0,center_y});
                    seed_u<=initial_u;seed_v<=initial_v;fx<=initial_u[6:0];fy<=initial_v[6:0];
                    sx<=$signed({1'b0,center_x})-2;sy<=$signed({1'b0,center_y})-2;
                    wx<=$signed({1'b0,center_x})-2+($signed(initial_u)>>>7);
                    wy<=$signed({1'b0,center_y})-2+($signed(initial_v)>>>7);
                    dx<=-2;dy<=-2;sample_index<=0;axx<=0;ayy<=0;axy<=0;axt<=0;ayt<=0;
                    range_ok<=1;
                end
                data_phase[CHECK]:begin
                    // Entire patch plus central derivatives and bilinear neighbours must fit.
                    if(cx<3 || cy<3 || cx>=image_width-3 || cy>=image_height-3 ||
                       wx<0 || wy<0 || wx+5>=image_width || wy+5>=image_height)begin range_ok<=0;end
                    else ;
                end
                data_phase[REQUEST]:begin
                    // Select coordinates and offsets before either arithmetic stage.
                    address_row_y<=sample_index>=5 ? wy : sy;
                    address_column_base<=sample_index>=5 ? wx : sx;
                    case(sample_index)
                        0,5:address_delta<=0;
                        1:address_delta<=16'hffff;
                        2,6:address_delta<=1;
                        3:address_delta<=16'd0-image_width;
                        4,7:address_delta<=image_width;
                        default:address_delta<=image_width+16'd1;
                    endcase
                    
                end
                data_phase[ADDRESS]:begin
                    row_product<=address_row_y*image_width;
                    column_sum<=address_column_base+address_delta;
                    
                end
                data_phase[ISSUE]:begin
                    read_enable<=1;read_current<=sample_index>=5;
                    read_address<=row_product+column_sum;
                    
                end
                data_phase[WAIT_RAM]:;
                data_phase[LATCH]:begin
                    case(sample_index)
                        0:p0<=read_data;1:pl<=read_data;2:pr<=read_data;3:pt<=read_data;4:pb<=read_data;
                        5:c00<=read_data;6:c10<=read_data;7:c01<=read_data;8:c11<=read_data;
                    endcase
                    if(sample_index==8);
                    else begin sample_index<=sample_index+1'b1;end
                end
                data_phase[MIX_X]:begin
                    mix_top<=c00*(16'd128-{9'd0,fx})+c10*{9'd0,fx};
                    mix_bottom<=c01*(16'd128-{9'd0,fx})+c11*{9'd0,fx};
                end
                data_phase[MIX_Y]:begin mix_total<=mix_top*(23'd128-{16'd0,fy})+mix_bottom*{16'd0,fy};end
                data_phase[GRADIENT]:begin
                    gx<=diff_x>>>1;gy<=diff_y>>>1;
                    gt<=$signed({1'b0,p0})-$signed({1'b0,mix_total[21:14]});
                end
                data_phase[PRODUCTS]:begin
                    xx_low<=$signed({1'b0,gx[3:0]})*gx;
                    xx_high<=$signed(gx[8:4])*gx;
                    yy_low<=$signed({1'b0,gy[3:0]})*gy;
                    yy_high<=$signed(gy[8:4])*gy;
                    xy_low<=$signed({1'b0,gx[3:0]})*gy;
                    xy_high<=$signed(gx[8:4])*gy;
                    xt_low<=$signed({1'b0,gx[3:0]})*gt;
                    xt_high<=$signed(gx[8:4])*gt;
                    yt_low<=$signed({1'b0,gy[3:0]})*gt;
                    yt_high<=$signed(gy[8:4])*gt;
                    
                end
                data_phase[PRODUCT_SUM]:begin
                    xx<=($signed({{4{xx_high[13]}},xx_high})<<<4)+$signed({{4{xx_low[13]}},xx_low});
                    yy<=($signed({{4{yy_high[13]}},yy_high})<<<4)+$signed({{4{yy_low[13]}},yy_low});
                    xy<=($signed({{4{xy_high[13]}},xy_high})<<<4)+$signed({{4{xy_low[13]}},xy_low});
                    xt<=($signed({{4{xt_high[13]}},xt_high})<<<4)+$signed({{4{xt_low[13]}},xt_low});
                    yt<=($signed({{4{yt_high[13]}},yt_high})<<<4)+$signed({{4{yt_low[13]}},yt_low});
                    
                end
                data_phase[ACCUMULATE]:begin
                    axx<=axx+xx;ayy<=ayy+yy;axy<=axy+xy;axt<=axt+xt;ayt<=ayt+yt;
                    sample_index<=0;
                    if(dx==2 && dy==2);
                    else begin
                        if(dx==2)begin dx<=-2;dy<=dy+1'b1;sx<=cx-2;sy<=sy+1'b1;wx<=cx-2+($signed(seed_u)>>>7);wy<=wy+1'b1;end
                        else begin dx<=dx+1'b1;sx<=sx+1'b1;wx<=wx+1'b1;end
                        
                    end
                end
                data_phase[MATRIX]:begin
                    d0<=axx*ayy;d1<=axy*axy;nu0<=ayy*axt;nu1<=axy*ayt;nv0<=axx*ayt;nv1<=axy*axt;
                    trace_square<=trace_value*trace_value;
                end
                data_phase[DIFFERENCES]:begin
                    det<=$signed({d0[47],d0})-$signed({d1[47],d1});
                    nu<=$signed({nu0[47],nu0})-$signed({nu1[47],nu1});
                    nv<=$signed({nv0[47],nv0})-$signed({nv1[47],nv1});
                end
                data_phase[QUALITY]:begin
                    if(trace_value<512 || det<=4096 || $unsigned(det)<=(trace_square>>6))begin range_ok<=0;end
                    else begin divisor<={15'd0,det};end
                end
                data_phase[DIV_U]:begin
                    dividend<=nu<0 ? ((-$signed({{15{nu[48]}},nu}))<<7) : ({15'd0,nu}<<7);
                    div_start<=1;
                end
                data_phase[WAIT_U]:if(div_done)begin
                    if(quotient>STEP_LIMIT)begin range_ok<=0;end
                    else begin du<=nu<0 ? -$signed({1'b0,quotient[15:0]}) : $signed({1'b0,quotient[15:0]});end
                end
                data_phase[DIV_V]:begin
                    dividend<=nv<0 ? ((-$signed({{15{nv[48]}},nv}))<<7) : ({15'd0,nv}<<7);
                    div_start<=1;
                end
                data_phase[WAIT_V]:if(div_done)begin
                    if(quotient>STEP_LIMIT)range_ok<=0;
                    dv<=nv<0 ? -$signed({1'b0,quotient[15:0]}) : $signed({1'b0,quotient[15:0]});
                end
                data_phase[FINISH]:begin
                    done<=busy;busy<=0;
                    valid<=range_ok && result_u>=-FLOW_LIMIT && result_u<=FLOW_LIMIT && result_v>=-FLOW_LIMIT && result_v<=FLOW_LIMIT;
                    if(range_ok && result_u>=-FLOW_LIMIT && result_u<=FLOW_LIMIT && result_v>=-FLOW_LIMIT && result_v<=FLOW_LIMIT)begin u<=result_u[15:0];v<=result_v[15:0];end
                    else begin u<=0;v<=0;end
                end
                // Preserve the one-start/one-done contract on an illegal FSM
                // encoding. IDLE alone strands the grid in WAIT_PATCH forever.
                // Retain the first observed code until explicit reset.
                default:begin
                    if(!debug_fault[11])debug_fault<={1'b1,1'b0,state,debug_previous_state};
                    range_ok<=0;
                end
            endcase
            if((state==IDLE && busy) || phase_decode(state)==0)begin
                if(!debug_fault[11])debug_fault<={1'b1,(state==IDLE && busy),state,debug_previous_state};
                range_ok<=0;
            end
        end
    end
endmodule
