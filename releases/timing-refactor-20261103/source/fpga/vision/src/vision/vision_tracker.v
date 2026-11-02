// PCLK-domain multi-rate perception. Sixteen versioned feature frames retain
// delayed semantic anchors. Fixed-template SAD search replays each intermediate
// frame; ambiguous/low-texture/expired/overwritten observations fail closed.
module vision_sad_tracker #(
    parameter GRID_W=80,GRID_H=45,PATCH=8,SEARCH=4,SCALE=16,
    parameter HISTORY=16,MAX_SEMANTIC_FRAMES=14,RED_PERMILLE=250,
    parameter MAX_SAD=1024,MIN_MARGIN=32,MIN_TEXTURE=16
)(
    input clk,rst_n,frame_start,frame_end,frame_good,input [15:0] frame_id,
    input feature_valid,input [15:0] feature_address,feature,
    input seed_valid,input [256:0] seed_data,
    output reg status_valid,output reg [255:0] status_payload
);
    localparam CELLS=GRID_W*GRID_H,AB=$clog2(CELLS),HB=$clog2(HISTORY),PIXELS=PATCH*PATCH;
    localparam IMAGE_W=GRID_W*SCALE,IMAGE_H=GRID_H*SCALE;
    reg [15:0] memory[0:(1<<(AB+HB))-1];
    reg [15:0] tags[0:HISTORY-1];reg [HISTORY-1:0] sealed;
    reg [15:0] write_fid,feature_count,latest_fid;reg have_frame,latest_good;
    reg [AB+HB-1:0] read_address;reg [15:0] read_word,read_cached;
    always @(posedge clk)begin
        if(feature_valid && feature_address<CELLS)memory[{write_fid[HB-1:0],feature_address[AB-1:0]}]<=feature;
        read_word<=memory[read_address];read_cached<=read_word;
    end
    integer k;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin sealed<=0;write_fid<=0;feature_count<=0;latest_fid<=0;have_frame<=0;latest_good<=0;for(k=0;k<HISTORY;k=k+1)tags[k]<=0;end
        else begin
            if(frame_start)begin write_fid<=frame_id;feature_count<=0;sealed[frame_id[HB-1:0]]<=0;end
            if(feature_valid && feature_address<CELLS)feature_count<=feature_count+1'b1;
            if(frame_end)begin
                latest_fid<=write_fid;have_frame<=1;latest_good<=frame_good && feature_count==CELLS;
                if(frame_good && feature_count==CELLS)begin tags[write_fid[HB-1:0]]<=write_fid;sealed[write_fid[HB-1:0]]<=1;end
            end
        end
    end
    function present;
        input [15:0] f;
        begin present=sealed[f[HB-1:0]] && tags[f[HB-1:0]]==f;end
    endfunction
    function [15:0] patch_origin;
        input [15:0] a,b;input integer limit;
        integer center;
        begin center=({1'b0,a}+{1'b0,b})/(2*SCALE)-PATCH/2;
            patch_origin=center<0 ? 0 : center>limit-PATCH ? limit-PATCH : center;end
    endfunction
    localparam IDLE=0,SEED_INIT=1,LOAD_REQ=2,LOAD_WAIT=3,LOAD_TAKE=4,
        SEARCH_INIT=5,CANDIDATE=6,SAD_REQ=7,SAD_WAIT=8,SAD_TAKE=9,ADVANCE=10,
        DECIDE=11,ROI_INIT=12,ROI_REQ=13,ROI_WAIT=14,ROI_TAKE=15,
        DIV_START=16,DIV_WAIT=17,PUBLISH=18,INVALID=19,SAD_ADD=20,SAD_COMPARE=21,MEM_WAIT2=22,
        ADDR_ROW=23,ADDR_SEND=24,MEM_WAIT1=25;
    reg [4:0] state;
    reg [4:0] after_wait;
    reg [15:0] request_row,request_col,row_offset;reg [HB-1:0] request_bank;
    wire [AB-1:0] cell_address=row_offset[AB-1:0]+request_col[AB-1:0];
    reg pending_seed;reg [256:0] held_seed;
    reg active,tracking_allowed,track_ok,observation_ok,force_yolo,history_miss,scene_change;
    reg have_report;reg [15:0] reported_fid,semantic_fid,job_fid,tracked_fid;
    reg [15:0] class_id,confidence,x1,y1,x2,y2,patch_x,patch_y,track_confidence,red_ratio;
    reg [7:0] reason;
    reg [7:0] template[0:PIXELS-1];reg [7:0] min_gray,max_gray;
    reg [7:0] pi;reg [15:0] sad,best_sad,second_sad;
    reg signed [7:0] sx,sy,best_dx,best_dy;
    reg [15:0] candidate_x,candidate_y;
    wire signed [17:0] cx=$signed({2'd0,patch_x})+sx,cy=$signed({2'd0,patch_y})+sy;
    wire signed [17:0] nx1=$signed({2'd0,x1})+$signed(best_dx)*SCALE;
    wire signed [17:0] nx2=$signed({2'd0,x2})+$signed(best_dx)*SCALE;
    wire signed [17:0] ny1=$signed({2'd0,y1})+$signed(best_dy)*SCALE;
    wire signed [17:0] ny2=$signed({2'd0,y2})+$signed(best_dy)*SCALE;
    reg [7:0] template_word,sample_difference;
    wire [7:0] difference=read_cached[7:0]>template_word ? read_cached[7:0]-template_word : template_word-read_cached[7:0];
    wire [15:0] added_sad=sad+{8'd0,sample_difference};
    reg [15:0] rx0,ry0,rxmax,rymax,rx,ry,red_sum,roi_cells;
    reg divide_start;reg [31:0] numerator,denominator;wire divide_done,divide_busy;wire [31:0] quotient;
    vision_divider divide(.clk(clk),.rst_n(rst_n),.start(divide_start),.numerator(numerator),.denominator(denominator),
        .busy(divide_busy),.done(divide_done),.quotient(quotient));
    wire [15:0] semantic_age=job_fid-semantic_fid,latest_age=latest_fid-semantic_fid;
    wire semantic_live=active && !semantic_age[15] && semantic_age<=MAX_SEMANTIC_FRAMES;
    wire color_red=red_ratio>=RED_PERMILLE;
    wire fused=semantic_live && observation_ok && color_red;
    wire [7:0] flags={~latest_good,scene_change,history_miss,force_yolo,color_red,semantic_live,track_ok,fused};
    wire [16:0] xsum={1'b0,x1}+{1'b0,x2};wire [18:0] xtriple={2'd0,xsum}+{1'd0,xsum,1'b0};
    wire [7:0] direction=!fused ? 0 : xtriple<IMAGE_W*2 ? 1 : xtriple>=IMAGE_W*4 ? 3 : 2;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;after_wait<=IDLE;request_row<=0;request_col<=0;row_offset<=0;request_bank<=0;
            template_word<=0;sample_difference<=0;read_address<=0;pending_seed<=0;held_seed<=0;status_valid<=0;status_payload<=0;
            active<=0;tracking_allowed<=0;track_ok<=0;observation_ok<=0;force_yolo<=1;history_miss<=0;scene_change<=0;
            have_report<=0;reported_fid<=0;semantic_fid<=0;job_fid<=0;tracked_fid<=0;class_id<=0;confidence<=0;
            x1<=0;y1<=0;x2<=0;y2<=0;patch_x<=0;patch_y<=0;track_confidence<=0;red_ratio<=0;reason<=1;
            min_gray<=255;max_gray<=0;pi<=0;sad<=0;best_sad<=65535;second_sad<=65535;
            sx<=0;sy<=0;best_dx<=0;best_dy<=0;candidate_x<=0;candidate_y<=0;
            rx0<=0;ry0<=0;rxmax<=0;rymax<=0;rx<=0;ry<=0;red_sum<=0;roi_cells<=0;
            divide_start<=0;numerator<=0;denominator<=0;
        end else begin
            status_valid<=0;divide_start<=0;
            if(seed_valid)begin held_seed<=seed_data;pending_seed<=1;end
            case(state)
                IDLE:if(pending_seed)begin
                    pending_seed<=0;semantic_fid<=held_seed[207:192];class_id<=held_seed[159:144];confidence<=held_seed[143:128];
                    x1<=held_seed[127:112];y1<=held_seed[111:96];x2<=held_seed[95:80];y2<=held_seed[79:64];
                    active<=held_seed[256];tracking_allowed<=0;track_ok<=0;observation_ok<=0;
                    force_yolo<=0;history_miss<=0;scene_change<=0;red_ratio<=0;track_confidence<=0;
                    job_fid<=held_seed[207:192];state<=SEED_INIT;
                end else if(have_frame && (!have_report || latest_fid!=reported_fid))begin
                    if(active && tracking_allowed && !latest_age[15] && latest_age<=MAX_SEMANTIC_FRAMES)begin
                        job_fid<=tracked_fid+1'b1;state<=SEARCH_INIT;
                    end else begin
                        job_fid<=latest_fid;observation_ok<=0;track_ok<=0;force_yolo<=1;
                        if(active && latest_age>MAX_SEMANTIC_FRAMES)begin active<=0;reason<=6;end
                        state<=PUBLISH;
                    end
                end
                SEED_INIT:begin
                    if(!active)begin reason<=2;job_fid<=latest_fid;force_yolo<=1;state<=PUBLISH;end
                    else if(!present(semantic_fid) || !have_frame || latest_age[15] || latest_age>=HISTORY-1 ||
                        x1>=x2 || y1>=y2 || x2>IMAGE_W || y2>IMAGE_H)begin
                        active<=0;history_miss<=1;force_yolo<=1;reason<=5;state<=PUBLISH;
                    end else begin
                        patch_x<=patch_origin(x1,x2,GRID_W);patch_y<=patch_origin(y1,y2,GRID_H);
                        pi<=0;min_gray<=255;max_gray<=0;state<=LOAD_REQ;
                    end
                end
                LOAD_REQ:begin request_bank<=job_fid[HB-1:0];request_row<=patch_y+pi/PATCH;request_col<=patch_x+pi%PATCH;after_wait<=LOAD_TAKE;state<=ADDR_ROW;end
                ADDR_ROW:begin row_offset<=request_row*GRID_W;state<=ADDR_SEND;end
                ADDR_SEND:begin read_address<={request_bank,cell_address};state<=MEM_WAIT1;end
                MEM_WAIT1:state<=MEM_WAIT2;
                LOAD_WAIT:begin state<=MEM_WAIT2;after_wait<=LOAD_TAKE;end
                MEM_WAIT2:state<=after_wait;
                LOAD_TAKE:begin
                    template[pi]<=read_cached[7:0];if(read_cached[7:0]<min_gray)min_gray<=read_cached[7:0];if(read_cached[7:0]>max_gray)max_gray<=read_cached[7:0];
                    if(pi==PIXELS-1)begin tracked_fid<=job_fid;observation_ok<=1;track_confidence<=1000;reason<=0;state<=ROI_INIT;end
                    else begin pi<=pi+1'b1;state<=LOAD_REQ;end
                end
                SEARCH_INIT:begin
                    if(!present(job_fid))begin history_miss<=1;reason<=5;state<=INVALID;end
                    else begin sx<=-SEARCH;sy<=-SEARCH;best_sad<=65535;second_sad<=65535;state<=CANDIDATE;end
                end
                CANDIDATE:begin
                    if(cx<0 || cy<0 || cx>GRID_W-PATCH || cy>GRID_H-PATCH)state<=ADVANCE;
                    else begin candidate_x<=cx[15:0];candidate_y<=cy[15:0];sad<=0;pi<=0;state<=SAD_REQ;end
                end
                SAD_REQ:begin
                    request_bank<=job_fid[HB-1:0];request_row<=candidate_y+pi/PATCH;request_col<=candidate_x+pi%PATCH;
                    template_word<=template[pi];after_wait<=SAD_TAKE;state<=ADDR_ROW;
                end
                SAD_WAIT:begin state<=MEM_WAIT2;after_wait<=SAD_TAKE;end
                SAD_TAKE:begin sample_difference<=difference;state<=SAD_ADD;end
                SAD_ADD:begin
                    sad<=added_sad;
                    if(pi==PIXELS-1)state<=SAD_COMPARE;else begin pi<=pi+1'b1;state<=SAD_REQ;end
                end
                SAD_COMPARE:begin
                    if(sad<best_sad)begin second_sad<=best_sad;best_sad<=sad;best_dx<=sx;best_dy<=sy;end
                    else if(sad<second_sad)second_sad<=sad;
                    state<=ADVANCE;
                end
                ADVANCE:if(sx==SEARCH)begin
                    sx<=-SEARCH;if(sy==SEARCH)state<=DECIDE;else begin sy<=sy+1'b1;state<=CANDIDATE;end
                end else begin sx<=sx+1'b1;state<=CANDIDATE;end
                DECIDE:begin
                    if(best_sad>MAX_SAD || second_sad-best_sad<MIN_MARGIN || nx1<0 || ny1<0 || nx2>IMAGE_W || ny2>IMAGE_H)begin
                        reason<=best_sad>MAX_SAD ? 3 : 4;scene_change<=best_sad>MAX_SAD*2;state<=INVALID;
                    end else begin
                        x1<=nx1[15:0];x2<=nx2[15:0];y1<=ny1[15:0];y2<=ny2[15:0];
                        patch_x<=patch_x+best_dx;patch_y<=patch_y+best_dy;tracked_fid<=job_fid;track_ok<=1;observation_ok<=1;
                        track_confidence<=best_sad>=2000 ? 0 : 1000-(best_sad>>1);reason<=0;
                        if(best_dx==SEARCH || best_dx==-SEARCH || best_dy==SEARCH || best_dy==-SEARCH ||
                            nx1<SCALE || ny1<SCALE || nx2>IMAGE_W-SCALE || ny2>IMAGE_H-SCALE)begin force_yolo<=1;reason<=7;end
                        state<=ROI_INIT;
                    end
                end
                ROI_INIT:begin
                    rx0<=x1/SCALE;ry0<=y1/SCALE;rx<=x1/SCALE;ry<=y1/SCALE;
                    rxmax<=(x2+SCALE-1)/SCALE;rymax<=(y2+SCALE-1)/SCALE;red_sum<=0;roi_cells<=0;state<=ROI_REQ;
                    if(job_fid==semantic_fid)begin
                        if(max_gray-min_gray<MIN_TEXTURE)begin tracking_allowed<=0;force_yolo<=1;reason<=8;end
                        else tracking_allowed<=1;
                    end
                end
                ROI_REQ:begin request_bank<=job_fid[HB-1:0];request_row<=ry;request_col<=rx;after_wait<=ROI_TAKE;state<=ADDR_ROW;end
                ROI_WAIT:begin state<=MEM_WAIT2;after_wait<=ROI_TAKE;end
                ROI_TAKE:begin
                    red_sum<=red_sum+{11'd0,read_cached[12:8]};roi_cells<=roi_cells+1'b1;
                    if(rx==rxmax-1)begin
                        rx<=rx0;if(ry==rymax-1)state<=DIV_START;else begin ry<=ry+1'b1;state<=ROI_REQ;end
                    end else begin rx<=rx+1'b1;state<=ROI_REQ;end
                end
                DIV_START:begin numerator<=red_sum*1000;denominator<={12'd0,roi_cells,4'd0};divide_start<=1;state<=DIV_WAIT;end
                DIV_WAIT:if(divide_done)begin red_ratio<=quotient[15:0];state<=PUBLISH;end
                INVALID:begin track_ok<=0;observation_ok<=0;tracking_allowed<=0;force_yolo<=1;track_confidence<=0;red_ratio<=0;state<=PUBLISH;end
                PUBLISH:begin
                    status_payload<={32'h46535441,8'd2,flags,reason,direction,job_fid,semantic_fid,class_id,confidence,
                        x1,y1,x2,y2,red_ratio,track_confidence,semantic_age,16'd0};
                    status_valid<=1;have_report<=1;reported_fid<=job_fid;state<=IDLE;
                end
            endcase
            // Writer may invalidate an old bank while processing; never use torn history.
            if(state!=IDLE && state!=SEED_INIT && state!=INVALID && state!=PUBLISH && !present(job_fid))begin
                history_miss<=1;reason<=5;state<=INVALID;
            end
        end
    end
endmodule
