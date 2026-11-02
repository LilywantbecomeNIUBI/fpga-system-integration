// Immutable triple-frame ownership; synchronous BSRAM is imported from upstream.
// Capture never stalls. Invalid/incomplete/nonconsecutive frames cannot form a pair.
module flow_frame_store #(parameter WIDTH=160,HEIGHT=90,PYRAMID=0)(
    input clk,rst_n,invalidate,frame_start,frame_end,frame_good,pixel_valid,
    input [15:0] frame_id,x,y,input [7:0] luma,
    input processing,read_enable,read_current,read_coarse,input [15:0] read_address,
    output [7:0] read_data,
    output reg pair_start,output reg [15:0] pair_id,
    output reg rejected,output reg [31:0] dropped_frames
);
    localparam AB=$clog2(WIDTH*HEIGHT),TOTAL=WIDTH*HEIGHT;
    reg [1:0] cap_bank,last_bank,previous_bank,current_bank;
    reg have_previous,active,bad;reg [15:0] written,expected_x,expected_y,cap_id,last_id;
    reg [1:0] free_bank;integer k;
    wire occupied=processing || pair_start;
    always @*begin
        free_bank=0;
        for(k=0;k<3;k=k+1)
            if((!have_previous || k!=last_bank) && (!occupied || (k!=previous_bank && k!=current_bank)))free_bank=k;
    end
    wire [1:0] selected_bank=read_current ? current_bank : previous_bank;
    wire [AB+1:0] wa={cap_bank,written[AB-1:0]},ra={selected_bank,read_address[AB-1:0]};
    wire write_enable=pixel_valid && active && !bad && written<TOTAL && x==expected_x && y==expected_y;
    wire [7:0] fine_data;
    frame_memory #(.DATA_WIDTH(8),.ADDR_WIDTH(AB+2),.DEPTH(3*(1<<AB))) frames(
        .clk(clk),.addr_a(wa),.data_a(luma),.we_a(write_enable),.addr_b(ra),.data_b(fine_data),.re_b(read_enable && !read_coarse));
    generate if(PYRAMID)begin:pyramid
        // 2x2 box filter: reuse the row-sum / registered-column pattern of vision_feature_grid.
        localparam CB=$clog2((WIDTH/2)*(HEIGHT/2));
        reg [7:0] left;reg [8:0] row_pairs[0:WIDTH/2-1];
        reg a_valid,b_valid,a_phase,b_phase;reg [15:0] a_col;
        reg [8:0] a_sum,b_sum,previous;reg [CB+1:0] a_addr,b_addr;
        wire [9:0] total={1'b0,previous}+{1'b0,b_sum};wire [7:0] coarse_data;
        wire [CB+1:0] coarse_read={selected_bank,read_address[CB-1:0]};
        frame_memory #(.DATA_WIDTH(8),.ADDR_WIDTH(CB+2),.DEPTH(3*(1<<CB))) frames(
            .clk(clk),.addr_a(b_addr),.data_a(total[9:2]),.we_a(b_valid && b_phase),
            .addr_b(coarse_read),.data_b(coarse_data),.re_b(read_enable && read_coarse));
        always @(posedge clk)begin
            // Even rows only write; odd rows only read. The old value on an
            // even-row write was unused, and requesting it inferred unsupported
            // SPX9 read-before-write mode on GW5AT. Keep the two roles disjoint.
            if(a_valid && a_phase)previous<=row_pairs[a_col];
            if(a_valid && !a_phase)row_pairs[a_col]<=a_sum;
        end
        wire [15:0] coarse_index=(y>>1)*(WIDTH/2)+(x>>1);
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin left<=0;a_valid<=0;b_valid<=0;a_phase<=0;b_phase<=0;a_col<=0;a_sum<=0;b_sum<=0;a_addr<=0;b_addr<=0;end
            else begin
                a_valid<=write_enable && x[0];b_valid<=a_valid;b_phase<=a_phase;b_sum<=a_sum;b_addr<=a_addr;
                if(write_enable)begin
                    if(!x[0])left<=luma;
                    else begin a_sum<={1'b0,left}+{1'b0,luma};a_col<=x>>1;a_phase<=y[0];a_addr<={cap_bank,coarse_index[CB-1:0]};end
                end
            end
        end
        assign read_data=read_coarse ? coarse_data : fine_data;
    end else begin:no_pyramid
        assign read_data=fine_data;
    end endgenerate
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            cap_bank<=0;last_bank<=0;previous_bank<=0;current_bank<=0;have_previous<=0;active<=0;bad<=0;
            written<=0;expected_x<=0;expected_y<=0;cap_id<=0;last_id<=0;pair_start<=0;pair_id<=0;rejected<=0;dropped_frames<=0;
        end else begin
            pair_start<=0;rejected<=0;
            if(frame_start)begin
                cap_bank<=free_bank;active<=1;bad<=0;written<=0;expected_x<=0;expected_y<=0;cap_id<=frame_id;
                if(active)begin rejected<=1;have_previous<=0;dropped_frames<=dropped_frames+1'b1;end
            end
            if(pixel_valid && active)begin
                if(written>=TOTAL || x!=expected_x || y!=expected_y)bad<=1;
                if(written<TOTAL)written<=written+1'b1;
                if(expected_x==WIDTH-1)begin expected_x<=0;expected_y<=expected_y+1'b1;end else expected_x<=expected_x+1'b1;
            end
            if(frame_end && active)begin
                active<=0;
                if(frame_good && !bad && written==TOTAL && !occupied)begin
                    if(have_previous && cap_id==last_id+16'd1)begin
                        previous_bank<=last_bank;current_bank<=cap_bank;pair_id<=cap_id;pair_start<=1;
                    end else if(have_previous)rejected<=1;
                    last_bank<=cap_bank;last_id<=cap_id;have_previous<=1;
                end else begin have_previous<=0;rejected<=1;dropped_frames<=dropped_frames+1'b1;end
            end
            if(invalidate)begin
                have_previous<=0;active<=0;bad<=1;pair_start<=0;rejected<=1;
                dropped_frames<=dropped_frames+1'b1;
            end
        end
    end
endmodule
