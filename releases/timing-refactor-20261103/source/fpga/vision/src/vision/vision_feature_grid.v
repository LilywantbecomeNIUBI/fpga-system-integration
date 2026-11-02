// 4x4 reduced pixels -> one feature cell (16x16 pixels in the original image).
// Each feature contains mean luma and exact red count among the 16 samples.
module vision_feature_grid #(parameter WIDTH=320)(
    input clk,rst_n,frame_start,pixel_valid,input [15:0] x,y,
    input [7:0] luma,input red,
    output reg feature_valid,output reg [15:0] feature_address,feature
);
    localparam COLS=WIDTH/4;
    reg [11:0] vertical_y[0:COLS-1];reg [4:0] vertical_r[0:COLS-1];
    reg [9:0] horizontal_y;reg [2:0] horizontal_r;
    wire [9:0] row_y=x[1:0]==0 ? {2'd0,luma} : horizontal_y+{2'd0,luma};
    wire [2:0] row_r=x[1:0]==0 ? {2'd0,red} : horizontal_r+{2'd0,red};
    reg a_valid,b_valid;reg [15:0] a_col,b_col,a_row,b_row;
    reg [1:0] a_phase,b_phase;reg [9:0] a_y,b_y;reg [2:0] a_r,b_r;
    reg [11:0] previous_y;reg [4:0] previous_r;
    wire [11:0] total_y=b_phase==0 ? {2'd0,b_y} : previous_y+{2'd0,b_y};
    wire [4:0] total_r=b_phase==0 ? {2'd0,b_r} : previous_r+{2'd0,b_r};
    always @(posedge clk)begin
        if(a_valid)begin previous_y<=vertical_y[a_col];previous_r<=vertical_r[a_col];end
        if(b_valid)begin vertical_y[b_col]<=total_y;vertical_r[b_col]<=total_r;end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            horizontal_y<=0;horizontal_r<=0;feature_valid<=0;feature_address<=0;feature<=0;
            a_valid<=0;b_valid<=0;a_col<=0;b_col<=0;a_row<=0;b_row<=0;a_phase<=0;b_phase<=0;a_y<=0;b_y<=0;a_r<=0;b_r<=0;
        end
        else begin
            feature_valid<=0;a_valid<=0;b_valid<=a_valid;
            b_col<=a_col;b_row<=a_row;b_phase<=a_phase;b_y<=a_y;b_r<=a_r;
            if(frame_start)begin horizontal_y<=0;horizontal_r<=0;end
            if(pixel_valid)begin
                horizontal_y<=row_y;horizontal_r<=row_r;
                if(x[1:0]==3)begin
                    a_valid<=1;a_col<=x>>2;a_row<=y>>2;a_phase<=y[1:0];a_y<=row_y;a_r<=row_r;
                end
            end
            if(b_valid && b_phase==3)begin feature_valid<=1;feature_address<=b_row*COLS+b_col;feature<={3'd0,total_r,total_y[11:4]};end
        end
    end
endmodule
