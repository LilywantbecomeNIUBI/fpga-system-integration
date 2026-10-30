// Quarter-scale nearest-neighbour sampling; RGB565 -> RGB888 -> BT.601 luma.
// Gray coefficients 77/150/29 reuse ACG720 chapter39 FORMULA arithmetic.
// Red feature: HSV-equivalent |hue|<=20deg, delta>=48, V>=80 (no divider).
module vision_pixel_pipeline(
    input clk,rst_n,pixel_valid,input [15:0] pixel,x,y,frame_id,
    input frame_start,frame_end,frame_good,
    output reg out_valid,output reg [15:0] out_pixel,out_x,out_y,out_frame_id,
    output reg [7:0] luma,output reg red,
    output out_start,out_end,out_good
);
    reg v0,v1,v2,red2;reg [7:0] r0,g0,b0;
    reg [15:0] p0,p1,p2,x0,x1,x2,y0,y1,y2,f0,f1,f2,gray_sum,red_product,green_product,blue_product;
    reg [7:0] delta1,gb1,r1;reg red_max1;
    reg [3:0] starts,ends,goods;
    wire [7:0] minimum=g0<b0 ? g0 : b0;
    wire [15:0] rw={8'd0,r0},gw={8'd0,g0},bw={8'd0,b0};
    wire [9:0] hue_distance={2'd0,gb1}+{1'd0,gb1,1'b0};
    assign out_start=starts[3];assign out_end=ends[3];assign out_good=goods[3];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            v0<=0;v1<=0;v2<=0;red2<=0;out_valid<=0;r0<=0;g0<=0;b0<=0;p0<=0;p1<=0;p2<=0;
            x0<=0;x1<=0;x2<=0;y0<=0;y1<=0;y2<=0;f0<=0;f1<=0;f2<=0;gray_sum<=0;red_product<=0;green_product<=0;blue_product<=0;
            delta1<=0;gb1<=0;r1<=0;red_max1<=0;starts<=0;ends<=0;goods<=0;
            out_pixel<=0;out_x<=0;out_y<=0;out_frame_id<=0;luma<=0;red<=0;
        end else begin
            starts<={starts[2:0],frame_start};ends<={ends[2:0],frame_end};goods<={goods[2:0],frame_good};
            v0<=pixel_valid && x[1:0]==0 && y[1:0]==0;
            f0<=frame_id;
            if(pixel_valid)begin
                r0<={pixel[15:11],pixel[15:13]};g0<={pixel[10:5],pixel[10:9]};b0<={pixel[4:0],pixel[4:2]};
                p0<=pixel;x0<=x>>2;y0<=y>>2;
            end
            v1<=v0;p1<=p0;x1<=x0;y1<=y0;f1<=f0;
            red_product<=rw*16'd77;green_product<=gw*16'd150;blue_product<=bw*16'd29;
            delta1<=r0-minimum;gb1<=g0>b0 ? g0-b0 : b0-g0;r1<=r0;red_max1<=r0>=g0 && r0>=b0;
            v2<=v1;p2<=p1;x2<=x1;y2<=y1;f2<=f1;
            gray_sum<=red_product+green_product+blue_product;
            red2<=red_max1 && r1>=80 && delta1>=48 && hue_distance<={2'd0,delta1};
            out_valid<=v2;out_pixel<=p2;out_x<=x2;out_y<=y2;out_frame_id<=f2;
            luma<=gray_sum[15:8];red<=red2;
        end
    end
endmodule
