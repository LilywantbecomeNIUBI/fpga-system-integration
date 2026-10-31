// ACG720 DVP_Capture/camera_udp_rows input-register and RGB565 assembly pattern.
// Publish frame completion only at VSYNC, after validating every complete line.
module vision_dvp_capture #(parameter WIDTH=1280,HEIGHT=720,STARTUP_FRAMES=10)(
    input clk,rst_n,vsync,href,input [7:0] data,
    output reg pixel_valid,output reg [15:0] pixel,x,y,frame_id,
    output reg frame_start,frame_end,frame_good,output reg format_error
);
    reg sv,sh,vd,hd,phase,active,bad;
    reg [7:0] sd,first_byte,startups;
    reg [15:0] bytes_in_line,rows;
    always @(posedge clk)begin sv<=vsync;sh<=href;sd<=data;end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            vd<=0;hd<=0;phase<=0;active<=0;bad<=0;first_byte<=0;startups<=0;
            bytes_in_line<=0;rows<=0;pixel_valid<=0;pixel<=0;x<=0;y<=0;frame_id<=0;
            frame_start<=0;frame_end<=0;frame_good<=0;format_error<=0;
        end else begin
            vd<=sv;hd<=sh;pixel_valid<=0;frame_start<=0;frame_end<=0;frame_good<=0;
            if(sv && !vd)begin
                frame_id<=frame_id+1'b1;
                if(startups<STARTUP_FRAMES)startups<=startups+1'b1;
                if(active)begin
                    frame_end<=1;frame_good<=!bad && rows==HEIGHT && !phase && !sh;
                    if(bad || rows!=HEIGHT || phase || sh)format_error<=1;
                end
                active<=0;bad<=0;rows<=0;bytes_in_line<=0;phase<=0;
            end else if(!sv && vd && startups>=STARTUP_FRAMES)begin
                frame_start<=1;active<=1;bad<=0;rows<=0;bytes_in_line<=0;phase<=0;
            end else if(active && !sv)begin
                if(sh)begin
                    if(bytes_in_line<WIDTH*2 && rows<HEIGHT && !bad)begin
                        bytes_in_line<=bytes_in_line+1'b1;
                        if(!phase)begin first_byte<=sd;phase<=1;end
                        else begin
                            pixel<={first_byte,sd};pixel_valid<=1;
                            x<=bytes_in_line[15:1];y<=rows;phase<=0;
                        end
                    end else begin bad<=1;format_error<=1;end
                end else if(hd)begin
                    if(bytes_in_line!=WIDTH*2 || phase)begin bad<=1;format_error<=1;end
                    rows<=rows+1'b1;bytes_in_line<=0;phase<=0;
                end
            end
        end
    end
endmodule
