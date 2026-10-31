// V1 baseline: regional radial-expansion occupancy, with unknown/stale fail-closed.
// This is a geometry risk proxy, not distance, TTC, semantic classification or IMU compensation.
// The consuming control clock must independently time out snapshots if this clock stops.
module flow_avoidance #(parameter WIDTH=160,HEIGHT=90,MIN_VALID=4,
    EXPANSION_Q7=2,MIN_MOTION_Q7=32,TIMEOUT_CYCLES=8400000,CLEAR_FRAMES=2)(
    input clk,rst_n,frame_begin,frame_done,invalidate,sample_valid,sample_good,
    input [15:0] frame_id,x,y,input signed [15:0] u,v,
    output reg status_valid,output reg [15:0] status_id,
    output reg [2:0] state,hazard,output reg stop_request,
    output reg signed [1:0] steer,output reg [7:0] speed_limit,
    output reg [15:0] left_valid,center_valid,right_valid,left_risk,center_risk,right_risk
);
    localparam UNKNOWN=0,FORWARD=1,LEFT=2,RIGHT=3,BRAKE=4;
    reg [15:0] n[0:2],r[0:2];reg poisoned,expired;
    reg [31:0] age;reg [7:0] clear_count;
    wire signed [16:0] dx=$signed({1'b0,x})-WIDTH/2;
    wire signed [16:0] dy=$signed({1'b0,y})-HEIGHT/2;
    reg sample_d,sample_sum,sample_risk;
    reg [3:0] done_pipe;reg [15:0] done_id[0:3];
    reg [1:0] region_d,region_sum,region_risk;reg good_d,good_sum,good_risk;
    reg signed [33:0] dot_x,dot_y;reg [33:0] norm_x,norm_y;
    reg [16:0] motion;
    wire signed [34:0] dot=$signed({dot_x[33],dot_x})+$signed({dot_y[33],dot_y});
    wire [34:0] norm={1'b0,norm_x}+{1'b0,norm_y};
    reg signed [34:0] dot_r;reg [34:0] norm_r;
    reg moving_r,risk_flag;
    wire [2:0] dangers={n[2]>=MIN_VALID && ({2'd0,r[2]}<<2)>=n[2],
                        n[1]>=MIN_VALID && ({2'd0,r[1]}<<2)>=n[1],
                        n[0]>=MIN_VALID && ({2'd0,r[0]}<<2)>=n[0]};
    integer k;
    // Arithmetic data registers need no reset: reset/abort clears their valid
    // tags. This also permits the product registers to pack into DSP hardware.
    always @(posedge clk)begin
        if(sample_valid)begin
            dot_x<=dx*u;dot_y<=dy*v;norm_x<=dx*dx;norm_y<=dy*dy;
            motion<=(u<0 ? -{u[15],u} : {1'b0,u})+(v<0 ? -{v[15],v} : {1'b0,v});
        end
        if(sample_d)begin dot_r<=dot;norm_r<=norm;moving_r<=motion>=MIN_MOTION_Q7;end
        if(sample_sum)risk_flag<=dot_r>0 && $unsigned(dot_r)>=norm_r*EXPANSION_Q7 && moving_r;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            for(k=0;k<3;k=k+1)begin n[k]<=0;r[k]<=0;end
            poisoned<=1;expired<=1;age<=0;clear_count<=0;
            sample_d<=0;sample_sum<=0;sample_risk<=0;done_pipe<=0;
            region_d<=0;region_sum<=0;region_risk<=0;good_d<=0;good_sum<=0;good_risk<=0;
            for(k=0;k<4;k=k+1)done_id[k]<=0;
            status_valid<=0;status_id<=0;
            state<=UNKNOWN;hazard<=0;stop_request<=1;steer<=0;speed_limit<=0;
            left_valid<=0;center_valid<=0;right_valid<=0;left_risk<=0;center_risk<=0;right_risk<=0;
        end else begin
            status_valid<=0;sample_d<=sample_valid;good_d<=sample_good;
            sample_sum<=sample_d;sample_risk<=sample_sum;
            region_sum<=region_d;region_risk<=region_sum;good_sum<=good_d;good_risk<=good_sum;
            done_pipe<={done_pipe[2:0],frame_done};done_id[0]<=frame_id;
            for(k=1;k<4;k=k+1)done_id[k]<=done_id[k-1];
            if(sample_valid)begin
                region_d<=x<WIDTH/3 ? 0 : x<(2*WIDTH)/3 ? 1 : 2;
            end
            if(age<TIMEOUT_CYCLES)age<=age+1'b1;
            if(frame_begin)begin
                for(k=0;k<3;k=k+1)begin n[k]<=0;r[k]<=0;end
                poisoned<=0;sample_sum<=0;sample_risk<=0;done_pipe<=0;
            end
            for(k=0;k<3;k=k+1)if(sample_risk && good_risk && region_risk==k && !poisoned && !frame_begin)begin
                if(n[k]!=16'hffff)n[k]<=n[k]+1'b1;
                if(risk_flag && r[k]!=16'hffff)r[k]<=r[k]+1'b1;
            end
            // Include even a sample coincident with frame_done, then commit the
            // drained counts and the matching frame ID. A new frame flushes it.
            if(done_pipe[3] && !frame_begin)begin
                status_valid<=1;status_id<=done_id[3];
                left_valid<=n[0];center_valid<=n[1];right_valid<=n[2];left_risk<=r[0];center_risk<=r[1];right_risk<=r[2];
                if(!poisoned && n[0]>=MIN_VALID && n[1]>=MIN_VALID && n[2]>=MIN_VALID)begin
                    age<=0;expired<=0;hazard<=dangers;
                    if(dangers==0)begin
                        if(clear_count<CLEAR_FRAMES)clear_count<=clear_count+1'b1;
                        if(clear_count>=CLEAR_FRAMES-1)begin state<=FORWARD;stop_request<=0;steer<=0;speed_limit<=255;end
                        else begin state<=BRAKE;stop_request<=1;steer<=0;speed_limit<=0;end
                    end else begin
                        clear_count<=0;
                        if(dangers==3'b111 || (dangers[1] && dangers[0] && dangers[2]))begin state<=BRAKE;stop_request<=1;steer<=0;speed_limit<=0;end
                        else if((dangers[1] && !dangers[0] && (dangers[2] || r[0]<=r[2])) || (!dangers[1] && dangers[2] && !dangers[0]))begin
                            state<=LEFT;stop_request<=0;steer<=-1;speed_limit<=96;
                        end else if(!dangers[2] && (dangers[1] || dangers[0]))begin state<=RIGHT;stop_request<=0;steer<=1;speed_limit<=96;end
                        else begin state<=FORWARD;stop_request<=0;steer<=0;speed_limit<=64;end
                    end
                end else begin state<=UNKNOWN;hazard<=0;stop_request<=1;steer<=0;speed_limit<=0;clear_count<=0;end
            end
            if(invalidate || (age>=TIMEOUT_CYCLES-1 && !expired))begin
                poisoned<=1;expired<=1;status_valid<=1;status_id<=frame_id;
                state<=UNKNOWN;hazard<=0;stop_request<=1;steer<=0;speed_limit<=0;clear_count<=0;
            end
        end
    end
endmodule
