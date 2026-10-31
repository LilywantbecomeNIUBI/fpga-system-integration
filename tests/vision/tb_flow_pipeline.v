`timescale 1ns/1ps
module tb_flow_pipeline #(parameter WIDTH=64,HEIGHT=48,PYRAMID=0);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,fs=0,fe=0,fg=0,pv=0;reg [15:0] fid=0,x=0,y=0;reg [7:0] luma=0;
    wire busy,done,sv,good,lost,pair_begin;wire [15:0] rid,fx,fy;wire signed [15:0] u,v;wire [31:0] cycles,drops;
    localparam TOTAL=WIDTH*HEIGHT,POINTS=((WIDTH-16)/8+1)*((HEIGHT-16)/8+1);
    reg [7:0] a[0:TOTAL-1],b[0:TOTAL-1];reg [1023:0] base,filename;
    integer out,status_log,ix,iy,count=0;
    flow_pipeline #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.PYRAMID(PYRAMID)) dut(.clk(clk),.rst_n(rst_n),.invalidate(1'b0),.frame_start(fs),.frame_end(fe),.frame_good(fg),
        .pixel_valid(pv),.frame_id(fid),.x(x),.y(y),.luma(luma),.busy(busy),.frame_done(done),
        .sample_valid(sv),.sample_good(good),.history_lost(lost),.pair_begin(pair_begin),.result_id(rid),.flow_x(fx),.flow_y(fy),.flow_u(u),.flow_v(v),
        .cycles(cycles),.dropped_frames(drops));
    always @(negedge clk)if(sv)begin
        $fdisplay(out,"%0d %0d %0d %0d %0d %0d",rid,fx,fy,u,v,good);count=count+1;
    end
    wire status,stop;wire [2:0] state;wire signed [1:0] steer;wire [7:0] speed;
    wire [15:0] nl,nc,nr,rl,rc,rr;
    flow_avoidance #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) safety(.clk(clk),.rst_n(rst_n),.frame_begin(pair_begin),.frame_done(done),
        .invalidate(lost),.sample_valid(sv),.sample_good(good),.frame_id(rid),.x(fx),.y(fy),.u(u),.v(v),.status_valid(status),.status_id(),
        .state(state),.hazard(),.stop_request(stop),.steer(steer),.speed_limit(speed),
        .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr));
    always @(negedge clk)if(status)$fdisplay(status_log,"%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",rid,nl,nc,nr,rl,rc,rr,state,stop,steer,speed);
    task send_frame;
        input [15:0] id;input second;input verdict;
        begin
            @(negedge clk);fid=id;fs=1;@(negedge clk);fs=0;
            for(iy=0;iy<HEIGHT;iy=iy+1)begin
                for(ix=0;ix<WIDTH;ix=ix+1)begin
                    @(negedge clk);pv=1;x=ix;y=iy;luma=second ? b[iy*WIDTH+ix] : a[iy*WIDTH+ix];
                    // Deliberate valid bubbles; no hidden contiguous-stream assumption.
                    if(ix%7==0)begin @(negedge clk);pv=0;end
                end
            end
            @(negedge clk);pv=0;fe=1;fg=verdict;@(negedge clk);fe=0;fg=0;
        end
    endtask
    initial begin
        if(!$value$plusargs("CASE=%s",base))$fatal(1,"CASE missing");
        filename={base,"/a.mem"};$readmemh(filename,a);filename={base,"/b.mem"};$readmemh(filename,b);
        filename={base,"/pipeline.txt"};out=$fopen(filename,"w");
        filename={base,"/status.txt"};status_log=$fopen(filename,"w");
        repeat(5)@(negedge clk);rst_n=1;
        send_frame(16'hfffe,0,1);repeat(5)@(negedge clk);if(busy || count!=0)$fatal(1,"first frame used uninitialized history");
        send_frame(16'hffff,1,1);wait(done);@(negedge clk);
        if(count!=POINTS || drops!=0)$fatal(1,"pair size/drop %0d %0d",count,drops);
        $display("FIRST_PAIR_CYCLES=%0d",cycles);
        // Rollover remains consecutive. Capture overlaps computation in a third bank.
        send_frame(16'h0000,0,1);wait(busy);send_frame(16'h0001,1,1);
        // The second FE arrives while busy: drop rather than overwrite owned RAM.
        if(drops!=1)$fatal(1,"overrun not recorded");
        wait(done);repeat(4)@(negedge clk);
        send_frame(16'h0002,0,0);repeat(4)@(negedge clk);
        if(busy)$fatal(1,"bad frame launched LK");
        send_frame(16'h0003,0,1);repeat(4)@(negedge clk);if(busy)$fatal(1,"history crossed rejected frame");
        send_frame(16'h0005,1,1);repeat(4)@(negedge clk);if(busy)$fatal(1,"history crossed frame-id gap");
        $fclose(out);$fclose(status_log);$display("PASS frame ownership, exact grid, bubbles, rollover, overrun, bad frame and gap; cycles=%0d",cycles);$finish;
    end
    initial begin #100000000;$fatal(1,"pipeline timeout");end
endmodule
