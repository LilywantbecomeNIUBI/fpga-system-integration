`timescale 1ns/1ps
module tb_flow_avoidance;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,fb=0,fd=0,inv=0,sv=0,good=1;reg [15:0] x=0,y=0,fid=0;reg signed [15:0] u=0,v=0;
    wire status,stop;wire [2:0] state,hazard;wire signed [1:0] steer;wire [7:0] speed;
    wire [15:0] nl,nc,nr,rl,rc,rr;
    flow_avoidance #(.TIMEOUT_CYCLES(1000)) dut(.clk(clk),.rst_n(rst_n),.frame_begin(fb),.frame_done(fd),.invalidate(inv),
        .sample_valid(sv),.sample_good(good),.frame_id(fid),.x(x),.y(y),.u(u),.v(v),.status_valid(status),.status_id(),
        .state(state),.hazard(hazard),.stop_request(stop),.steer(steer),.speed_limit(speed),
        .left_valid(nl),.center_valid(nc),.right_valid(nr),.left_risk(rl),.center_risk(rc),.right_risk(rr));
    integer i,j;
    task field;
        input [2:0] risk;input enough;input recede;
        begin
            @(negedge clk);fb=1;fid=fid+1;@(negedge clk);fb=0;
            for(i=0;i<3;i=i+1)for(j=0;j<6;j=j+1)begin
                @(negedge clk);sv=1;good=enough;x=20+i*60;y=16+j*8;
                u=risk[i] ? ($signed({1'b0,x})-80)*6 : 0;
                v=risk[i] ? ($signed({1'b0,y})-45)*6 : 0;
                if(recede)begin u=-u;v=-v;end
                @(negedge clk);sv=0;
            end
            repeat(3)@(negedge clk);fd=1;@(negedge clk);fd=0;repeat(2)@(negedge clk);
        end
    endtask
    initial begin
        repeat(5)@(negedge clk);rst_n=1;
        if(!stop || state!=0)$fatal(1,"reset must stop");
        field(0,1,0);if(!stop)$fatal(1,"clear hysteresis");
        field(0,1,0);if(stop || state!=1 || speed!=255)$fatal(1,"two clear frames release");
        field(3'b001,1,0);if(hazard!=1 || steer!=1 || state!=3)$fatal(1,"left danger -> right");
        field(3'b100,1,0);if(hazard!=4 || steer!=-1 || state!=2)$fatal(1,"right danger -> left");
        field(3'b010,1,0);if(hazard!=2 || steer!=-1)$fatal(1,"center tie -> left");
        field(3'b011,1,0);if(steer!=1)$fatal(1,"center+left -> right");
        field(3'b110,1,0);if(steer!=-1)$fatal(1,"center+right -> left");
        field(3'b111,1,0);if(!stop || state!=4)$fatal(1,"all sectors -> brake");
        // Two center samples are below MIN_MOTION_Q7; they count as valid but not risk.
        if(nl!=6 || nc!=6 || nr!=6 || rl!=6 || rc!=4 || rr!=6)$fatal(1,"regional counts %0d %0d %0d / %0d %0d %0d",nl,nc,nr,rl,rc,rr);
        field(3'b111,1,1);field(3'b111,1,1);if(stop || hazard!=0)$fatal(1,"receding is not approaching");
        field(0,0,0);if(!stop || state!=0)$fatal(1,"low confidence -> unknown");
        field(0,1,0);field(0,1,0);@(negedge clk);inv=1;@(negedge clk);inv=0;
        if(!stop || state!=0)$fatal(1,"invalid must stop immediately");
        field(0,1,0);field(0,1,0);repeat(1005)@(negedge clk);
        if(!stop || state!=0)$fatal(1,"stale must stop");
        $display("PASS regional counts, expansion/recession, steering, all-blocked, texture, clear hysteresis, invalidation and timeout");$finish;
    end
endmodule
