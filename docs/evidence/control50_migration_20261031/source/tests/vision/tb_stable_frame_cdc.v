`timescale 1ns/1ps
module tb_stable_frame_cdc;
    reg wc=0,rc=0,run_wr=1,run_rd=1,rst=0,ready=0;
    real wh=5.95235,rh=10.0;
    always begin #(wh);if(run_wr)wc=~wc;else wc=0;end
    initial begin #3.37;forever begin #(rh);if(run_rd)rc=~rc;else rc=0;end end
    reg fs=0,fe=0,fg=0,pv=0,red=0;reg [15:0] fid=0,x=0,y=0;reg [7:0] gray=0;
    wire os,oe,og,ov,orr,invalid;wire [15:0] ofid,ox,oy;wire [7:0] ol;
    wire [31:0] overflows,aborts;
    vision_frame_cdc #(.FIFO_AB(4),.TIMEOUT_CYCLES(20000)) dut(
        .wr_clk(wc),.wr_rst_n(rst),.capture_ready(ready),.frame_start(fs),.frame_end(fe),.frame_good(fg),
        .pixel_valid(pv),.frame_id(fid),.x(x),.y(y),.luma(gray),.red(red),
        .rd_clk(rc),.rd_rst_n(rst),.out_start(os),.out_end(oe),.out_good(og),.out_valid(ov),
        .out_id(ofid),.out_x(ox),.out_y(oy),.out_luma(ol),.out_red(orr),.invalidate(invalid),
        .overflow_count(overflows),.abort_count(aborts));
    wire fv;wire [15:0] fa,feature;
    vision_feature_grid grid(.clk(rc),.rst_n(rst),.frame_start(os),.pixel_valid(ov),.x(ox),.y(oy),
        .luma(ol),.red(orr),.feature_valid(fv),.feature_address(fa),.feature(feature));
    reg processing=0,re=0,current=0;reg [15:0] ra=0;
    wire [7:0] rd;wire pair,rejected;wire [15:0] pair_id;wire [31:0] dropped;
    flow_frame_store store(.clk(rc),.rst_n(rst),.invalidate(invalid),.frame_start(os),.frame_end(oe),.frame_good(og),
        .pixel_valid(ov && !ox[0] && !oy[0]),.frame_id(ofid),.x(ox>>1),.y(oy>>1),.luma(ol),
        .processing(processing),.read_enable(re),.read_current(current),.read_coarse(1'b0),.read_address(ra),.read_data(rd),
        .pair_start(pair),.pair_id(pair_id),.rejected(rejected),.dropped_frames(dropped));
    wire sv;wire [255:0] status;
    vision_tracker tracker(.clk(rc),.rst_n(rst),.invalidate(invalid),.frame_start(os),.frame_end(oe),.frame_good(og),.frame_id(ofid),
        .feature_valid(fv),.feature_address(fa),.feature(feature),.seed_valid(1'b0),.seed_data(257'd0),.status_valid(sv),.status_payload(status));
    function [7:0] value(input integer f,xx,yy);value=(f*19+xx*3+yy*5)&255;endfunction
    reg checking=1,inframe=0;integer pixels=0,features=0,ends=0,pairs=0,invalids=0;
    integer xx,yy,expected,sum,rr,cc;reg [15:0] seen_id;
    always @(posedge rc)if(rst)begin
        if(invalid)begin invalids=invalids+1;inframe=0;end
        if(os)begin
            inframe=1;seen_id=ofid;pixels=0;features=0;
            if(ov || oe)$fatal(1,"start must precede data and end");
        end
        if(checking && ov)begin
            if(!inframe || ox!=pixels%320 || oy!=pixels/320 || ofid!=seen_id ||
               ol!==value(seen_id,ox,oy) || orr!==((ox+oy)&1))$fatal(1,"pixel order/data f=%d index=%d x=%d y=%d",ofid,pixels,ox,oy);
            pixels=pixels+1;
        end
        if(checking && fv)begin
            sum=0;
            for(rr=0;rr<4;rr=rr+1)for(cc=0;cc<4;cc=cc+1)sum=sum+value(seen_id,(features%80)*4+cc,(features/80)*4+rr);
            if(fa!==features || feature!=={3'd0,5'd8,8'(sum/16)})$fatal(1,"feature mismatch %d %h expected mean=%d",fa,feature,sum/16);
            features=features+1;
        end
        if(checking && oe)begin
            if(!inframe || !og || pixels!=57600 || features!=3600)$fatal(1,"incomplete frame sealed pixels=%d features=%d",pixels,features);
            ends=ends+1;inframe=0;
        end
        if(pair)begin pairs=pairs+1;processing=1;end
    end
    task event_tick;begin @(negedge wc);fs=0;fe=0;pv=0;fg=0;end endtask
    task settle;begin repeat(150)@(negedge wc);end endtask
    task image(input integer f);integer i;begin
        for(i=0;i<57600;i=i+1)begin
            @(negedge wc);fid=f;fs=i==0;fe=i==57599;fg=fe;pv=1;
            x=i%320;y=i/320;gray=value(f,x,y);red=(x+y)&1;
            event_tick();repeat(6)@(negedge wc);
        end
        settle();
    end endtask
    task read_check(input integer f,input integer use_current);integer i;begin
        for(i=0;i<14400;i=i+997)begin
            @(negedge rc);re=1;current=use_current;ra=i;
            @(negedge rc);re=0;
            if(rd!==value(f,(i%160)*2,(i/160)*2))$fatal(1,"leased RAM was overwritten f=%d addr=%d got=%h",f,i,rd);
        end
    end endtask
    initial begin
        if($value$plusargs("WRITE_HALF=%f",wh))begin end
        if($value$plusargs("READ_HALF=%f",rh))begin end
        #137;rst=1;ready=1;settle();
        image(1);image(2);
        if(ends!=2 || pairs!=1 || pair_id!=2 || overflows!=0)$fatal(1,"normal CDC pairing %d %d",ends,pairs);
        read_check(1,0);read_check(2,1);
        image(3);read_check(1,0);read_check(2,1);
        if(pairs!=1 || dropped==0)$fatal(1,"busy ownership not respected");
        processing=0;
        // Camera reset never resets FIFO pointers or LK black box; invalidate tags.
        checking=0;@(negedge wc);ready=0;settle();
        if(tracker.sealed!=0 || store.have_previous || tracker.active)$fatal(1,"camera reset retained history");
        @(negedge wc);ready=1;settle();checking=1;image(0);image(1);
        if(pairs!=2 || pair_id!=1)$fatal(1,"new capture epoch pairing");processing=0;
        // Pause the consumer to force full FIFO. A dropped tail cannot seal.
        checking=0;run_rd=0;
        @(negedge wc);fs=1;fid=77;event_tick();
        repeat(80)begin pv=1;x=x+1;@(negedge wc);end
        pv=0;fe=1;fg=1;event_tick();run_rd=1;settle();
        if(overflows==0 || tracker.sealed!=0 || store.have_previous)$fatal(1,"overflow failed closed");
        checking=1;image(10);image(11);if(pairs!=3 || pair_id!=11)$fatal(1,"overflow recovery");processing=0;
        // A corrupted coordinate must invalidate even with an asserted good end.
        checking=0;
        @(negedge wc);fs=1;fid=12;event_tick();
        @(negedge wc);pv=1;x=7;y=0;event_tick();settle();
        if(tracker.sealed!=0 || store.have_previous)$fatal(1,"malformed coordinates retained history");
        // Stop PCLK while the stable clock runs; timeout is not a solver reset.
        checking=0;run_wr=0;repeat(20500)@(negedge rc);
        if(tracker.sealed!=0 || store.have_previous || invalids<3)$fatal(1,"PCLK stop retained healthy history");
        run_wr=1;settle();checking=1;image(20);image(21);
        if(pairs!=4 || pair_id!=21)$fatal(1,"PCLK restart pairing");
        // Reset both pointer domains together with data in flight.
        checking=0;rst=0;#173;rst=1;processing=0;settle();checking=1;image(30);image(31);
        if(pair_id!=31)$fatal(1,"system reset recovery");
        $display("PASS stable CDC wh=%0f rh=%0f: ordered/coincident events, exact features, RAM lease, overflow, camera reset, stopped PCLK, common reset",wh,rh);$finish;
    end
    initial begin #150000000;$fatal(1,"CDC timeout");end
endmodule
