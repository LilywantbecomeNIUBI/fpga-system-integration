`timescale 1ns/1ps
// Continuous input, coalescing under backpressure, and coherent frame tags.
module tb_event_ingress_bursts;
    reg clk=0,rst=0,update=0,ready=0;always #5 clk=~clk;
    reg [255:0] f=0,v=0,s=0;wire fv,vv;wire [255:0] fp,vp;
    flow_event_bridge #(.CLK_HZ(1000),.TIMEOUT_MS(10000)) flow(
        .clk(clk),.rst_n(rst),.stream_ready(1'b1),.observation_valid(update),.observation_payload(f),
        .event_ready(ready),.event_valid(fv),.event_payload(fp));
    vision_v23_bridge #(.CLK_HZ(1000),.TIMEOUT_MS(10000)) vision(
        .clk(clk),.rst_n(rst),.stream_ready(1'b1),.observation_valid(update),.observation_payload(v),
        .semantic_update(update),.semantic_valid(1'b1),.semantic_payload(s),
        .event_ready(ready),.event_valid(vv),.event_payload(vp));
    integer i,fn=0,vn=0,last_f=-1,last_v=-1;
    always @(posedge clk)if(rst && ready)begin
        if(fv && fp[63:32]!=0)begin
            if(fp[63:32]!=(32'h51880000+fp[191:176]) || (last_f>=0 && fp[191:176]<last_f))
                $fatal(1,"flow frame/payload mixed or reordered");
            if(fp[201] && fp[79:72]!=(fp[191:176]&255))$fatal(1,"flow control/frame mixed");
            last_f=fp[191:176];fn=fn+1;
        end
        if(vv && vp[159:128]!=0)begin
            if(vp[159:128]!=(32'h71880000+vp[175:160]) || (last_v>=0 && vp[175:160]<last_v))
                $fatal(1,"vision frame/payload mixed or reordered");
            last_v=vp[175:160];vn=vn+1;
        end
    end
    initial begin
        repeat(4)@(negedge clk);rst=1;
        for(i=0;i<500;i=i+1)begin
            update=1;ready=i%17<11;
            f=0;f[255:224]=32'h4f465354;f[207:200]=2;f[191:176]=i;
            f[79:72]=i;f[63:32]=32'h51880000+i;
            v=0;v[255:216]={32'h46535441,8'd2};v[208]=1;v[199:192]=2;
            v[175:160]=i;v[159:128]=32'h71880000+i;s[207:192]=i;
            @(negedge clk);
        end
        update=0;ready=1;repeat(30)@(negedge clk);
        if(fn<20 || vn<20 || last_f!=499 || last_v!=499)$fatal(1,"continuous stream starved or latest frame lost");
        $display("PASS continuous ingress: coherent frame/control tags, no reordering/starvation, latest frame retained under stalls");$finish;
    end
endmodule
