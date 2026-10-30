`timescale 1ns/1ps
module tb_flow_patch;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,start=0;reg [15:0] x=0,y=0;reg signed [15:0] su=0,sv=0;
    wire busy,done,good,re,rc;wire [15:0] addr;wire signed [15:0] u,v;reg [7:0] data;
    reg [7:0] a[0:3071],b[0:3071];
    flow_lk_patch #(.WIDTH(64),.HEIGHT(48)) dut(.clk(clk),.rst_n(rst_n),.start(start),.center_x(x),.center_y(y),
        .coarse(1'b0),
        .initial_u(su),.initial_v(sv),.busy(busy),.done(done),.valid(good),.u(u),.v(v),
        .read_enable(re),.read_current(rc),.read_address(addr),.read_data(data));
    always @(posedge clk)if(re)begin
        if(addr>=3072)$fatal(1,"out-of-bounds RAM read %0d",addr);
        data<=rc ? b[addr] : a[addr];
    end
    integer fd,n,eu,ev,eg,ix,iy,isu,isv,count=0;
    reg [1023:0] base,filename;
    initial begin
        if(!$value$plusargs("CASE=%s",base))$fatal(1,"CASE missing");
        filename={base,"/a.mem"};$readmemh(filename,a);filename={base,"/b.mem"};$readmemh(filename,b);
        filename={base,"/patch.txt"};fd=$fopen(filename,"r");if(!fd)$fatal(1,"vectors missing");
        repeat(5)@(negedge clk);rst_n=1;
        while(!$feof(fd))begin
            n=$fscanf(fd,"%d %d %d %d %d %d %d\n",ix,iy,isu,isv,eu,ev,eg);
            if(n!=7)$fatal(1,"bad oracle record");
            @(negedge clk);x=ix;y=iy;su=isu;sv=isv;start=1;
            @(negedge clk);start=0;
            wait(done);@(negedge clk);
            if(good!==eg[0] || $signed(u)!==eu || $signed(v)!==ev)
                $fatal(1,"patch(%0d,%0d) seed(%0d,%0d) actual(%0d,%0d,%0d) expected(%0d,%0d,%0d)",ix,iy,isu,isv,u,v,good,eu,ev,eg);
            count=count+1;
        end
        $display("PASS %0d bit-exact LK patches",count);$finish;
    end
    initial begin #100000000;$fatal(1,"patch timeout");end
endmodule
