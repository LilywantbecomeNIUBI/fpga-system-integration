`timescale 1ns/1ps
module tb_reset_mailbox_banks;
    parameter WIDTH=257;
    reg sc=0,dc=0,arst=0,valid=0;reg [WIDTH-1:0] data=0;
    always #4 sc=~sc;always #10 dc=~dc;
    wire sr,dr;wire [3:0] sb;wire [1:0] db;
    reset_sync #(.COPIES(4)) source_reset(.clk(sc),.arst_n(arst),.rst_n(sr),.rst_copies(sb));
    reset_sync #(.COPIES(2)) destination_reset(.clk(dc),.arst_n(arst),.rst_n(dr),.rst_copies(db));
    reg [2:0] source_ref=0,destination_ref=0;
    always @(posedge sc or negedge arst)if(!arst)source_ref<=0;else source_ref<={source_ref[1:0],1'b1};
    always @(posedge dc or negedge arst)if(!arst)destination_ref<=0;else destination_ref<={destination_ref[1:0],1'b1};
    wire dv,rv;wire [WIDTH-1:0] dd,rd;
    vision_mailbox #(.WIDTH(WIDTH),.SPLIT_RESET(1)) dut(.src_clk(sc),.src_rst_n(sr),.src_valid(valid),.src_data(data),
        .dst_clk(dc),.dst_rst_n(dr),.dst_valid(dv),.dst_data(dd),.src_reset_bank(sb),.dst_reset_bank(db));
    vision_mailbox_reference #(.WIDTH(WIDTH)) reference(.src_clk(sc),.src_rst_n(source_ref[2]),.src_valid(valid),.src_data(data),
        .dst_clk(dc),.dst_rst_n(destination_ref[2]),.dst_valid(rv),.dst_data(rd));
    always @(negedge sc)begin
        if(sb!=={4{source_ref[2]}} || sr!==source_ref[2])$fatal(1,"source release latency");
        if(dut.held!==reference.held || dut.newest!==reference.newest || dut.req!==reference.req)$fatal(1,"source mailbox divergence");
    end
    always @(negedge dc)begin
        if(db!=={2{destination_ref[2]}} || dr!==destination_ref[2])$fatal(1,"destination release latency");
        if(dv!==rv || dd!==rd)$fatal(1,"destination mailbox divergence");
    end
    integer i;
    initial begin
        #1;arst=1;#1;arst=0;#29;arst=1;
        for(i=0;i<6000;i=i+1)begin
            @(negedge sc);valid=(i%11)!=0;data={9{32'h1379abcd*i}};
            if(i%317==100)begin #1;arst=0;#1;
                if(sb!==0 || db!==0)$fatal(1,"async reset assertion");
                #13;arst=1;
            end
        end
        @(negedge sc);valid=0;repeat(50)@(negedge dc);
        $display("PASS parallel reset: async assertion/third-edge release; split mailbox cycle equivalence WIDTH=%d",WIDTH);$finish;
    end
endmodule
