`timescale 1ns/1ps
// Exercise both ownership slots, all four address banks, full-frame dropping,
// pointer wrap and slot reuse with a much faster receiving clock.
module tb_frame_banks #(parameter W=4,H=2,C=4);
    reg wc=0,rc=0,rst=0,fs=0,fe=0,pv=0,read=0,done=0;
    always #7 wc=~wc;always #2 rc=~rc;
    reg [15:0] id=0,pixel=0;
    wire [7:0] payload;wire [15:0] length,fid;wire first,start;
    wire [31:0] dropped;
    vision_frame_stream #(.WIDTH(W),.ACTIVE_HEIGHT(H),.CANVAS_HEIGHT(C)) dut(
        .wr_clk(wc),.wr_rst_n(rst),.frame_start(fs),.frame_end(fe),.frame_good(1'b1),
        .frame_id(id),.pixel(pixel),.pixel_valid(pv),.rd_clk(rc),.rd_rst_n(rst),
        .payload_read(read),.tx_done(done),.payload_data(payload),.payload_length(length),
        .packet_frame_id(fid),.packet_first(first),.tx_start(start),.dropped_frames(dropped));
    task send_frame(input integer n);
        integer i;
        begin
            @(negedge wc);id=n;fs=1;
            @(negedge wc);fs=0;pv=1;
            for(i=0;i<W*H;i=i+1)begin pixel=n*4096+i*257;@(negedge wc);end
            pv=0;fe=1;@(negedge wc);fe=0;
        end
    endtask
    reg queued=0;
    initial begin
        repeat(4)@(negedge wc);rst=1;
        send_frame(1);send_frame(2);send_frame(3);
        if(dropped!=1)$fatal(1,"full queue must drop exactly one complete frame");
        queued=1;
        wait(dut.rptr==2);repeat(4)@(negedge wc);
        send_frame(4);send_frame(5);
    end
    integer f,n,row,b,expected,value;reg [7:0] held;
    initial begin
        wait(queued);
        for(f=0;f<4;f=f+1)begin
            n=f<2 ? f+1 : f+2;
            for(row=0;row<C;row=row+1)begin
                wait(dut.state==2);@(negedge rc);
                if(fid!=n || length!=16+W*2 || first!=(row==0))$fatal(1,"bank frame metadata");
                for(b=0;b<16+W*2;b=b+1)begin
                    read=0;held=payload;
                    repeat(1+(b%3))begin @(negedge rc);if(payload!==held)$fatal(1,"stall corrupted bank data");end
                    if(b<16)begin
                        case(b)
                            0:expected="C";1:expected="V";2:expected=2;3:expected=1;
                            4:expected=n;6:expected=row;10,12:expected=W;14:expected=C;default:expected=0;
                        endcase
                    end else begin
                        value=(row>=(C-H)/2 && row<(C-H)/2+H) ? n*4096+((row-(C-H)/2)*W+(b-16)/2)*257 : 16'h738e;
                        expected=b[0] ? (value>>8)&255 : value&255;
                    end
                    if(payload!==expected[7:0])$fatal(1,"bank/frame corrupt frame=%0d row=%0d byte=%0d got=%h expected=%h",n,row,b,payload,expected);
                    read=1;@(negedge rc);
                end
                read=0;done=1;@(negedge rc);done=0;
                wait(dut.state!=2);
            end
        end
        if(dropped!=1)$fatal(1,"unexpected overwrite/drop");
        $display("PASS banked frame RAM %0dx%0d: two queued frames, full drop, pointer wrap, slot reuse, stalls",W,H);$finish;
    end
    initial begin #100000;$fatal(1,"bank test timeout");end
endmodule
