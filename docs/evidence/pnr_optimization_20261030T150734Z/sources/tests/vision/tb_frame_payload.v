`timescale 1ns/1ps
// Verify actual dual-clock frame storage, padding/header bytes and arbitrary
// consumer stalls. This is the source feeding the production UDP payload mux.
module tb_frame_payload;
    reg wr_clk=0,rd_clk=0,rst=0,fs=0,fe=0,pv=0,read=0,done=0;
    always #6 wr_clk=~wr_clk;always #4 rd_clk=~rd_clk;
    reg [15:0] pixel=0;
    wire [7:0] payload;wire [15:0] length,fid;wire first,start;
    vision_frame_stream #(.WIDTH(16),.ACTIVE_HEIGHT(8),.CANVAS_HEIGHT(16)) dut(
        .wr_clk(wr_clk),.wr_rst_n(rst),.frame_start(fs),.frame_end(fe),.frame_good(1'b1),
        .frame_id(16'd42),.pixel(pixel),.pixel_valid(pv),.rd_clk(rd_clk),.rd_rst_n(rst),
        .payload_read(read),.tx_done(done),.payload_data(payload),.payload_length(length),
        .packet_frame_id(fid),.packet_first(first),.tx_start(start),.dropped_frames());
    integer k,row,col,word_value,expected;reg [7:0] held;
    initial begin
        repeat(4)@(negedge wr_clk);rst=1;fs=1;@(negedge wr_clk);fs=0;
        for(k=0;k<128;k=k+1)begin pv=1;pixel=(k*257)^16'h5a3c;@(negedge wr_clk);end
        pv=0;fe=1;@(negedge wr_clk);fe=0;
    end
    initial begin
        for(row=0;row<16;row=row+1)begin
            wait(start);@(negedge rd_clk);
            if(length!=48 || fid!=42 || first!=(row==0))$fatal(1,"frame metadata");
            for(col=0;col<48;col=col+1)begin
                if(col%5==0)begin
                    read=0;held=payload;
                    repeat(1+col%7)begin
                        @(negedge rd_clk);
                        if(payload!==held)$fatal(1,"payload changed under stall");
                    end
                end
                if(col<16)begin
                    case(col)
                        0:expected="C";1:expected="V";2:expected=2;3:expected=1;
                        4:expected=42;6:expected=row;10,12,14:expected=16;default:expected=0;
                    endcase
                end else begin
                    word_value=(row>=4 && row<12) ? (((row-4)*16+(col-16)/2)*257)^16'h5a3c : 16'h738e;
                    expected=col[0] ? (word_value>>8)&255 : word_value&255;
                end
                if(payload!==expected[7:0])$fatal(1,"pixel/header mismatch row=%0d byte=%0d got=%h expected=%h",row,col,payload,expected);
                read=1;@(negedge rd_clk);
            end
            read=0;done=1;@(negedge rd_clk);done=0;
        end
        $display("PASS actual frame payload: 16 rows, headers, RGB565 bytes, letterbox and consumer stalls");$finish;
    end
    initial begin #1000000;$fatal(1,"frame payload timeout");end
endmodule
