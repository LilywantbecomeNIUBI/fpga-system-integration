`timescale 1ns/1ps
// Independent 200 Hz debug path; 48-byte v2 records, 115200 8N1.
// Raw acquisition never waits for this module. Selected-frame drops and
// deliberate decimation have distinct counters/sequence numbers.
// Payload byte order is little endian; CRC is high byte first. See README.md.
module imu_fast_uart_stream #(
    parameter integer CLK_HZ=50000000, BAUD=115200, STATUS_US=1000000,
    parameter integer DECIMATION=5, POLL_US=150, FAST_BUS=1, LOW_LATENCY_FILTER=1
)(
    input clk, input rst_n, input sample_valid, input ready, input fault,
    input signed [15:0] ax, ay, az, gx, gy, gz, temperature_raw,
    input [31:0] sample_seq, timestamp_us, now_us,
    input [7:0] who_am_i, last_error,
    input [15:0] error_count, invalid_count,
    input sample_stale, temperature_valid,
    input [15:0] observe_to_commit_us, sample_interval_us,
    output tx, output reg [15:0] drop_count,
    output reg [31:0] debug_seq
);
    localparam integer BAUD_DIV=(CLK_HZ+BAUD/2)/BAUD;
    localparam integer STATUS_CYCLES=(CLK_HZ/1000000)*STATUS_US;
    localparam IDLE=0, CRC=1, SEND=2, WAIT_BYTE=3;
    reg [1:0] state;
    reg [367:0] body;
    reg [15:0] crc;
    reg [5:0] index;
    reg [31:0] silence;
    reg [15:0] decimation_count;
    wire selected=sample_valid && decimation_count==DECIMATION-1;
    wire [7:0] flags={2'b0,(LOW_LATENCY_FILTER!=0),(FAST_BUS!=0),temperature_valid,sample_stale,fault,ready};
    wire [7:0] profile_id=LOW_LATENCY_FILTER ? 8'h02 : 8'h03;
    localparam [7:0] DECIMATION_BYTE=DECIMATION, POLL_BYTE=POLL_US;
    reg [9:0] shift;
    reg [3:0] bits_left;
    reg [31:0] baud_counter;
    wire tx_ready=bits_left==0;
    wire [7:0] next_byte=index<46 ? body[index*8 +: 8] : (index==46 ? crc[15:8] : crc[7:0]);
    assign tx=tx_ready ? 1'b1 : shift[0];
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] c;
        integer n;
        begin
            c=previous^{value,8'h00};
            for(n=0;n<8;n=n+1) c=c[15] ? (c<<1)^16'h1021 : c<<1;
            crc_byte=c;
        end
    endfunction
    // UART bit framing adapted from ../src/uart_selftest.v (validated 8N1
    // divider/shift convention); payload generation and CRC are independent.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin shift<=10'h3ff; bits_left<=0; baud_counter<=0; end
        else if (bits_left!=0) begin
            if (baud_counter!=0) baud_counter<=baud_counter-1'b1;
            else begin shift<={1'b1,shift[9:1]}; bits_left<=bits_left-1'b1; baud_counter<=BAUD_DIV-1; end
        end else if (state==SEND) begin
            shift<={1'b1,next_byte,1'b0}; bits_left<=10; baud_counter<=BAUD_DIV-1;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; body<=0; crc<=16'hffff; index<=0; silence<=0; drop_count<=0;
            decimation_count<=0; debug_seq<=0;
        end
        else begin
            if (silence<STATUS_CYCLES) silence<=silence+1'b1;
            if (sample_valid) begin
                if (selected) begin decimation_count<=0; debug_seq<=debug_seq+1'b1; end
                else decimation_count<=decimation_count+1'b1;
            end
            if (selected && state!=IDLE && drop_count!=16'hffff) drop_count<=drop_count+1'b1;
            case(state)
                IDLE: if (selected || silence>=STATUS_CYCLES) begin
                    body<={POLL_BYTE,DECIMATION_BYTE,sample_interval_us,observe_to_commit_us,
                        (selected ? debug_seq+32'd1 : debug_seq),drop_count,invalid_count,error_count,
                        profile_id,last_error,who_am_i,flags,temperature_raw,gz,gy,gx,az,ay,ax,
                        (selected ? timestamp_us : now_us),sample_seq,
                        (selected ? 8'h01 : 8'h02),8'h02,16'h5aa5};
                    index<=0; crc<=16'hffff; silence<=0; state<=CRC;
                end
                CRC: begin
                    crc<=crc_byte(crc,body[index*8 +:8]);
                    if (index==45) begin index<=0; state<=SEND; end
                    else index<=index+1'b1;
                end
                SEND: if (tx_ready) state<=WAIT_BYTE;
                WAIT_BYTE: if (tx_ready) begin
                    if (index==47) state<=IDLE;
                    else begin index<=index+1'b1; state<=SEND; end
                end
            endcase
        end
    end
endmodule
