// Periodic camera status: CAM=HH\r\n, hexadecimal LED bits 7..0, 115200 8N1.
module camera_status_uart #(parameter BAUD_DIV=434, SECOND_CYCLES=50000000, TELEMETRY=0,NETWORK=0)(
    input clk, input rst_n, input [7:0] status, input [31:0] pclk_cycles,input [31:0] network_debug, output tx
);
    reg [7:0] sync0, sync1, snapshot;
    reg [25:0] interval;
    reg active;
    reg [5:0] index;
    reg [31:0] net_sync0,net_sync1,net_snapshot;
    reg [31:0] pclk_snapshot;
    reg [9:0] shift;
    reg [3:0] bits_left;
    reg [15:0] bit_timer;
    function [7:0] hex;
        input [3:0] nibble;
        begin hex=nibble<10 ? 8'h30+nibble : 8'h41+nibble-10; end
    endfunction
    function [7:0] character;
        input [5:0] offset;
        input [7:0] flags;
        input [31:0] frequency,network_word;
        begin
            case(offset)
                0:character="C"; 1:character="A"; 2:character="M";
                3:character="="; 4:character=hex(flags[7:4]);
                5:character=hex(flags[3:0]);
                6:character=TELEMETRY ? " " : 8'h0d;
                7:character=TELEMETRY ? "P" : 8'h0a;
                8:character="C";9:character="L";10:character="K";11:character="=";
                12:character=hex(frequency[31:28]);13:character=hex(frequency[27:24]);
                14:character=hex(frequency[23:20]);15:character=hex(frequency[19:16]);
                16:character=hex(frequency[15:12]);17:character=hex(frequency[11:8]);
                18:character=hex(frequency[7:4]);19:character=hex(frequency[3:0]);
                20:character=NETWORK ? " " : 8'h0d;
                21:character=NETWORK ? "R" : 8'h0a;22:character="X";23:character="=";
                24,25,26,27,28,29,30,31:character=hex(network_word[31-(offset-24)*4 -: 4]);
                32:character=8'h0d;
                default:character=8'h0a;
            endcase
        end
    endfunction
    assign tx=bits_left==0 ? 1'b1 : shift[0];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sync0<=0;sync1<=0;snapshot<=0;interval<=0;active<=0;index<=0;
            shift<=10'h3ff;bits_left<=0;bit_timer<=0;pclk_snapshot<=0;
            net_sync0<=0;net_sync1<=0;net_snapshot<=0;
        end else begin
            sync0<=status;sync1<=sync0;
            net_sync0<=network_debug;net_sync1<=net_sync0;
            if(interval==SECOND_CYCLES-1) begin
                interval<=0;snapshot<=sync1;pclk_snapshot<=pclk_cycles;net_snapshot<=net_sync1;active<=1;index<=0;
            end else interval<=interval+1'b1;
            if(bits_left!=0) begin
                if(bit_timer!=0) bit_timer<=bit_timer-1'b1;
                else begin shift<={1'b1,shift[9:1]};bits_left<=bits_left-1'b1;bit_timer<=BAUD_DIV-1;end
            end else if(active) begin
                shift<={1'b1,character(index,snapshot,pclk_snapshot,net_snapshot),1'b0};bits_left<=10;bit_timer<=BAUD_DIV-1;
                if(index==(NETWORK ? 33 : TELEMETRY ? 21 : 7)) active<=0;else index<=index+1'b1;
            end
        end
    end
endmodule

// Divide PCLK by 256 before synchronizing one bit. No multi-bit asynchronous
// counter is sampled. Resolution is 256 Hz over the board's 50 MHz reference.
module camera_clock_meter #(parameter SECOND_CYCLES=50000000)(
    input pclk, input clk, input rst_n, output reg [31:0] cycles
);
    reg [7:0] divider;
    reg sync0,sync1,previous;
    reg [25:0] interval;
    reg [23:0] edges;
    always @(posedge pclk or negedge rst_n)
        if(!rst_n)divider<=0;else divider<=divider+1'b1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin sync0<=0;sync1<=0;previous<=0;interval<=0;edges<=0;cycles<=0;end
        else begin
            sync0<=divider[7];sync1<=sync0;previous<=sync1;
            if(interval==SECOND_CYCLES-1)begin
                cycles<={edges,8'b0};interval<=0;edges<=sync1 && !previous ? 1 : 0;
            end else begin
                interval<=interval+1'b1;
                if(sync1 && !previous)edges<=edges+1'b1;
            end
        end
    end
endmodule
