`timescale 1ns/1ps
// Independent read-only I2C transaction, matching the STM32 sequence:
// START, addr/W, 0x75, repeated START, addr/R, one byte, NACK, STOP.
// Uses a 1 us timing base instead of imu_fast_i2c_master's quarter-bit FSM.
// SCL is always open drain. Sample two us AFTER the observed SCL high.
module imu_whoami_i2c #(parameter integer CLK_HZ=50000000)(
    input clk,rst_n,request,input [6:0] device_addr,
    output reg busy,done,output reg [3:0] error,
    output reg [7:0] who_am_i,output reg [2:0] ack_mask,
    output reg [7:0] failed_stage,output reg [1:0] line_fault,
    output reg [15:0] scl_rises,inout scl,sda
);
    localparam integer US_DIV=CLK_HZ/1000000;
    localparam [3:0] IDLE=0,FREE=1,START_HOLD=2,PREP=3,LOW_WAIT=4,
        WAIT_HIGH=5,SAMPLE=6,HIGH_WAIT=7,REP_LOW=8,REP_RISE=9,
        REP_SETUP=10,REP_HOLD=11,STOP_PREP=12,STOP_RISE=13,
        STOP_SETUP=14,STOP_FREE=15;
    localparam [1:0] TX=0,ACK=1,RX=2,NACK=3;
    reg [3:0] state;
    reg [1:0] kind,byte_index;
    reg [2:0] bit_index;
    reg [7:0] tx_byte;
    reg [6:0] saved_addr;
    reg scl_low,sda_low,ack_sample,previous_scl;
    reg [3:0] delay_us;
    reg [31:0] divider,watchdog;
    (* syn_preserve=1 *) reg [1:0] scl_sync,sda_sync;
    assign scl=scl_low ? 1'b0 : 1'bz;
    assign sda=sda_low ? 1'b0 : 1'bz;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin scl_sync<=3;sda_sync<=3;end
        else begin scl_sync<={scl_sync[0],scl};sda_sync<={sda_sync[0],sda};end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;kind<=TX;byte_index<=0;bit_index<=7;tx_byte<=0;saved_addr<=0;
            scl_low<=0;sda_low<=0;ack_sample<=1;previous_scl<=1;delay_us<=0;
            divider<=0;watchdog<=0;busy<=0;done<=0;error<=0;who_am_i<=0;
            ack_mask<=0;failed_stage<=8'hff;line_fault<=0;scl_rises<=0;
        end else begin
            done<=0;previous_scl<=scl_sync[1];
            if(divider==US_DIV-1)divider<=0;else divider<=divider+1'b1;
            if(busy)begin
                watchdog<=watchdog+1'b1;
                if(!previous_scl && scl_sync[1])scl_rises<=scl_rises+1'b1;
            end
            if(busy && watchdog>=CLK_HZ/50-1)begin
                scl_low<=0;sda_low<=0;busy<=0;done<=1;error<=2;
                failed_stage<={6'd0,byte_index};state<=IDLE;
            end else if(state==IDLE)begin
                scl_low<=0;sda_low<=0;
                if(request)begin
                    saved_addr<=device_addr;tx_byte<={device_addr,1'b0};
                    byte_index<=0;bit_index<=7;kind<=TX;ack_mask<=0;error<=0;
                    who_am_i<=0;failed_stage<=8'hff;line_fault<=0;scl_rises<=0;
                    busy<=1;watchdog<=0;divider<=0;delay_us<=9;state<=FREE;
                end
            end else if(divider==US_DIV-1)begin
                case(state)
                    FREE:if(scl_sync[1] && sda_sync[1])begin
                        if(delay_us==0)begin sda_low<=1;delay_us<=4;state<=START_HOLD;end
                        else delay_us<=delay_us-1'b1;
                    end else delay_us<=9;
                    START_HOLD:if(delay_us==0)begin scl_low<=1;state<=PREP;end
                        else delay_us<=delay_us-1'b1;
                    PREP:begin
                        // Change data only after the physical falling edge.
                        sda_low<=kind==TX ? !tx_byte[bit_index] : 1'b0;
                        delay_us<=3;state<=LOW_WAIT;
                    end
                    LOW_WAIT:if(delay_us==0)begin
                        if(scl_sync[1])line_fault[0]<=1;
                        if(sda_low && sda_sync[1])line_fault[1]<=1;
                        scl_low<=0;state<=WAIT_HIGH;
                    end else delay_us<=delay_us-1'b1;
                    WAIT_HIGH:if(scl_sync[1])begin delay_us<=1;state<=SAMPLE;end
                    SAMPLE:if(delay_us==0)begin
                        if(kind==ACK)ack_sample<=sda_sync[1];
                        if(kind==RX)who_am_i<={who_am_i[6:0],sda_sync[1]};
                        delay_us<=2;state<=HIGH_WAIT;
                    end else delay_us<=delay_us-1'b1;
                    HIGH_WAIT:if(delay_us==0)begin
                        scl_low<=1;state<=PREP;
                        case(kind)
                            TX:if(bit_index==0)kind<=ACK;else bit_index<=bit_index-1'b1;
                            ACK:if(ack_sample)begin
                                error<=1;failed_stage<={6'd0,byte_index};state<=STOP_PREP;
                            end else begin
                                ack_mask[byte_index]<=1;
                                case(byte_index)
                                    0:begin byte_index<=1;tx_byte<=8'h75;bit_index<=7;kind<=TX;end
                                    1:begin byte_index<=2;delay_us<=4;state<=REP_LOW;end
                                    2:begin bit_index<=7;kind<=RX;end
                                endcase
                            end
                            RX:if(bit_index==0)kind<=NACK;else bit_index<=bit_index-1'b1;
                            NACK:state<=STOP_PREP;
                        endcase
                    end else delay_us<=delay_us-1'b1;
                    REP_LOW:begin
                        sda_low<=0;
                        if(delay_us==0)begin scl_low<=0;state<=REP_RISE;end
                        else delay_us<=delay_us-1'b1;
                    end
                    REP_RISE:if(scl_sync[1])begin delay_us<=4;state<=REP_SETUP;end
                    REP_SETUP:if(delay_us==0)begin
                        sda_low<=1;delay_us<=4;state<=REP_HOLD;
                    end else delay_us<=delay_us-1'b1;
                    REP_HOLD:if(delay_us==0)begin
                        scl_low<=1;tx_byte<={saved_addr,1'b1};bit_index<=7;kind<=TX;state<=PREP;
                    end else delay_us<=delay_us-1'b1;
                    STOP_PREP:begin sda_low<=1;delay_us<=4;state<=STOP_RISE;end
                    STOP_RISE:if(delay_us==0)begin scl_low<=0;state<=STOP_SETUP;delay_us<=4;end
                        else delay_us<=delay_us-1'b1;
                    STOP_SETUP:if(scl_sync[1])begin
                        if(delay_us==0)begin sda_low<=0;delay_us<=9;state<=STOP_FREE;end
                        else delay_us<=delay_us-1'b1;
                    end else delay_us<=4;
                    STOP_FREE:if(delay_us==0)begin busy<=0;done<=1;state<=IDLE;end
                        else delay_us<=delay_us-1'b1;
                endcase
            end
        end
    end
endmodule
