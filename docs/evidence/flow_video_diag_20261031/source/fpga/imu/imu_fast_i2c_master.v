`timescale 1ns/1ps
// Register transactions for the ICM-42688-P: 7-bit address, 8-bit register,
// one-byte write or 1..MAX_READ byte repeated-START read. No FIFO policy here.
// Local ch35/ch47 i2c_control/bit_shift were reviewed; their single-byte API
// and push-pull SCL cannot supply this interface. This engine adds open-drain
// SCL, stretching, bounded bus clear and atomic burst completion.
module imu_fast_i2c_master #(
    parameter integer CLK_HZ=50000000, I2C_HZ=100000,
    parameter integer TIMEOUT_US=10000, MAX_READ=32
)(
    input clk, input rst_n, input req, input read_not_write,
    input [6:0] device_addr, input [7:0] reg_addr, input [7:0] write_data,
    input [5:0] read_length,
    output reg busy, output reg done, output reg [3:0] error,
    output reg [MAX_READ*8-1:0] read_data,
    inout scl, inout sda
);
    // Round UP, never exceed the requested rate. A quarter also satisfies
    // tLOW/2 and tBUF/2 for Standard/Fast/Fast-mode Plus. At 50 MHz:
    // 100k -> Q=125, 400k -> Q=33 (~379k), 1M -> Q=13 (~962k).
    // The original engine truncated the divisor: Q=12 would exceed 1 MHz.
    localparam integer REQUEST_Q=(CLK_HZ+I2C_HZ*4-1)/(I2C_HZ*4);
    localparam integer MIN_HALF_NS=I2C_HZ<=100000 ? 2350 : (I2C_HZ<=400000 ? 650 : 250);
    localparam integer MIN_Q=((CLK_HZ/1000)*MIN_HALF_NS+999999)/1000000;
    localparam integer QUARTER=REQUEST_Q>MIN_Q ? REQUEST_Q : MIN_Q;
    localparam integer LIMIT=(CLK_HZ/1000000)*TIMEOUT_US;
    localparam IDLE=0, FREE=1, START=2, START_HOLD=3, START_END=4,
        SEND=5, SLAVE_ACK=6, RESTART=7, RESTART_HIGH=8,
        RESTART_WAIT=9, RESTART_HOLD=10, RESTART_END=11,
        RECEIVE=12, MASTER_ACK=13, STOP_LOW=14, STOP_HIGH=15,
        STOP_WAIT=16, STOP_HOLD=17, STOP_RELEASE=18, STOP_FREE=19,
        COMPLETE=20, CLEAR=21, START_END_READ=22;
    reg [4:0] state;
    reg scl_low, sda_low;
    assign scl=scl_low ? 1'b0 : 1'bz;
    assign sda=sda_low ? 1'b0 : 1'bz;
    (* syn_preserve=1 *) reg [1:0] scl_sync, sda_sync;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin scl_sync<=3; sda_sync<=3; end
        else begin scl_sync<={scl_sync[0],scl}; sda_sync<={sda_sync[0],sda}; end
    reg [31:0] divider, watchdog;
    reg [2:0] phase, bit_index;
    reg [1:0] stage;
    reg [7:0] tx_byte, rx_byte, saved_reg, saved_write;
    reg [6:0] saved_addr;
    reg saved_read, nack_seen, clearing;
    reg [5:0] length, received;
    reg [3:0] clear_count;
    wire tick=divider==QUARTER-1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; scl_low<=0; sda_low<=0; busy<=0; done<=0;
            error<=0; read_data<=0; divider<=0; watchdog<=0;
            phase<=0; bit_index<=7; stage<=0; tx_byte<=0; rx_byte<=0;
            saved_reg<=0; saved_write<=0; saved_addr<=0; saved_read<=0;
            nack_seen<=0; clearing<=0; length<=0; received<=0; clear_count<=0;
        end else begin
            done<=0;
            if (tick) divider<=0; else divider<=divider+1'b1;
            if (busy) watchdog<=watchdog+1'b1; else watchdog<=0;
            if (busy && watchdog>=LIMIT-1) begin
                scl_low<=0; sda_low<=0; error<=2; busy<=0; done<=1; state<=IDLE;
            end else if (state==IDLE) begin
                scl_low<=0; sda_low<=0;
                if (req) begin
                    saved_addr<=device_addr; saved_reg<=reg_addr; saved_write<=write_data;
                    saved_read<=read_not_write; length<=read_length; received<=0;
                    error<=0; read_data<=0; busy<=1; watchdog<=0; divider<=0;
                    clearing<=0; clear_count<=0; stage<=0; state<=FREE;
                    if (read_not_write && (read_length==0 || read_length>MAX_READ)) begin
                        error<=4; busy<=0; done<=1; state<=IDLE;
                    end
                end
            end else if (tick) begin
                case (state)
                    FREE: if (scl_sync[1]) begin
                        if (sda_sync[1]) state<=START;
                        else begin state<=CLEAR; phase<=0; end
                    end
                    START: begin sda_low<=1; state<=START_HOLD; end
                    START_HOLD: state<=START_END;
                    START_END: begin
                        scl_low<=1; tx_byte<={saved_addr,1'b0}; bit_index<=7;
                        phase<=0; stage<=0; state<=SEND;
                    end
                    SEND: case (phase)
                        0: begin scl_low<=1; sda_low<=!tx_byte[bit_index]; phase<=1; end
                        1: begin scl_low<=0; phase<=2; end
                        2: if (scl_sync[1]) phase<=3;
                        3: begin
                            scl_low<=1; phase<=0;
                            if (bit_index==0) state<=SLAVE_ACK;
                            else bit_index<=bit_index-1'b1;
                        end
                    endcase
                    SLAVE_ACK: case (phase)
                        0: begin sda_low<=0; phase<=1; end
                        1: begin scl_low<=0; phase<=2; end
                        2: if (scl_sync[1]) begin nack_seen<=sda_sync[1]; phase<=3; end
                        3: begin
                            scl_low<=1; phase<=0; bit_index<=7;
                            if (nack_seen) begin error<=1; state<=STOP_LOW; end
                            else case (stage)
                                0: begin stage<=1; tx_byte<=saved_reg; state<=SEND; end
                                1: if (saved_read) state<=RESTART;
                                   else begin stage<=2; tx_byte<=saved_write; state<=SEND; end
                                2: state<=STOP_LOW;
                                3: begin state<=RECEIVE; rx_byte<=0; end
                            endcase
                        end
                    endcase
                    RESTART: begin sda_low<=0; state<=RESTART_HIGH; end
                    RESTART_HIGH: begin scl_low<=0; state<=RESTART_WAIT; end
                    RESTART_WAIT: if (scl_sync[1]) state<=RESTART_HOLD;
                    RESTART_HOLD: begin sda_low<=1; state<=RESTART_END; end
                    RESTART_END: state<=START_END_READ;
                    RECEIVE: case (phase)
                        0: begin sda_low<=0; phase<=1; end
                        1: begin scl_low<=0; phase<=2; end
                        2: if (scl_sync[1]) begin rx_byte<={rx_byte[6:0],sda_sync[1]}; phase<=3; end
                        3: begin
                            scl_low<=1; phase<=0;
                            if (bit_index==0) state<=MASTER_ACK;
                            else bit_index<=bit_index-1'b1;
                        end
                    endcase
                    MASTER_ACK: case (phase)
                        0: begin sda_low<=received!=length-1; phase<=1; end
                        1: begin scl_low<=0; phase<=2; end
                        2: if (scl_sync[1]) phase<=3;
                        3: begin
                            scl_low<=1; phase<=0; bit_index<=7; received<=received+1'b1;
                            read_data<={read_data[MAX_READ*8-9:0],rx_byte};
                            if (received==length-1) state<=STOP_LOW;
                            else state<=RECEIVE;
                        end
                    endcase
                    STOP_LOW: begin scl_low<=1; sda_low<=1; state<=STOP_HIGH; end
                    STOP_HIGH: begin scl_low<=0; state<=STOP_WAIT; end
                    STOP_WAIT: if (scl_sync[1]) state<=STOP_HOLD;
                    STOP_HOLD: state<=STOP_RELEASE;
                    STOP_RELEASE: begin sda_low<=0; state<=STOP_FREE; end
                    STOP_FREE: state<=COMPLETE;
                    COMPLETE: begin
                        if (clearing) begin
                            clearing<=0;
                            if (sda_sync[1]) state<=START;
                            else begin error<=3; done<=1; busy<=0; state<=IDLE; end
                        end else begin busy<=0; done<=1; state<=IDLE; end
                    end
                    CLEAR: case (phase)
                        0: begin scl_low<=1; sda_low<=0; phase<=1; end
                        1: phase<=2;
                        2: begin scl_low<=0; phase<=3; end
                        3: if (scl_sync[1]) phase<=4;
                        4: phase<=5;
                        5: begin
                            scl_low<=1; phase<=0; clear_count<=clear_count+1'b1;
                            if (clear_count==8) begin clearing<=1; state<=STOP_LOW; end
                        end
                    endcase
                    START_END_READ: begin
                        scl_low<=1; tx_byte<={saved_addr,1'b1}; bit_index<=7;
                        stage<=3; phase<=0; state<=SEND;
                    end
                    default: begin error<=3; state<=STOP_LOW; end
                endcase
            end
        end
    end
endmodule
