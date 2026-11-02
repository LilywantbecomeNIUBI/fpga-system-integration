`timescale 1ns/1ps
// Behavioral slave, independent of DUT state. Receives actual open-drain bus
// traffic and checks address, 14-byte snapshots, repeated START and final master NACK.
module mpu6050_model # (parameter integer SAMPLE_NS=5000000)(
    inout scl, inout sda, input present, input [7:0] identity,
    input corrupt_readback, input invalid_data, input freeze_samples, input reset_stuck
);
    reg drive_low=0;
    assign sda=drive_low ? 1'b0 : 1'bz;
    localparam RX_ADDR=0,RX_REG=1,RX_WRITE=2,TX_READ=3,MASTER_ACK=4,IGNORE=5;
    integer state=IGNORE, next_state=IGNORE, bits=0, pointer=0;
    integer starts=0, stops=0, reads=0, writes=0, burst_reads=0;
    integer master_acks=0, master_nacks=0, burst_length=0;
    reg [7:0] regs[0:127];
    reg [7:0] frozen[0:13];
    reg [7:0] shift=0, tx_byte=0, value;
    reg ack_phase=0, ack_clocked=0, ack_enable=0, master_ack_clocked=0, master_nack=0;
    reg pointer_valid=0;
    time enabled_at=0;
    integer j;
    task defaults;
        integer k;
        begin
            for(k=0;k<128;k=k+1) regs[k]=0;
            regs['h6b]='h40;
            regs['h3b]='h04; regs['h3c]='h00; // ax=1024
            regs['h3d]='hf8; regs['h3e]='h00; // ay=-2048
            regs['h3f]='h20; regs['h40]='h00; // az=8192
            regs['h41]='h01; regs['h42]='h54; // temp=340
            regs['h43]='h80; regs['h44]='h01; // gx=-32767
            regs['h45]='h03; regs['h46]='he8; // gy=1000
            regs['h47]='h7f; regs['h48]='hff; // gz=32767
        end
    endtask
    initial begin defaults(); for(j=0;j<14;j=j+1) frozen[j]=0; end
    always begin
        #(SAMPLE_NS);
        if (!regs['h6b][6] && regs['h38][0] && !freeze_samples) regs['h3a]=regs['h3a]|8'h01;
    end
    function [7:0] get_byte;
        input integer p;
        begin
            if(p=='h75) get_byte=identity;
            else if(p>='h3b && p<='h48) get_byte=frozen[p-'h3b];
            else if(p=='h1b && corrupt_readback) get_byte=regs[p]^8'h10;
            else get_byte=regs[p];
        end
    endfunction
    task load_tx_byte;
        begin
            tx_byte=get_byte(pointer);
            // R/C: snapshot and clear are one register-read event. Clearing
            // later at the master's ACK would discard a NEW DRDY event that
            // arrived while this already-snapshotted byte was shifting out.
            if(pointer=='h3a) regs['h3a]=0;
        end
    endtask
    always @(negedge sda) if(scl===1'b1) begin
        state=RX_ADDR; bits=0; shift=0; ack_phase=0; ack_clocked=0;
        drive_low=0; starts=starts+1;
    end
    always @(posedge sda) if(scl===1'b1) begin
        state=IGNORE; drive_low=0; stops=stops+1; pointer_valid=0;
    end
    always @(posedge scl) begin
        if(ack_phase) ack_clocked=1;
        else if(state==RX_ADDR || state==RX_REG || state==RX_WRITE) begin
            value={shift[6:0],sda}; shift=value;
            if(bits==7) begin
                bits=0; ack_enable=present; ack_phase=1; ack_clocked=0;
                case(state)
                    RX_ADDR: begin
                        ack_enable=present && value[7:1]==7'h68;
                        next_state=ack_enable ? (value[0] ? TX_READ : RX_REG) : IGNORE;
                        if(ack_enable && value[0]) begin
                            if(!pointer_valid) $fatal(1,"read without register repeated START");
                            reads=reads+1; burst_length=0;
                            for(j=0;j<14;j=j+1) frozen[j]=regs['h3b+j];
                            if(invalid_data) begin frozen[0]=8'h80; frozen[1]=0; end
                        end
                    end
                    RX_REG: begin pointer=value; pointer_valid=1; next_state=RX_WRITE; end
                    RX_WRITE: begin
                        next_state=RX_WRITE;
                        if(present) begin
                            writes=writes+1;
                            if(pointer=='h6b && value[7]) begin
                                defaults();
                                if(reset_stuck===1'b1) regs['h6b]=8'h80;
                            end else regs[pointer]=value;
                            pointer=(pointer+1)&127;
                        end
                    end
                endcase
            end else bits=bits+1;
        end else if(state==TX_READ) begin
            if(bits==7) begin state=MASTER_ACK; bits=0; master_ack_clocked=0; end
            else bits=bits+1;
        end else if(state==MASTER_ACK) begin
            master_nack=sda; master_ack_clocked=1; burst_length=burst_length+1;
            if(master_nack) begin
                master_nacks=master_nacks+1;
                if(burst_length>1) burst_reads=burst_reads+1;
            end else master_acks=master_acks+1;
        end
    end
    always @(negedge scl) begin
        if(ack_phase) begin
            if(!ack_clocked) drive_low=ack_enable;
            else begin
                ack_phase=0; drive_low=0; state=next_state; bits=0;
                if(state==TX_READ) begin load_tx_byte(); drive_low=!tx_byte[7]; end
            end
        end else if(state==TX_READ) drive_low=!tx_byte[7-bits];
        else if(state==MASTER_ACK) begin
            drive_low=0;
            if(master_ack_clocked) begin
                if(master_nack) state=IGNORE;
                else begin
                    pointer=(pointer+1)&127; bits=0; state=TX_READ;
                    load_tx_byte(); drive_low=!tx_byte[7];
                end
            end
        end else drive_low=0;
    end
endmodule
