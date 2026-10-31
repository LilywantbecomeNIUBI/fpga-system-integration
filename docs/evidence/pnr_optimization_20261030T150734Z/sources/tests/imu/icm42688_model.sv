`timescale 1ns/1ps
// Behavioral slave, independent of DUT state. Receives actual open-drain bus
// traffic and checks address, bank access, repeated START and final master NACK.
module icm42688_model # (parameter integer SAMPLE_NS=5000000)(
    inout scl, inout sda, input present, input [7:0] identity,
    input corrupt_readback, input invalid_data, input freeze_samples
);
    reg drive_low=0;
    assign sda=drive_low ? 1'b0 : 1'bz;
    localparam RX_ADDR=0,RX_REG=1,RX_WRITE=2,TX_READ=3,MASTER_ACK=4,IGNORE=5;
    integer state=IGNORE, next_state=IGNORE, bits=0, bank=0, pointer=0;
    integer starts=0, stops=0, reads=0, writes=0, burst_reads=0;
    integer master_acks=0, master_nacks=0, burst_length=0;
    reg [7:0] regs[0:639];
    reg [7:0] frozen[0:13];
    reg [7:0] shift=0, tx_byte=0, value;
    reg ack_phase=0, ack_clocked=0, ack_enable=0, master_ack_clocked=0, master_nack=0;
    reg pointer_valid=0;
    time enabled_at=0;
    integer j;
    task defaults;
        integer k;
        begin
            for(k=0;k<640;k=k+1) regs[k]=0;
            bank=0; regs['h13]='h05; regs['h4c]='h30; regs['h4d]='h91;
            regs['h51]='h16; regs['h53]='h0d; regs['h4f]='h06; regs['h50]='h06; regs['h52]='h11;
            regs['h2d]='h10; regs[128+'h7a]='h02; regs[128+'h7c]='h5f;
            regs['h1d]='h00; regs['h1e]='h84;
            regs['h1f]='h04; regs['h20]='h00; // ax=1024
            regs['h21]='hf8; regs['h22]='h00; // ay=-2048
            regs['h23]='h20; regs['h24]='h00; // az=8192
            regs['h25]='h80; regs['h26]='h01; // gx=-32767
            regs['h27]='h03; regs['h28]='he8; // gy=1000
            regs['h29]='h7f; regs['h2a]='hff; // gz=32767
        end
    endtask
    initial begin defaults(); for(j=0;j<14;j=j+1) frozen[j]=0; end
    always begin
        #(SAMPLE_NS);
        if ((regs['h4e]&8'h0f)==8'h0f && !freeze_samples) regs['h2d]=regs['h2d]|8'h08;
    end
    function [7:0] get_byte;
        input integer p;
        begin
            if(p=='h76) get_byte=bank;
            else if(bank==0 && p=='h75) get_byte=identity;
            else if(bank==0 && p>='h1d && p<='h2a) get_byte=frozen[p-'h1d];
            else if(bank==0 && p=='h4f && corrupt_readback) get_byte=regs[p]^8'h20;
            else get_byte=regs[bank*128+p];
        end
    endfunction
    task load_tx_byte;
        begin
            tx_byte=get_byte(pointer);
            // R/C: snapshot and clear are one register-read event. Clearing
            // later at the master's ACK would discard a NEW DRDY event that
            // arrived while this already-snapshotted byte was shifting out.
            if(bank==0 && pointer=='h2d) regs['h2d]=0;
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
                        ack_enable=present && value[7:1]==7'h69;
                        next_state=ack_enable ? (value[0] ? TX_READ : RX_REG) : IGNORE;
                        if(ack_enable && value[0]) begin
                            if(!pointer_valid) $fatal(1,"read without register repeated START");
                            reads=reads+1; burst_length=0;
                            for(j=0;j<14;j=j+1) frozen[j]=regs['h1d+j];
                            if(invalid_data) begin frozen[2]=8'h80; frozen[3]=0; end
                        end
                    end
                    RX_REG: begin pointer=value; pointer_valid=1; next_state=RX_WRITE; end
                    RX_WRITE: begin
                        next_state=RX_WRITE;
                        if(present) begin
                            writes=writes+1;
                            if(enabled_at!=0 && $time-enabled_at<200000) $fatal(1,"write in 200us startup exclusion");
                            if(pointer=='h76) bank=value&7;
                            else if(bank==0 && pointer=='h11 && value[0]) defaults();
                            else begin
                                regs[bank*128+pointer]=value;
                                if(bank==0 && pointer=='h4e && value[3:0]==4'hf) enabled_at=$time;
                            end
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
