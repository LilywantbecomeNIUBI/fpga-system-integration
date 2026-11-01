// CRC computation is serialized, with no combinational whole-packet CRC chain.
module frame_crc #(parameter HIGH_FIRST=0)(
    input wire clk,reset_n,capture,input wire [367:0] body_in,
    output reg [383:0] frame,output reg valid
);
    reg [367:0] body;
    reg [15:0] crc;
    reg [5:0] index;
    reg busy;
    function automatic [15:0] update_crc(input [15:0] old,input [7:0] value);
        reg [15:0] c;integer n;
        begin c=old^{value,8'b0};for(n=0;n<8;n=n+1)c=c[15]?(c<<1)^16'h1021:c<<1;update_crc=c;end
    endfunction
    wire [15:0] next_crc=update_crc(crc,body[index*8+:8]);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin body<=0;crc<=16'hffff;index<=0;busy<=0;frame<=0;valid<=0;end
        else begin
            valid<=0;
            if(capture)begin body<=body_in;crc<=16'hffff;index<=0;busy<=1;end
            else if(busy)begin
                crc<=next_crc;
                if(index==45)begin
                    frame<=HIGH_FIRST ? {next_crc[7:0],next_crc[15:8],body} : {next_crc,body};
                    busy<=0;valid<=1;
                end else index<=index+6'd1;
            end
        end
    end
endmodule

// Complete 48-byte packet ownership; one physical UART; newest pending snapshot wins.
module frame_uart_mux #(parameter CLK_HZ=50000000,BAUD=115200,AUX_ENABLED=0)(
    input wire clk,reset_n,motor_valid,imu_valid,
    input wire [383:0] motor_frame,imu_frame,
    input wire aux_valid,input wire [383:0] aux_frame,
    output wire tx,output wire aux_ready,output reg [15:0] aux_drops,output reg [15:0] imu_drops
);
    assign aux_ready=AUX_ENABLED && !have_aux;
    reg [383:0] motor_pending,imu_pending,active;
    reg have_motor,have_imu,have_aux;reg [383:0] aux_pending;
    reg [1:0] state;
    reg [5:0] index;
    reg start;
    reg [7:0] data;
    wire busy;
    uart_tx_byte #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) uart(.clk(clk),.reset_n(reset_n),.start(start),.data(data),.tx(tx),.busy(busy));
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            motor_pending<=0;imu_pending<=0;active<=0;have_motor<=0;have_imu<=0;
            state<=0;index<=0;start<=0;data<=0;imu_drops<=0;aux_drops<=0;have_aux<=0;aux_pending<=0;
        end else begin
            start<=0;
            if(AUX_ENABLED && aux_valid)begin
                aux_pending<=aux_frame;have_aux<=1;
                if(have_aux && !(state==0 && !have_motor && !have_imu) && aux_drops!=16'hffff)aux_drops<=aux_drops+16'd1;
            end
            if(motor_valid)begin motor_pending<=motor_frame;have_motor<=1;end
            if(imu_valid)begin
                imu_pending<=imu_frame;have_imu<=1;
                if(have_imu && !(state==0 && !have_motor) && imu_drops!=16'hffff)imu_drops<=imu_drops+16'd1;
            end
            case(state)
                0: if(have_motor)begin active<=motor_pending;have_motor<=motor_valid;index<=0;state<=1;end
                   else if(have_imu)begin active<=imu_pending;have_imu<=imu_valid;index<=0;state<=1;end
                   else if(AUX_ENABLED && have_aux)begin active<=aux_pending;have_aux<=aux_valid;index<=0;state<=1;end
                1: if(!busy)begin data<=active[index*8+:8];start<=1;state<=2;end
                2: if(busy)state<=3;
                3: if(!busy)begin
                    if(index==47)state<=0;
                    else begin index<=index+6'd1;state<=1;end
                end
            endcase
        end
    end
endmodule
