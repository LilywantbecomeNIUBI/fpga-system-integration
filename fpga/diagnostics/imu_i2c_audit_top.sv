// Temporary, read-only WHO_AM_I probe; all motor pads are constant low.
module imu_i2c_audit_top #(
    parameter integer CLK_HZ=50000000, BAUD=115200, PWM_LIMIT=350, REPORT_HZ=10
)(
    input wire clk50m,reset_n,lidar_rx,
    inout wire imu_i2c_scl,imu_i2c_sda,
    input wire left_enc_a,left_enc_b,right_enc_a,right_enc_b,
    output wire left_in1,left_in2,right_in1,right_in2,uart_tx,
    output wire legacy_m1_disable,
    output wire [5:0] legacy_m1_pwm_disabled,
    output wire [0:0] legacy_cal_disabled,
    output wire [7:0] led
);

    assign {left_in1,left_in2,right_in1,right_in2,legacy_m1_disable,
        legacy_m1_pwm_disabled,legacy_cal_disabled}=12'd0;
    reg [31:0] timer,sequence_id;
    reg [2:0] state;
    reg address_sel,request,capture,valid;
    reg [367:0] body;
    reg [383:0] pending;
    wire [6:0] address=address_sel ? 7'h69 : 7'h68;
    wire busy,done,crc_valid,aux_ready;
    wire [3:0] error;
    wire [255:0] read_data;
    wire [383:0] checked;
    reg [1:0] idle_levels;
    reg [1:0] scl_sync,sda_sync;
    always @(posedge clk50m or negedge reset_n)
        if(!reset_n)begin scl_sync<=3;sda_sync<=3;end
        else begin scl_sync<={scl_sync[0],imu_i2c_scl};sda_sync<={sda_sync[0],imu_i2c_sda};end
    assign led={address_sel,state,sequence_id[3:0]};
    wire [2:0] ack_mask;wire [7:0] failed_stage;wire [1:0] line_fault;wire [15:0] scl_rises;
    imu_whoami_i2c #(.CLK_HZ(CLK_HZ)) bus(.clk(clk50m),.rst_n(reset_n),.request(request),
        .device_addr(address),.busy(busy),.done(done),.error(error),.who_am_i(read_data[7:0]),
        .ack_mask(ack_mask),.failed_stage(failed_stage),.line_fault(line_fault),.scl_rises(scl_rises),
        .scl(imu_i2c_scl),.sda(imu_i2c_sda));
    frame_crc crc(.clk(clk50m),.reset_n(reset_n),.capture(capture),.body_in(body),
        .frame(checked),.valid(crc_valid));
    frame_uart_mux #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.AUX_ENABLED(1)) sender(
        .clk(clk50m),.reset_n(reset_n),.motor_valid(1'b0),.motor_frame(384'd0),
        .imu_valid(1'b0),.imu_frame(384'd0),.aux_valid(valid && aux_ready),
        .aux_frame(pending),.aux_ready(aux_ready),.tx(uart_tx));
    always @(posedge clk50m or negedge reset_n)begin
        if(!reset_n)begin
            timer<=0;sequence_id<=0;state<=0;address_sel<=0;
            request<=0;capture<=0;valid<=0;body<=0;pending<=0;idle_levels<=0;
        end else begin
            request<=0;capture<=0;
            case(state)
                0:if(timer==CLK_HZ/REPORT_HZ-1)begin
                    timer<=0;idle_levels<={sda_sync[1],scl_sync[1]};
                    request<=1;state<=1;
                end else timer<=timer+1'b1;
                1:if(done)begin
                    body<=0;body[31:0]<=32'h01073cc3;
                    body[39:32]<={1'b0,address};body[47:40]<={4'd0,error};
                    body[55:48]<=read_data[7:0];body[63:56]<={6'd0,idle_levels};
                    body[95:64]<=sequence_id;body[127:96]<=32'd100000;
                    body[135:128]<={5'd0,ack_mask};body[143:136]<=failed_stage;
                    body[151:144]<={6'd0,line_fault};body[175:160]<=scl_rises;
                    capture<=1;state<=2;
                end
                2:if(crc_valid)begin pending<=checked;valid<=1;state<=3;end
                3:if(valid && aux_ready)begin
                    valid<=0;address_sel<=!address_sel;
                    sequence_id<=sequence_id+1'b1;state<=0;
                end
                default:state<=0;
            endcase
        end
    end
endmodule
