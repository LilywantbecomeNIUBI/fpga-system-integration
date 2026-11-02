// OV5640 DVP/RGB565 bring-up. Register contents and SCCB engine: ACG720 ch44-47.
// Probe 0x300a/0x300b before configuring; NACK never advances the table.
module ov5640_config #(
    parameter SYS_CLOCK = 50_000_000,
    parameter SCL_CLOCK = 100_000,
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter IMAGE_FLIP_EN = 0, parameter IMAGE_MIRROR_EN = 0,
    parameter OUTPUT_JPEG = 0, parameter FRAME_RATE = 30,
    parameter RETRIES = 3
)(
    input clk, input rst_n,
    output camera_rst_n, output camera_pwdn,
    output sclk, inout sdat,
    output reg id_ok, output reg done, output reg error
);
    localparam WAIT_POWER=0, ISSUE=1, WAIT_BUS=2, NEXT=3, FINISHED=4, FAILED=5;
    localparam POWER_DELAY = SYS_CLOCK / 1000 * 21;
    localparam RESET_DELAY = SYS_CLOCK / 1000;
    reg [2:0] state;
    reg [31:0] timer;
    reg [7:0] index;
    reg [1:0] probe;
    reg [7:0] attempts;
    reg wr_req, rd_req;
    wire [23:0] lut;
    wire [7:0] rddata;
    wire bus_done, nack;
    wire [15:0] address = probe == 0 ? 16'h300a :
                          probe == 1 ? 16'h300b : lut[23:8];
    wire [31:0] bus_delay = probe == 2 && index == 1 ? SYS_CLOCK / 200 : 0;
    assign camera_rst_n = rst_n && timer >= RESET_DELAY;
    assign camera_pwdn = 1'b0;

    ov5640_init_table_rgb #(.IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .IMAGE_FLIP_EN(IMAGE_FLIP_EN), .IMAGE_MIRROR_EN(IMAGE_MIRROR_EN),
        .OUTPUT_JPEG(OUTPUT_JPEG),.FRAME_RATE(FRAME_RATE))
        table_rgb(.clk(clk), .addr(index), .q(lut));
    i2c_control #(.SYS_CLOCK(SYS_CLOCK), .SCL_CLOCK(SCL_CLOCK)) bus (
        .Clk(clk), .Rst_n(rst_n), .wrreg_req(wr_req), .rdreg_req(rd_req),
        .addr(address), .addr_mode(1'b1), .wrdata(lut[7:0]), .rddata(rddata),
        .device_id(8'h78), .RW_Done(bus_done), .ack(nack),
        .dly_cnt_max(bus_delay), .i2c_sclk(sclk), .i2c_sdat(sdat)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= WAIT_POWER; timer <= 0; index <= 0; probe <= 0;
            attempts <= 0; wr_req <= 0; rd_req <= 0;
            id_ok <= 0; done <= 0; error <= 0;
        end else begin
            wr_req <= 0; rd_req <= 0;
            case (state)
                WAIT_POWER: if (timer < POWER_DELAY) timer <= timer + 1'b1;
                            else state <= ISSUE;
                ISSUE: begin
                    wr_req <= probe == 2;
                    rd_req <= probe != 2;
                    state <= WAIT_BUS;
                end
                WAIT_BUS: if (bus_done) begin
                    if (nack) begin
                        if (attempts == RETRIES-1) state <= FAILED;
                        else begin attempts <= attempts + 1'b1; state <= ISSUE; end
                    end else if (probe == 0 && rddata != 8'h56) state <= FAILED;
                    else if (probe == 1 && rddata != 8'h40) state <= FAILED;
                    else begin attempts <= 0; state <= NEXT; end
                end
                NEXT: begin
                    if (probe < 2) begin
                        probe <= probe + 1'b1;
                        if (probe == 1) id_ok <= 1;
                        state <= ISSUE;
                    end else if (index == 252) begin done <= 1; state <= FINISHED; end
                    else begin index <= index + 1'b1; state <= ISSUE; end
                end
                FINISHED: state <= FINISHED;
                FAILED: begin error <= 1; state <= FAILED; end
                default: state <= FAILED;
            endcase
        end
    end
endmodule
