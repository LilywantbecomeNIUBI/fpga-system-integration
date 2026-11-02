`timescale 1ns/1ps
// Two independent controllers see identical delayed pin-level slave models.
module tb_i2c_controller_compare;
    reg clk=0,rst_n=0,request=0,stuck=0;
    reg [6:0] address=7'h68;
    reg [7:0] identity=8'h70;
    reg [1:0] nack_at=0;
    always #10 clk=~clk;
    tri1 scl_a,sda_a,scl_b,sda_b;
    reg hold_a=0,hold_b=0;
    assign scl_a=(stuck || hold_a) ? 1'b0 : 1'bz;
    assign scl_b=(stuck || hold_b) ? 1'b0 : 1'bz;
    // A low clamp lasting 700 ns after release models a slow SCL rising edge.
    always @(posedge legacy.scl_low)hold_a=1;
    always @(negedge legacy.scl_low)begin #700;hold_a=0;end
    always @(posedge independent.scl_low)hold_b=1;
    always @(negedge independent.scl_low)begin #700;hold_b=0;end
    wire busy_a,done_a,busy_b,done_b;
    wire [3:0] error_a,error_b;
    wire [255:0] data_a;
    wire [7:0] data_b,failed_stage;
    wire [2:0] ack_mask;
    wire [1:0] line_fault;
    imu_fast_i2c_master #(.I2C_HZ(100000),.TIMEOUT_US(5000)) legacy(
        .clk(clk),.rst_n(rst_n),.req(request),.read_not_write(1'b1),.device_addr(address),
        .reg_addr(8'h75),.write_data(8'd0),.read_length(6'd1),.busy(busy_a),.done(done_a),
        .error(error_a),.read_data(data_a),.scl(scl_a),.sda(sda_a));
    imu_whoami_i2c independent(.clk(clk),.rst_n(rst_n),.request(request),.device_addr(address),
        .busy(busy_b),.done(done_b),.error(error_b),.who_am_i(data_b),.ack_mask(ack_mask),
        .failed_stage(failed_stage),.line_fault(line_fault),.scl_rises(),.scl(scl_b),.sda(sda_b));
    mpu_i2c_delayed_model a(.scl(scl_a),.sda(sda_a),.present(1'b1),.identity(identity),
        .nack_at(nack_at),.corrupt_readback(1'b0),.invalid_data(1'b0),
        .freeze_samples(1'b0),.reset_stuck(1'b0));
    mpu_i2c_delayed_model b(.scl(scl_b),.sda(sda_b),.present(1'b1),.identity(identity),
        .nack_at(nack_at),.corrupt_readback(1'b0),.invalid_data(1'b0),
        .freeze_samples(1'b0),.reset_stuck(1'b0));
    reg got_a=0,got_b=0;
    always @(posedge clk)begin if(done_a)got_a<=1;if(done_b)got_b<=1;end
    integer starts_a,starts_b,stops_a,stops_b;
    task run;begin
        @(negedge clk);got_a=0;got_b=0;
        starts_a=a.starts;starts_b=b.starts;stops_a=a.stops;stops_b=b.stops;
        request=1;@(negedge clk);request=0;wait(got_a && got_b);@(negedge clk);
    end endtask
    task success;begin
        run();
        if(error_a || error_b || data_a[7:0]!==identity || data_b!==identity ||
           ack_mask!=7 || failed_stage!=8'hff || line_fault)
            $fatal(1,"delayed slave mismatch old=%h/%h new=%h/%h ack=%b fault=%b",
                error_a,data_a[7:0],error_b,data_b,ack_mask,line_fault);
        if(a.starts-starts_a!=2 || b.starts-starts_b!=2 ||
           a.stops-stops_a!=1 || b.stops-stops_b!=1 || a.writes || b.writes)
            $fatal(1,"transaction mismatch starts=%0d/%0d stops=%0d/%0d writes=%0d/%0d",
                a.starts-starts_a,b.starts-starts_b,a.stops-stops_a,b.stops-stops_b,a.writes,b.writes);
    end endtask
    initial begin
        repeat(10)@(negedge clk);rst_n=1;#10000;
        success();identity=8'h68;success();
        address=7'h69;run();
        if(error_a!=1 || error_b!=1 || failed_stage!=0 || ack_mask!=0)
            $fatal(1,"write address NACK attribution");
        address=7'h68;nack_at=1;run();
        if(error_a!=1 || error_b!=1 || failed_stage!=1 || ack_mask!=1)
            $fatal(1,"register pointer NACK attribution");
        nack_at=2;run();
        if(error_a!=1 || error_b!=1 || failed_stage!=2 || ack_mask!=3)
            $fatal(1,"read address NACK attribution");
        nack_at=0;success();
        stuck=1;run();
        if(error_a!=2 || error_b!=2 || busy_a || busy_b)$fatal(1,"stuck SCL timeout");
        stuck=0;success();
        $display("PASS both I2C controllers: 700ns SCL rise, 3us slave data/ACK, D0/75/Sr/D1, three NACK stages, timeout/recovery");
        $finish;
    end
    initial begin #60000000;$fatal(1,"I2C compare timeout");end
endmodule
