`timescale 1ns/1ps
// A complete third capture ends exactly when the solver finishes. Ownership
// must remain held while downstream avoidance drains its final result.
module tb_flow_drain;
    reg clk=0,rst=0,fs=0,fe=0,pv=0;always #5 clk=~clk;
    reg [15:0] fid=0,x=0,y=0;wire busy,done,lost,status;
    wire [31:0] drops;wire [15:0] status_id;
    flow_avoidance_top #(.WIDTH(24),.HEIGHT(24)) dut(
        .clk(clk),.rst_n(rst),.frame_start(fs),.frame_end(fe),.frame_good(1'b1),
        .pixel_valid(pv),.frame_id(fid),.x(x),.y(y),.luma(8'd100),
        .busy(busy),.frame_done(done),.sample_valid(),.sample_good(),.history_lost(lost),
        .result_id(),.flow_x(),.flow_y(),.flow_u(),.flow_v(),.cycles(),.dropped_frames(drops),
        .status_valid(status),.status_id(status_id),.state(),.hazard(),.stop_request(),.steer(),.speed_limit(),
        .left_valid(),.center_valid(),.right_valid(),.left_risk(),.center_risk(),.right_risk());
    task pixels;input [15:0] id;integer row,col;
        begin
            @(negedge clk);fid=id;fs=1;@(negedge clk);fs=0;
            for(row=0;row<24;row=row+1)for(col=0;col<24;col=col+1)begin
                pv=1;x=col;y=row;@(negedge clk);
            end
            pv=0;
        end
    endtask
    task finish_frame;
        begin fe=1;@(negedge clk);fe=0;end
    endtask
    integer i;reg solver_finished=0,saw_status=0;
    always @(negedge clk)if(status && status_id==2)saw_status=1;
    initial begin
        repeat(4)@(negedge clk);rst=1;
        pixels(1);finish_frame();repeat(4)@(negedge clk);
        pixels(2);finish_frame();wait(busy);
        pixels(3);
        // Solver work must outlast capture so this exercises a real FE race.
        if(done || !busy)$fatal(1,"fixture did not overlap capture/solver");
        wait(done);@(negedge clk);
        if(!busy)$fatal(1,"banks released before the consumer drain");
        finish_frame();
        if(drops!=1 || !lost)$fatal(1,"frame at solver completion overwrote owned banks");
        for(i=0;i<3;i=i+1)begin
            @(negedge clk);if(!busy)$fatal(1,"ownership expired during downstream drain");
        end
        wait(!busy);repeat(3)@(negedge clk);
        if(!saw_status)$fatal(1,"drained frame status disappeared");
        $display("PASS real flow ownership: simultaneous capture completion/solver done retains banks through avoidance drain");
        $finish;
    end
    initial begin #1000000;$fatal(1,"flow drain timeout");end
endmodule
