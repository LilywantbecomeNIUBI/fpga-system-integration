module imu_status_source #(parameter CLK_HZ=50000000,DECIMATION=10)(
    input wire clk,reset_n,sample_valid,ready,fault,sample_stale,temperature_valid,
    input wire signed [15:0] ax,ay,az,gx,gy,gz,temperature_raw,
    input wire [31:0] sample_seq,timestamp_us,now_us,
    input wire [7:0] who_am_i,last_error,
    input wire [15:0] error_count,invalid_count,observe_to_commit_us,sample_interval_us,drop_count,
    output wire [383:0] frame,output wire valid
);
    integer decimation_count,silence;
    reg [31:0] debug_seq;
    reg [367:0] body;
    reg capture;
    wire selected=sample_valid && decimation_count==DECIMATION-1;
    wire [7:0] flags={2'b0,2'b11,temperature_valid,sample_stale,fault,ready};
    localparam [7:0] DECIMATION_BYTE=DECIMATION;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin decimation_count<=0;silence<=0;debug_seq<=0;body<=0;capture<=0;end
        else begin
            capture<=0;
            if(silence<CLK_HZ)silence<=silence+1;
            if(sample_valid)begin
                if(selected)begin decimation_count<=0;debug_seq<=debug_seq+32'd1;end
                else decimation_count<=decimation_count+1;
            end
            if(selected || silence>=CLK_HZ)begin
                body<={8'd150,DECIMATION_BYTE,sample_interval_us,observe_to_commit_us,
                    (selected ? debug_seq+32'd1 : debug_seq),drop_count,invalid_count,error_count,
                    8'h02,last_error,who_am_i,flags,temperature_raw,gz,gy,gx,az,ay,ax,
                    (selected ? timestamp_us : now_us),sample_seq,
                    (selected ? 8'h01 : 8'h02),8'h02,16'h5aa5};
                silence<=0;capture<=1;
            end
        end
    end
    frame_crc #(.HIGH_FIRST(1)) crc(.clk(clk),.reset_n(reset_n),.capture(capture),.body_in(body),.frame(frame),.valid(valid));
endmodule
