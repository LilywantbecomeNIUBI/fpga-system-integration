// Asynchronous assertion, synchronous release in each receiving clock domain.
module reset_sync(input clk, input arst_n, output rst_n);
    reg [2:0] release_pipe;
    always @(posedge clk or negedge arst_n)
        if (!arst_n) release_pipe <= 3'b000;
        else release_pipe <= {release_pipe[1:0], 1'b1};
    assign rst_n = release_pipe[2];
endmodule
