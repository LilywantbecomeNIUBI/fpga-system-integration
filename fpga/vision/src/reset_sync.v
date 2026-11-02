// Asynchronous assertion, three-edge synchronous release in each clock domain.
// Parallel final stages share stage 1, never cascade or add release latency.
module reset_sync #(parameter COPIES=1)(input clk, input arst_n, output rst_n,
    output [COPIES-1:0] rst_copies);
    (* syn_preserve=1, syn_keep=1 *) reg [COPIES+1:0] release_pipe;
    always @(posedge clk or negedge arst_n)
        if (!arst_n) release_pipe <= 0;
        else begin
            release_pipe[1:0] <= {release_pipe[0], 1'b1};
            release_pipe[COPIES+1:2] <= {COPIES{release_pipe[1]}};
        end
    assign rst_n = release_pipe[2];
    assign rst_copies = release_pipe[COPIES+1:2];
endmodule
