// Bounded WIDTH-cycle unsigned restoring divider; default preserves the vision API.
module vision_divider #(parameter WIDTH=32)(input clk,rst_n,start,input [WIDTH-1:0] numerator,denominator,
    output reg busy,done,output reg [WIDTH-1:0] quotient,output [7:0] debug_count);
    reg [WIDTH-1:0] n,d;reg [WIDTH:0] remainder;reg [$clog2(WIDTH)-1:0] count;
    wire [WIDTH:0] trial={remainder[WIDTH-1:0],n[WIDTH-1]};
    assign debug_count=count;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin busy<=0;done<=0;quotient<=0;n<=0;d<=0;remainder<=0;count<=0;end
        else begin
            done<=0;
            if(start && !busy)begin
                n<=numerator;d<=denominator;remainder<=0;quotient<=0;count<=0;
                if(denominator==0)done<=1;else busy<=1;
            end else if(busy)begin
                n<={n[WIDTH-2:0],1'b0};
                if(trial>={1'b0,d})begin remainder<=trial-{1'b0,d};quotient<={quotient[WIDTH-2:0],1'b1};end
                else begin remainder<=trial;quotient<={quotient[WIDTH-2:0],1'b0};end
                if(count==WIDTH-1)begin busy<=0;done<=1;end else count<=count+1'b1;
            end
        end
    end
endmodule
