// X roller layout, viewed from above; order FL, FR, RL, RR.
// x forward, y left, z counterclockwise tangential wheel contribution.
module mc_mecanum_mixer(
 input wire signed [15:0] x,y,z,
 output wire signed [15:0] fl,fr,rl,rr
);
 wire signed [17:0] a=$signed({{2{x[15]}},x})-$signed({{2{y[15]}},y})-$signed({{2{z[15]}},z});
 wire signed [17:0] b=$signed({{2{x[15]}},x})+$signed({{2{y[15]}},y})+$signed({{2{z[15]}},z});
 wire signed [17:0] c=$signed({{2{x[15]}},x})+$signed({{2{y[15]}},y})-$signed({{2{z[15]}},z});
 wire signed [17:0] d=$signed({{2{x[15]}},x})-$signed({{2{y[15]}},y})+$signed({{2{z[15]}},z});
 wire wide=(a>2048 || a< -2048 || b>2048 || b< -2048 || c>2048 || c< -2048 || d>2048 || d< -2048);
 wire over=(a>1024 || a< -1024 || b>1024 || b< -1024 || c>1024 || c< -1024 || d>1024 || d< -1024);
 wire [1:0] shift=wide ? 2 : over ? 1 : 0;
 // Inputs validated to +/-1024 by command controller. Shared scaling keeps ratios.
 assign fl=a>>>shift; assign fr=b>>>shift; assign rl=c>>>shift; assign rr=d>>>shift;
endmodule
