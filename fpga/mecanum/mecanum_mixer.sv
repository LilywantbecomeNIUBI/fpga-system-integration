// X roller layout, viewed from above; order FL, FR, RL, RR.
// x forward, y left, z counterclockwise tangential wheel contribution.
module mc_mecanum_mixer #(parameter integer TARGET_LIMIT=1024)(
 input wire signed [15:0] x,y,z,
 output wire signed [15:0] fl,fr,rl,rr
);
 wire signed [17:0] a=$signed({{2{x[15]}},x})-$signed({{2{y[15]}},y})-$signed({{2{z[15]}},z});
 wire signed [17:0] b=$signed({{2{x[15]}},x})+$signed({{2{y[15]}},y})+$signed({{2{z[15]}},z});
 wire signed [17:0] c=$signed({{2{x[15]}},x})+$signed({{2{y[15]}},y})-$signed({{2{z[15]}},z});
 wire signed [17:0] d=$signed({{2{x[15]}},x})-$signed({{2{y[15]}},y})+$signed({{2{z[15]}},z});
 wire wide=(a>2*TARGET_LIMIT || a< -2*TARGET_LIMIT || b>2*TARGET_LIMIT || b< -2*TARGET_LIMIT || c>2*TARGET_LIMIT || c< -2*TARGET_LIMIT || d>2*TARGET_LIMIT || d< -2*TARGET_LIMIT);
 wire over=(a>TARGET_LIMIT || a< -TARGET_LIMIT || b>TARGET_LIMIT || b< -TARGET_LIMIT || c>TARGET_LIMIT || c< -TARGET_LIMIT || d>TARGET_LIMIT || d< -TARGET_LIMIT);
 wire [1:0] shift=wide ? 2 : over ? 1 : 0;
 // Inputs validated to +/-1024 by command controller. Shared scaling keeps ratios.
 assign fl=a>>>shift; assign fr=b>>>shift; assign rl=c>>>shift; assign rr=d>>>shift;
endmodule
