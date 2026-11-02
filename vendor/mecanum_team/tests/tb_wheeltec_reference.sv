`timescale 1ns/1ps
// Independent reference from WHEELTEC 2026-09-21 BALANCE/balance.c.
// Their A=RL, B=FL, C=FR, D=RR (2022 non-symmetric manual Fig1-3).
// Equal arm lengths: z=omega*(half axle spacing + half wheel spacing).
module tb_wheeltec_reference;
 reg signed [15:0] x,y,z;
 wire signed [15:0] fl,fr,rl,rr;
 integer ix,iy,iz,a,b,c,d,scale,cases=0;
 mecanum_mixer dut(.x(x),.y(y),.z(z),.fl(fl),.fr(fr),.rl(rl),.rr(rr));
 initial begin
  for(ix=-1024;ix<=1024;ix=ix+256)
   for(iy=-1024;iy<=1024;iy=iy+256)
    for(iz=-1024;iz<=1024;iz=iz+256)begin
     x=ix;y=iy;z=iz;#1;
     // Original manufacturer's A/B/C/D order, not our output order.
     a=+iy+ix-iz;b=-iy+ix-iz;c=+iy+ix+iz;d=-iy+ix+iz;
     scale=(a>2048||a< -2048||b>2048||b< -2048||c>2048||c< -2048||d>2048||d< -2048)?2:
           (a>1024||a< -1024||b>1024||b< -1024||c>1024||c< -1024||d>1024||d< -1024)?1:0;
     if(fl!==(b>>>scale) || fr!==(c>>>scale) || rl!==(a>>>scale) || rr!==(d>>>scale))
      $fatal(1,"manufacturer mapping mismatch x=%0d y=%0d z=%0d",ix,iy,iz);
     cases=cases+1;
    end
  // Deliberately unequal arms prove why current equal-z RTL cannot serve
  // a staggered chassis by changing motor polarity alone.
  ix=0;iy=0;iz=100;
  a=-iz*3;b=-iz*2;c=iz*3;d=iz*2;
  if(a==b || c==d)$fatal(1,"asymmetric reference fixture");
  $display("PASS WHEELTEC reference: %0d combinations, A/B/C/D permutation and shared scaling; unequal-arm limitation recorded",cases);
  $finish;
 end
endmodule
