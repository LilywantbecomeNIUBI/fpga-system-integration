"""Compare actual target LK netlist with RTL through its image RAM interface."""
from pathlib import Path
import subprocess,json,hashlib,math

def main():
    out=Path('build/tests');vg=Path('build/system/impl/gwsynthesis/system.vg')
    source=Path('fpga/vision/src/optical_flow/flow_lk_patch.v')
    (out/'lk_reference.v').write_text(source.read_text().replace('module flow_lk_patch','module flow_lk_patch_ref').replace('vision_divider #','vision_divider_ref #'))
    (out/'divider_reference.v').write_text(Path('fpga/vision/src/vision/vision_divider.v').read_text().replace('module vision_divider','module vision_divider_ref'))
    def pixel(x,y):return max(0,min(255,round(128+40*math.sin(x*.43)+35*math.cos(y*.37)+20*math.sin((x+y)*.21))))
    cases=[(0,0),(1,0),(-1,0),(0,1),(0,-1)]
    for k,(dx,dy) in enumerate(cases):
        for kind in ['previous','current']:
            a=[pixel(x-(dx if kind=='current' else 0),y-(dy if kind=='current' else 0)) for y in range(45) for x in range(80)]
            (out/('lk_'+kind+str(k)+'.mem')).write_text(''.join('%02x\n'%v for v in a))
    bench='''`timescale 1ns/1ps
module tb_lk_math_synth;
reg clk=0;always #5 clk=~clk;
reg rst_n=0,start=0;reg [7:0] oldmem[0:3599],newmem[0:3599];
reg [7:0] data_a=0,data_b=0;wire [15:0] addr_a;wire [11:0] addr_b;
wire cur_a,cur_b,done_a,done_b,valid_a,valid_b;wire signed [15:0] u_a,v_a,u_b,v_b;
reg seen_a=0,seen_b=0;reg va,vb;reg signed [15:0] ua,va_flow,ub,vb_flow;
flow_lk_patch_ref #(.WIDTH(80),.HEIGHT(45)) refdut(.clk(clk),.rst_n(rst_n),.start(start),.coarse(1'b0),.center_x(16'd40),.center_y(16'd24),.initial_u(16'd0),.initial_v(16'd0),.read_address(addr_a),.read_current(cur_a),.read_data(data_a),.done(done_a),.valid(valid_a),.u(u_a),.v(v_a));
flow_lk_patch dut(.camera_pclk_d(clk),.n17_6(~rst_n),.cap_rst_n(rst_n),.patch_start(start),.patch_center_x(16'd40),.patch_center_y(16'd24),.seed_u(16'd0),.seed_v(16'd0),.read_word(data_b),.lk_address(addr_b),.lk_current(cur_b),.patch_done(done_b),.patch_good(valid_b),.pu(u_b),.pv(v_b));
always @(posedge clk)begin
 if(addr_a<3600)data_a<=cur_a ? newmem[addr_a]:oldmem[addr_a];
 if(addr_b<3600)data_b<=cur_b ? newmem[addr_b]:oldmem[addr_b];
end
always @(negedge clk)begin
 if(!rst_n)begin seen_a=0;seen_b=0;end
 else begin
  if(done_a)begin seen_a=1;va=valid_a;ua=u_a;va_flow=v_a;end
  if(done_b)begin seen_b=1;vb=valid_b;ub=u_b;vb_flow=v_b;end
 end
end
integer k;
initial begin
 force GSR.GSRI=1'b1;
'''
    for k in range(len(cases)):
        bench+='''rst_n=0;repeat(4)@(negedge clk);
$readmemh("build/tests/lk_previous%d.mem",oldmem);$readmemh("build/tests/lk_current%d.mem",newmem);
rst_n=1;repeat(3)@(negedge clk);start=1;@(negedge clk);start=0;
wait(seen_a && seen_b);@(negedge clk);
if(!va || va!==vb || ua!==ub || va_flow!==vb_flow)$fatal(1,"Case %d RTL %%b %%d %%d synthesis %%b %%d %%d",va,ua,va_flow,vb,ub,vb_flow);
$display("PASS synthesized image case %d u=%%d v=%%d",ub,vb_flow);
'''%(k,k,k,k)
    bench+='$finish;end\ninitial begin #1000000;$fatal(1,"timeout");end\nendmodule\n'
    (out/'tb_lk_math_synth.v').write_text(bench)
    cmd=['iverilog','-g2012','-s','tb_lk_math_synth','-s','GSR','-o',str(out/'lk_math_synth.vvp'),str(out/'tb_lk_math_synth.v'),str(out/'lk_reference.v'),str(out/'divider_reference.v'),str(vg),'C:/Gowin/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v']
    r=subprocess.run(cmd,capture_output=True,text=True,timeout=90);assert r.returncode==0,r.stdout+r.stderr
    r=subprocess.run(['vvp',str(out/'lk_math_synth.vvp')],capture_output=True,text=True,timeout=90)
    (out/'lk_math_synth.txt').write_text(r.stdout+r.stderr);assert r.returncode==0,r.stdout+r.stderr
    result=dict(passed=True,cases=len(cases),netlist_sha256=hashlib.sha256(vg.read_bytes()).hexdigest(),rtl_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_tested=False)
    (out/'lk_math_synth.json').write_text(json.dumps(result,indent=2));print(r.stdout)

if __name__=='__main__':main()
