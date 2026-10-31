"""Compare the actual synthesized dual-clock front end with its RTL interface."""
from pathlib import Path
import hashlib,json,re,subprocess
ROOT=Path(__file__).resolve().parents[1]
def main():
    out=ROOT/'build/tests';vg=ROOT/'build/system/impl/gwsynthesis/system.vg'
    net=vg.read_text();body=net.split('module vision_frame_cdc (')[1].split('endmodule')[0]
    clear=re.search(r'\bDFF\w*\s+state_\w+\s*\(.*?\.CLEAR\((\w+)\)',body,re.S)[1]
    inputs=re.findall(r'^input (?:\[\d+:\d+\] )?(\w+);',body,re.M)
    values=dict(camera_pclk_d='wc',clk50='rc',os='fs',cap_rst_n='ready',front_rst_n='rst',
                red_Z='red',ov='pv',og='fg',oe='fe',luma_Z='gray',oy='y[7:0]',ox='x[8:0]',ofid='fid')
    values[clear]='~rst';assert set(inputs)==set(values),(inputs,values)
    outputs=dict(proc_invalidate='iv_b',proc_fs='os_b',proc_fe='oe_b',proc_fg='og_b',proc_pv='ov_b',
                 proc_red='red_b',held='id_b',proc_x='x_b',proc_y='y_b',proc_luma='gray_b')
    ports=','.join('.'+k+'('+v+')' for k,v in list(values.items())+list(outputs.items()))
    src=ROOT/'fpga/vision/src/vision'
    (out/'frame_cdc_ref.v').write_text((src/'vision_frame_cdc.v').read_text().replace('vision_frame_cdc','vision_frame_cdc_ref').replace('vision_async_fifo','vision_async_fifo_ref'))
    (out/'async_fifo_ref.v').write_text((src/'vision_async_fifo.v').read_text().replace('vision_async_fifo','vision_async_fifo_ref'))
    bench='''`timescale 1ns/1ps
module tb_frame_cdc_synth;
reg wc=0,rc=0,run_rd=1,rst=0,ready=1,fs=0,fe=0,fg=0,pv=0,red=0;
always #5.95235 wc=~wc;
always begin #10;if(run_rd)rc=~rc;else rc=0;end
reg [15:0] x=0,y=0,fid=0;reg [7:0] gray=0;
wire iv_a,iv_b,os_a,os_b,oe_a,oe_b,og_a,og_b,ov_a,ov_b,red_a,red_b;
wire [15:0] id_a,id_b,xa,ya;wire [8:0] x_b;wire [7:0] y_b,gray_a,gray_b;
vision_frame_cdc_ref refdut(.wr_clk(wc),.wr_rst_n(rst),.capture_ready(ready),.frame_start(fs),.frame_end(fe),.frame_good(fg),.pixel_valid(pv),.frame_id(fid),.x(x),.y(y),.luma(gray),.red(red),.rd_clk(rc),.rd_rst_n(rst),.out_start(os_a),.out_end(oe_a),.out_good(og_a),.out_valid(ov_a),.out_id(id_a),.out_x(xa),.out_y(ya),.out_luma(gray_a),.out_red(red_a),.invalidate(iv_a));
vision_frame_cdc dut(PORTS);
integer pixels=0,ends=0,invalids=0,i;
always @(negedge rc)if(rst)begin
 if({iv_a,os_a,oe_a,og_a,ov_a}!=={iv_b,os_b,oe_b,og_b,ov_b})$fatal(1,"RTL/synthesis flag mismatch");
 if(iv_b)invalids=invalids+1;
 if(ov_b)begin
  if({id_a,xa[8:0],ya[7:0],gray_a,red_a}!=={id_b,x_b,y_b,gray_b,red_b})$fatal(1,"RTL/synthesis data mismatch");
  pixels=pixels+1;
 end
 if(oe_b && og_b)ends=ends+1;
end
initial begin
 force GSR.GSRI=1'b1;
 #137;rst=1;repeat(30)@(negedge wc);
 fid=42;
 for(i=0;i<57600;i=i+1)begin
  @(negedge wc);fs=i==0;fe=i==57599;fg=fe;pv=1;x=i%320;y=i/320;gray=i;red=i&1;
  @(negedge wc);fs=0;fe=0;pv=0;fg=0;repeat(6)@(negedge wc);
 end
 repeat(100)@(negedge wc);
 if(pixels!=57600 || ends!=1 || invalids!=0)$fatal(1,"Incomplete synthesized frame");
 // A camera reset and a full FIFO must invalidate on the same cycles as RTL.
 ready=0;repeat(100)@(negedge wc);ready=1;repeat(100)@(negedge wc);
 run_rd=0;fs=1;@(negedge wc);fs=0;pv=1;
 repeat(300)@(negedge wc);pv=0;run_rd=1;repeat(200)@(negedge wc);
 if(invalids<2)$fatal(1,"Synthesized abort protocol missing");
 $display("PASS actual synthesized frame CDC: 57600 exact pixels, feature stream flags, camera reset and FIFO overflow");$finish;
end
initial begin #10000000;$fatal(1,"timeout");end
endmodule
'''.replace('PORTS',ports)
    (out/'tb_frame_cdc_synth.v').write_text(bench)
    cmd=['iverilog','-g2012','-s','tb_frame_cdc_synth','-s','GSR','-o',str(out/'frame_cdc_synth.vvp'),
         str(out/'tb_frame_cdc_synth.v'),str(out/'frame_cdc_ref.v'),str(out/'async_fifo_ref.v'),str(vg),
         'C:/Gowin/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v']
    p=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=90);assert p.returncode==0,p.stdout+p.stderr
    p=subprocess.run(['vvp',str(out/'frame_cdc_synth.vvp')],cwd=ROOT,capture_output=True,text=True,timeout=300)
    (out/'frame_cdc_synth.txt').write_text(p.stdout+p.stderr);assert p.returncode==0,p.stdout+p.stderr
    record=dict(passed=True,hardware_tested=False,netlist_sha256=hashlib.sha256(vg.read_bytes()).hexdigest(),pixels=57600)
    (out/'frame_cdc_synth.json').write_text(json.dumps(record,indent=2)+'\n');print(p.stdout)
if __name__=='__main__':main()
