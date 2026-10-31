"""Check that actual Gowin synthesis retained the observed-state recovery guard."""
from pathlib import Path
import argparse,hashlib,json,re,subprocess
ROOT=Path(__file__).resolve().parents[1]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--netlist',type=Path,default=ROOT/'build/system/impl/gwsynthesis/system.vg');args=ap.parse_args()
    primitive=Path('C:/Gowin/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
    folder=ROOT/'build/tests';net=args.netlist.read_text(errors='replace');logs=[]
    # Gowin renames/splits hierarchical ports after optimization. Inspect this
    # actual netlist instead of pretending the source module interface survived.
    for module,busy,done,valid,seen,kind in [
        ('flow_lk_patch','patch_busy','patch_done','patch_good',None,None),
        ('flow_lk_patch_0','debug_status_59','debug_status_60','debug_status_61','debug_status_55','debug_status_142')]:
        body=net.split('module '+module+' (')[1].split('endmodule')[0]
        inputs=re.findall(r'^input (?:\[(\d+):(\d+)\] )?(\w+);',body,re.M)
        assert inputs and 'input camera_pclk_d;' in body and 'input n17_6;' in body
        for name in [busy,done,valid]+([seen,kind] if seen else []):
            assert re.search(r'^output '+name+r';',body,re.M),(module,name)
        connections=[]
        for high,low,name in inputs:
            width=int(high)-int(low)+1 if high else 1
            value='clk' if name=='camera_pclk_d' else '~rst_n' if name=='n17_6' else 'rst_n' if name=='cap_rst_n' else str(width)+"'d0"
            connections.append('.'+name+'('+value+')')
        connections.extend('.'+name+'('+wire+')' for name,wire in [(busy,'busy'),(done,'done'),(valid,'valid')])
        if seen:connections.extend(['.'+seen+'(seen)','.'+kind+'(kind)'])
        fault_check=' || !seen || !kind' if seen else ''
        busy_cell=re.search(r'\bDFFCE\s+(\w+)\s*\(\s*\.Q\('+re.escape(busy)+r'\)',body)
        assert busy_cell,(module,busy)
        # Inject the primitive's stored bit. Forcing only its output net leaves
        # the underlying stored zero intact when force is released.
        busy_storage=busy_cell[1]+'.Q_reg'
        bench=folder/('flow_recovery_'+module+'.v');image=bench.with_suffix('.vvp')
        bench.write_text('''`timescale 1ns/1ps
module tb_flow_recovery_synth;
reg clk=0;always #5 clk=~clk;
reg rst_n=0;wire busy,done,valid,seen,kind;
'''+module+' dut('+','.join(connections)+''');
integer completions=0;
always @(negedge clk)if(rst_n && done)completions=completions+1;
initial begin
force GSR.GSRI=1'b1;
repeat(8)@(negedge clk);rst_n=1;
repeat(3)@(negedge clk);
// The candidate uses Gray WAIT_RAM=2, LATCH=6. Exercise the actual
// preserved state bank, not a separate behavioral FSM model.
force dut.state=5'd2;@(negedge clk);release dut.state;
#1;
if(dut.state!==5'd6)$fatal(1,"Synthesized WAIT_RAM did not enter LATCH");
rst_n=0;repeat(4)@(negedge clk);rst_n=1;
repeat(3)@(negedge clk);
force dut.'''+busy_storage+'''=1'b1;@(negedge clk);release dut.'''+busy_storage+''';
repeat(8)@(negedge clk);
if(busy || valid || completions!=1'''+fault_check+''')
$fatal(1,"Synthesized guard missing busy=%b valid=%b count=%0d",busy,valid,completions);
$display("PASS actual Gowin netlist: one invalid completion from captured IDLE/busy contradiction");$finish;
end
initial begin #10000;$fatal(1,"timeout");end
endmodule
''')
        command=['iverilog','-g2012','-s','tb_flow_recovery_synth','-s','GSR','-o',str(image),str(bench),str(args.netlist),str(primitive)]
        result=subprocess.run(command,capture_output=True,text=True,timeout=90)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        result=subprocess.run(['vvp',str(image)],capture_output=True,text=True,timeout=30)
        log=result.stdout+result.stderr;assert result.returncode==0 and 'PASS actual Gowin' in log,log
        logs.append(module+': '+log)
    log='\n'.join(logs);(folder/'flow_recovery_synth.log').write_text(log)
    record=dict(passed=True,instances=2,netlist_sha256=hashlib.sha256(args.netlist.read_bytes()).hexdigest(),hardware_tested=False)
    (folder/'flow_recovery_synth.json').write_text(json.dumps(record,indent=2)+'\n');print(log);print(json.dumps(record))
if __name__=='__main__':main()
