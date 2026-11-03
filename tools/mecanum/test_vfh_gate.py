"""Run all existing VFH fixtures through the current synthesized SDPB unit."""
from pathlib import Path
import hashlib,json,re,subprocess,sys,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/mecanum_navigation/vfh_synthesis'
IDE=Path('C:/Gowin/Gowin_V1.9.12.04_x64/IDE')


def main():
    sys.path.insert(0,str(ROOT/'tools/task4'))
    from test_vfh import run
    expected=run();rtl=(ROOT/'build/task4/vfh_output.txt').read_text().splitlines()
    OUT.mkdir(parents=True,exist_ok=True);source=ROOT/'fpga/navigation/nav_vfh.sv'
    p=ET.Element('Project');ET.SubElement(p,'Template').text='FPGA';ET.SubElement(p,'Version').text='5'
    ET.SubElement(p,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012'
    fl=ET.SubElement(p,'FileList');ET.SubElement(fl,'File',path=str(source),type='file.verilog',enable='1')
    ET.ElementTree(p).write(OUT/'vfh.gprj',encoding='utf-8',xml_declaration=True)
    (OUT/'syn.tcl').write_text('open_project {'+(OUT/'vfh.gprj').as_posix()+'}\nset_option -top_module nav_vfh\nset_option -verilog_std sysv2017\nset_option -output_base_name vfh\nrun syn\nexit\n')
    bench=(ROOT/'tests/navigation/tb_vfh.sv').read_text().replace('module tb_vfh;','module tb_vfh_gate;')
    bench=bench.replace('build/task4/vfh_output.txt','build/mecanum_navigation/vfh_synthesis/gate_output.txt')
    bench=bench.replace('initial begin','initial begin force GSR.GSRI=1\'b1;',1)
    (OUT/'tb_vfh_gate.sv').write_text(bench)
    net=OUT/'impl/gwsynthesis/vfh.vg'
    commands=[([str(IDE/'bin/gw_sh.exe'),str(OUT/'syn.tcl')],'synthesis'),
        (['iverilog','-g2012','-s','tb_vfh_gate','-s','GSR','-o',str(OUT/'gate.vvp'),str(OUT/'tb_vfh_gate.sv'),str(net),str(IDE/'simlib/gw5a/prim_sim.v')],'compile'),
        (['vvp',str(OUT/'gate.vvp')],'gate')]
    for command,label in commands:
        r=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=120)
        (OUT/(label+'.log')).write_text(r.stdout+r.stderr)
        assert not r.returncode,r.stdout+r.stderr
    assert [r.split() for r in rtl]==[r.split() for r in (OUT/'gate_output.txt').read_text().splitlines()]
    text=net.read_text();assert len(re.findall(r'\bSDPB\s+',text))==1
    result=dict(passed=True,**expected,primitive='SDPB',read_mode=0,unsupported_sp_write_mode=False,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),netlist_sha256=hashlib.sha256(net.read_bytes()).hexdigest(),routed_timing_signoff=False)
    (OUT/'gate_result.json').write_text(json.dumps(result,indent=2)+'\n');print(result)


if __name__=='__main__':main()
