"""Current packed-map synthesis + real Gowin SDPB functional model. No PnR/IO."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/mecanum_navigation/grid_synthesis'
IDE=Path('C:/Gowin/Gowin_V1.9.12.04_x64/IDE')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    sources=[ROOT/p for p in ('fpga/navigation/mc_nav_grid.sv','fpga/navigation/mc_nav_sweep.sv',
              'fpga/navigation/mc_nav_cell_index.sv','fpga/mecanum/cordic_sincos.sv','tests/navigation/tb_mc_nav_memory.sv')]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in sources}
    project=ET.Element('Project');ET.SubElement(project,'Template').text='FPGA';ET.SubElement(project,'Version').text='5'
    ET.SubElement(project,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012'
    listing=ET.SubElement(project,'FileList');ET.SubElement(listing,'File',path=str(sources[0]),type='file.verilog',enable='1')
    ET.ElementTree(project).write(OUT/'grid.gprj',encoding='utf-8',xml_declaration=True)
    (OUT/'syn.tcl').write_text('open_project {'+(OUT/'grid.gprj').as_posix()+'}\nset_option -top_module mc_nav_grid\nset_option -verilog_std sysv2017\nset_option -output_base_name grid\nrun syn\nexit\n')
    net=OUT/'impl/gwsynthesis/grid.vg';primitive=IDE/'simlib/gw5a/prim_sim.v'
    commands=[([str(IDE/'bin/gw_sh.exe'),str(OUT/'syn.tcl')],OUT,'synthesis'),
              (['iverilog','-g2012','-s','tb_mc_nav_memory','-s','GSR','-o',str(OUT/'grid_gate.vvp'),
                str(sources[4]),str(net),*map(str,sources[1:4]),str(primitive)],ROOT,'compile'),
              (['vvp',str(OUT/'grid_gate.vvp')],ROOT,'simulation')]
    for command,cwd,label in commands:
        p=subprocess.run(command,cwd=cwd,capture_output=True,text=True,timeout=120)
        (OUT/(label+'_gate.log')).write_text(p.stdout+p.stderr,encoding='utf-8')
        assert p.returncode==0,p.stdout+p.stderr
        if label=='simulation':print(p.stdout,end='')
    text=net.read_text();assert len(re.findall(r'\bSDPB\s+',text))==4
    assert len(re.findall(r'\.READ_MODE=1\'b0',text))==4
    assert all(sha(ROOT/p)==h for p,h in hashes.items())
    result=dict(passed=True,scope='Standalone current map synthesis + SDPB primitive functional simulation',
                full_system_gate_level_simulated=False,routed_timing_signoff=False,programmed=False,
                source_sha256=hashes,netlist_sha256=sha(net),primitive_model_sha256=sha(primitive),map_bsram=4,read_mode=0)
    (ROOT/'build/mecanum_navigation/grid_gate_result.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    main()
