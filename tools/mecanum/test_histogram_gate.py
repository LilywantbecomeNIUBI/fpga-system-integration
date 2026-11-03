"""Synthesize the enabled synchronous-clear histogram and compare SDPB model to old RTL."""
from pathlib import Path
import hashlib,json,re,subprocess,xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/mecanum_navigation/histogram_synthesis'
IDE=Path('C:/Gowin/Gowin_V1.9.12.04_x64/IDE')


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    source=ROOT/'fpga/navigation/nav_histogram.sv'
    text=source.read_text()
    header=text[text.index('module nav_histogram'):text.index(');')+2]
    wrapper=header.replace('nav_histogram #(parameter SYNC_CLEAR=0)','histogram_gate',1)
    (OUT/'wrapper.sv').write_text(wrapper+'\nnav_histogram #(.SYNC_CLEAR(1)) impl(.*);\nendmodule\n')
    p=ET.Element('Project');ET.SubElement(p,'Template').text='FPGA';ET.SubElement(p,'Version').text='5'
    ET.SubElement(p,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012'
    fl=ET.SubElement(p,'FileList')
    for path in (source,OUT/'wrapper.sv'):ET.SubElement(fl,'File',path=str(path),type='file.verilog',enable='1')
    ET.ElementTree(p).write(OUT/'hist.gprj',encoding='utf-8',xml_declaration=True)
    (OUT/'syn.tcl').write_text('open_project {'+(OUT/'hist.gprj').as_posix()+'}\nset_option -top_module histogram_gate\nset_option -verilog_std sysv2017\nset_option -output_base_name hist\nrun syn\nexit\n')
    bench=ROOT/'tests/navigation/tb_mc_nav_histogram_sdp.sv'
    modified=bench.read_text().replace('nav_histogram #(.SYNC_CLEAR(1)) dut','histogram_gate dut')
    # Pointer registers are optimized/renamed in the netlist. The gate bench
    # checks externally visible clear/unknown behavior; RTL also checks pointers.
    modified=modified.replace(' || dut.wp || dut.rp','')
    modified=modified.replace('initial begin\n  repeat(4)','initial begin\n  force GSR.GSRI=1\'b1;\n  repeat(4)',1)
    (OUT/'bench.sv').write_text(modified)
    oracle=ROOT/'tests/navigation/reference/nav_histogram_reference.sv'
    commands=[([str(IDE/'bin/gw_sh.exe'),str(OUT/'syn.tcl')],'synthesis'),
        (['iverilog','-g2012','-s','tb_mc_nav_histogram_sdp','-s','GSR','-o',str(OUT/'gate.vvp'),str(OUT/'bench.sv'),
          str(OUT/'impl/gwsynthesis/hist.vg'),str(oracle),str(IDE/'simlib/gw5a/prim_sim.v')],'compile'),
        (['vvp',str(OUT/'gate.vvp')],'simulation')]
    for command,label in commands:
        r=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=120)
        (OUT/(label+'.log')).write_text(r.stdout+r.stderr)
        assert not r.returncode,r.stdout+r.stderr
        if label=='simulation':print(r.stdout,end='')
    net=OUT/'impl/gwsynthesis/hist.vg';data=net.read_text()
    assert len(re.findall(r'\bSDPB\s+',data))==4
    assert len(re.findall(r"\.READ_MODE=1'b0",data))==4
    result=dict(passed=True,scope='Standalone synchronous-clear histogram SDPB functional model, no routed STA',
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source,bench,oracle)},
        netlist_sha256=hashlib.sha256(net.read_bytes()).hexdigest(),sdpb=4,programmed=False)
    (OUT/'gate_result.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
