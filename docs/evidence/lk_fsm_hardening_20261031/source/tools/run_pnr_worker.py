"""Run an isolated Place strategy on the comparison's frozen synthesis netlist."""
import argparse,json,re,shutil,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
from build_system import ROOT,sha
from gowin_runner import run_gowin

def main():
    ap=argparse.ArgumentParser();ap.add_argument('stamp');ap.add_argument('place',type=int,choices=(3,4));a=ap.parse_args()
    assert re.fullmatch(r'\d{8}T\d{6}Z',a.stamp)
    parent=ROOT/'build'/('pnr_compare_'+a.stamp);e=ROOT/'docs/evidence'/('pnr_optimization_'+a.stamp)
    identity=json.loads((parent/'source_identity.json').read_text())
    assert all(sha(ROOT/p)==h for p,h in identity.items())
    audit=json.loads((e/'netlist_audit.json').read_text());source=ROOT/'build/system'
    assert sha(source/'impl/gwsynthesis/system.vg')==audit['netlist_sha256']
    frozen=parent/'frozen.vg'
    if not frozen.exists():shutil.copyfile(source/'impl/gwsynthesis/system.vg',frozen)
    assert sha(frozen)==audit['netlist_sha256']
    work=parent/('worker_'+str(a.place));work.mkdir(exist_ok=True)
    shutil.copytree(source/'impl/gwsynthesis',work/'impl/gwsynthesis',dirs_exist_ok=True)
    project=ET.parse(source/'system.gprj')
    for f in project.getroot().find('FileList'):
        f.set('path',(source/f.get('path')).resolve().as_posix())
    project.write(work/'system.gprj',encoding='utf-8',xml_declaration=True)
    # A newly opened project refreshes synthesis state even for run pnr.
    # Establish its cache, then explicitly restore the identical frozen VG.
    tcl=(source/'build.tcl').read_text().replace('run all','run syn\nfile copy -force {'+frozen.as_posix()+'} {'+(work/'impl/gwsynthesis/system.vg').as_posix()+'}\nrun pnr')
    tcl=re.sub(r'set_option -place_option \d+','set_option -place_option '+str(a.place),tcl)+'\nexit\n'
    (work/'build.tcl').write_text(tcl)
    case=e/('place_'+str(a.place));case.mkdir(exist_ok=True)
    tool_result=run_gowin('C:/Gowin/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe',work/'build.tcl',case/'build.log',pnr=True)
    assert sha(work/'impl/gwsynthesis/system.vg')==audit['netlist_sha256']
    p=work/'impl/pnr';text=(p/'system.tr').read_text(errors='replace')
    table=text.split('3.1.1 Setup Paths Table')[-1].split('3.1.2 Hold Paths Table')[0]
    row=dict(place_option=a.place,route_option=1,timing_driven=1,
             setup=int(re.search(r'<Numbers of Setup Violated Endpoints>:(\d+)',text)[1]),
             hold=int(re.search(r'<Numbers of Hold Violated Endpoints>:(\d+)',text)[1]),
             wns_ns=float(re.search(r'^\s+1\s+([-\d.]+)',table,re.M)[1]),
             min_pulse_slack_ns=min(float(v) for v in re.findall(r'MPW Summary:\s+Slack:\s+([-\d.]+)',text)),
             clock_route_warnings=[s for s in (p/'system.log').read_text().splitlines() if 'PR1014' in s],
             netlist_sha256=audit['netlist_sha256'],bitstream_sha256=sha(p/'system.fs'))
    row['tool_result']=tool_result
    for src,dest in [('system.tr','timing.txt'),('system_tr_content.html','timing.html'),('system.rpt.txt','pnr_report.txt'),('system.log','pnr.log'),('cmd.do','cmd.do')]:shutil.copyfile(p/src,case/dest)
    for name in ('system.sdc','system.cst'):shutil.copyfile(source/name,case/name)
    bits=parent/'bits'/str(a.place);bits.mkdir(parents=True);shutil.copyfile(p/'system.fs',bits/'system.fs')
    (case/'result.json').write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row),flush=True)

if __name__=='__main__':main()
