"""Isolated PnR comparisons of the same control50 synthesis netlist. No download."""
import argparse,concurrent.futures,json,re,shutil,xml.etree.ElementTree as ET
from datetime import datetime,timezone
from pathlib import Path
from build_system import ROOT,prepare,sha
from gowin_runner import run_gowin
from audit_control50_timing import audit

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--places',type=int,nargs='+',default=[2,3,4]);args=ap.parse_args()
    assert set(args.places)<={2,3,4}
    base,sources=prepare()
    parent=ROOT/'build'/('control50_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));parent.mkdir()
    frozen=parent/'frozen.vg';shutil.copyfile(base/'impl/gwsynthesis/system.vg',frozen)
    vg_hash=sha(frozen)
    identity={p.relative_to(ROOT).as_posix():sha(p) for p in sources}
    identity['fpga/vision/constraints/perception_cdc.sdc']=sha(ROOT/'fpga/vision/constraints/perception_cdc.sdc')
    (parent/'source_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    (ROOT/'runs/commissioning/control50_latest.txt').write_text(str(parent))
    def worker(place):
        work=parent/('place_'+str(place));work.mkdir()
        shutil.copytree(base/'impl/gwsynthesis',work/'impl/gwsynthesis')
        for n in ['system.sdc','system.cst']:shutil.copyfile(base/n,work/n)
        project=ET.parse(base/'system.gprj')
        for f in project.getroot().find('FileList'):
            original=(base/f.get('path')).resolve()
            f.set('path',((work/original.name) if original.suffix in ('.sdc','.cst') else original).as_posix())
        project.write(work/'system.gprj',encoding='utf-8',xml_declaration=True)
        tcl=(base/'build.tcl').read_text().replace('run all','run syn\nfile copy -force {'+frozen.as_posix()+'} {'+(work/'impl/gwsynthesis/system.vg').as_posix()+'}\nrun pnr\nexit')
        tcl=re.sub(r'set_option -place_option \d+','set_option -place_option '+str(place),tcl)
        (work/'build.tcl').write_text(tcl)
        result=run_gowin('C:/Gowin/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe',work/'build.tcl',work/'build.log',pnr=True)
        assert sha(work/'impl/gwsynthesis/system.vg')==vg_hash
        assert all(sha(ROOT/p)==h for p,h in identity.items()),'Input changed during PnR'
        p=work/'impl/pnr';text=(p/'system.tr').read_text(errors='replace')
        # Per-clock custom reports replace Gowin's default Setup table.
        timing_audit=audit(work)
        row=dict(place_option=place,route_option=1,timing_driven=1,replicate_resources=0,
                 setup=int(re.search(r'<Numbers of Setup Violated Endpoints>:(\d+)',text)[1]),
                 hold=int(re.search(r'<Numbers of Hold Violated Endpoints>:(\d+)',text)[1]),
                 control50_wns_ns=timing_audit['control50_wns_ns'],cam_pclk_wns_ns=timing_audit['cam_pclk_wns_ns'],global_wns_ns=None,
                 min_pulse_slack_ns=min(float(v) for v in re.findall(r'MPW Summary:\s+Slack:\s+([-\d.]+)',text)),
                 netlist_sha256=vg_hash,bitstream_sha256=sha(p/'system.fs'),tool_result=result,
                 programmed=False,hardware_tested=False,timing_signoff=False,
                 clock_route_warnings=[s for s in (p/'system.log').read_text().splitlines() if 'PR1014' in s])
        row['setup_hold_passed']=row['setup']==row['hold']==0
        (work/'result.json').write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row),flush=True)
        return row
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(args.places)) as pool:
        rows=list(pool.map(worker,args.places))
    passing=[r for r in rows if r['setup_hold_passed']]
    record=dict(cases=rows,source_sha256=identity,netlist_sha256=vg_hash,
                selected_place=max(passing,key=lambda r:r['control50_wns_ns'])['place_option'] if passing else None,
                timing_signoff=False,hardware_tested=False)
    (parent/'comparison.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record),flush=True)
    assert passing,'No passing setup/hold candidate'
if __name__=='__main__':main()
