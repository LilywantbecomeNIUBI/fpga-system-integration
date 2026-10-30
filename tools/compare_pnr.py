"""Frozen-netlist Place 2/3/4 comparison. Offline builds only; never programs."""
from pathlib import Path
import argparse,hashlib,json,re,shutil,subprocess,sys
from datetime import datetime,timezone
from build_system import prepare,ROOT,sha
from gowin_runner import run_gowin

def main():
    folder,sources=prepare()
    ap=argparse.ArgumentParser();ap.add_argument('--resume');ap.add_argument('--places',nargs='+',type=int,choices=(2,3,4),default=[2,3,4]);args=ap.parse_args()
    stamp=args.resume or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out=ROOT/'build'/('pnr_compare_'+stamp);out.mkdir(exist_ok=True)
    evidence=ROOT/'docs/evidence'/('pnr_optimization_'+stamp);evidence.mkdir(parents=True,exist_ok=True)
    identity={p.relative_to(ROOT).as_posix():sha(p) for p in sources}
    identity_path=out/'source_identity.json'
    if identity_path.exists():assert json.loads(identity_path.read_text())==identity,'Resume source identity mismatch'
    else:identity_path.write_text(json.dumps(identity,indent=2)+'\n')
    exe='C:/Gowin/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe'
    base=(folder/'build.tcl').read_text().replace('run all','run syn')+'\nexit\n'
    script=folder/'compare.tcl';script.write_text(base)
    if not args.resume:
        run_gowin(exe,script,out/'synthesis.log')
    vg=folder/'impl/gwsynthesis/system.vg';vg_hash=sha(vg)
    net=vg.read_text(errors='replace')
    module=net.split('module flow_event_bridge (')[1].split('endmodule')[0]
    banks=re.findall(r'DFF\w*\s+(read_bank_\d+_\w+)\s*\(',module)
    assert set(re.search(r'read_bank_(\d+)_',b)[1] for b in banks)==set(map(str,range(8))),('read-bank groups not preserved',banks)
    # Wide slot and mux payload data must use ordinary (non-clear) FFs.
    data_cells=re.findall(r'\b(DFF\w*)\s+((?:slot[01]|out_payload|event_payload)_\w+)\s*\(',net)
    assert data_cells and not any('C' in cell for cell,name in data_cells if name.startswith(('slot0_','slot1_','out_payload_'))),data_cells[:5]
    audit=dict(netlist_sha256=vg_hash,read_bank_replicas=banks,data_cell_types=sorted(set(t for t,n in data_cells)))
    (evidence/'netlist_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    shutil.copyfile(out/'synthesis.log',evidence/'synthesis.log')
    results=[]
    for place in args.places:
        case=evidence/('place_'+str(place))
        if (case/'result.json').exists():
            saved=json.loads((case/'result.json').read_text());assert saved['netlist_sha256']==vg_hash
            results.append(saved);continue
        case.mkdir(exist_ok=True)
        tcl=re.sub(r'set_option -place_option \d+','set_option -place_option '+str(place),base).replace('run syn','run pnr')
        script.write_text(tcl)
        print('Starting Place',place,flush=True)
        tool_result=run_gowin(exe,script,case/'build.log',pnr=True)
        assert sha(vg)==vg_hash,'Synthesis netlist changed during comparison'
        assert all(sha(ROOT/p)==h for p,h in identity.items()),'RTL/constraints changed during build'
        pnr=folder/'impl/pnr';txt=(pnr/'system.tr').read_text(errors='replace')
        setup=int(re.search(r'<Numbers of Setup Violated Endpoints>:(\d+)',txt)[1])
        hold=int(re.search(r'<Numbers of Hold Violated Endpoints>:(\d+)',txt)[1])
        table=txt.split('3.1.1 Setup Paths Table')[-1].split('3.1.2 Hold Paths Table')[0]
        wns=float(re.search(r'^\s+1\s+([-\d.]+)',table,re.M)[1])
        pulse=[float(s) for s in re.findall(r'MPW Summary:\s+Slack:\s+([-\d.]+)',txt)]
        warnings=[s for s in (pnr/'system.log').read_text().splitlines() if 'PR1014' in s]
        row=dict(place_option=place,route_option=1,timing_driven=1,setup=setup,hold=hold,wns_ns=wns,
                 min_pulse_slack_ns=min(pulse),clock_route_warnings=warnings,netlist_sha256=vg_hash,bitstream_sha256=sha(pnr/'system.fs'))
        results.append(row)
        row['tool_result']=tool_result
        for source,name in [('system.tr','timing.txt'),('system_tr_content.html','timing.html'),('system.rpt.txt','pnr_report.txt'),('system.log','pnr.log'),('cmd.do','cmd.do')]:shutil.copyfile(pnr/source,case/name)
        shutil.copyfile(folder/'system.sdc',case/'system.sdc');shutil.copyfile(folder/'system.cst',case/'system.cst')
        (case/'result.json').write_text(json.dumps(row,indent=2)+'\n')
        best=max(results,key=lambda r:(r['setup']==0 and r['hold']==0,r['wns_ns']))
        if best is row:
            selected=out/'selected';selected.mkdir(exist_ok=True)
            shutil.copyfile(pnr/'system.fs',selected/'system.fs')
        print(json.dumps(row),flush=True)
    best=max(results,key=lambda r:(r['setup']==0 and r['hold']==0,r['wns_ns']))
    worker_bit=out/'bits'/str(best['place_option'])/'system.fs'
    if worker_bit.exists():
        assert sha(worker_bit)==best['bitstream_sha256']
        (out/'selected').mkdir(exist_ok=True);shutil.copyfile(worker_bit,out/'selected/system.fs')
    (ROOT/'config/pnr.json').write_text(json.dumps(dict(place_option=best['place_option'],route_option=1,timing_driven=1,evidence=evidence.relative_to(ROOT).as_posix()),indent=2)+'\n')
    waiver=dict(id='PR1014-B1',scope=dict(device='GW5AT-LV138PG484AC1/I0',port='camera_pclk',ball='B1',clock_period_ns=11.9047),
                status='accepted_configuration_exception',reason='Existing fixed B1 non-GCLK wiring; global spine still has generic ingress.',
                physical_issue_fixed=False,hardware_validation=False,authorization='User explicitly requested B1 limited-configuration waiver; raw warnings retained.')
    summary=dict(created_utc=stamp,comparison_complete=set(r['place_option'] for r in results)=={2,3,4},source_sha256=identity,netlist_audit=audit,results=results,selected=best,
                 bitstream_path=(out/'selected/system.fs').relative_to(ROOT).as_posix(),waivers=[waiver],
                 mpw_status='unresolved_not_waived',timing_signoff=False,programmed=False,hardware_tested=False)
    (evidence/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (ROOT/'build/latest_pnr_comparison.json').write_text(json.dumps(dict(evidence=str(evidence),build=str(out)),indent=2)+'\n')
    print('SELECTED',json.dumps(best),flush=True)

if __name__=='__main__':main()
