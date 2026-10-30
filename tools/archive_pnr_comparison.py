"""Verify and archive regressions alongside a completed PnR comparison."""
from pathlib import Path
import hashlib,json,re,shutil,subprocess
from build_system import ROOT,sha

def main():
    latest=json.loads((ROOT/'build/latest_pnr_comparison.json').read_text())
    evidence=Path(latest['evidence']);evidence.resolve().relative_to(ROOT/'docs/evidence')
    summary_path=evidence/'summary.json';s=json.loads(summary_path.read_text())
    assert all(sha(ROOT/p)==h for p,h in s['source_sha256'].items())
    bit=ROOT/s['bitstream_path'];assert sha(bit)==s['selected']['bitstream_sha256']
    regression={}
    identities=dict(s['source_sha256'])
    for name in ('system.json','perception_result.json','full_top_result.json'):
        source=ROOT/'build/tests'/name;r=json.loads(source.read_text());assert r['passed']
        mismatches=[p for p,h in r['source_sha256'].items() if sha(ROOT/p)!=h]
        assert not mismatches,(name,mismatches)
        identities.update(r['source_sha256'])
        shutil.copyfile(source,evidence/name);regression[name]=dict(passed=True,sha256=sha(source),sources_match=True)
    for name in ('unit_and_motor','lk','dual','full_top'):
        shutil.copyfile(ROOT/'build/tests'/(name+'.log'),evidence/(name+'.log'))
    for name in ('perception','ground','motor_tuner','imu'):
        test=ROOT/'tests'/('test_'+name+'_ui.js')
        r=subprocess.run(['node',str(test)],capture_output=True,text=True,check=True)
        (evidence/(name+'_ui.log')).write_text(r.stdout+r.stderr)
        regression[name+'_ui']=dict(passed=True,test_sha256=sha(test))
        identities[test.relative_to(ROOT).as_posix()]=sha(test)
    for rel,h in identities.items():
        assert sha(ROOT/rel)==h,rel
        dest=evidence/'sources'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dest)
    net=(ROOT/'build/system/impl/gwsynthesis/system.vg').read_text()
    assert sha(ROOT/'build/system/impl/gwsynthesis/system.vg')==s['selected']['netlist_sha256']
    reset_audit={}
    for name,prefixes in [('flow_event_bridge',('slot0','slot1','latest','vector')),('vision_v23_bridge',('latest','event_payload')),('event_packet_mux',('out_payload',))]:
        module=net.split('module '+name+' (')[1].split('endmodule')[0]
        cells=[(t,n) for t,n in re.findall(r'\b(DFF\w*)\s+(\w+)\s*\(',module) if any(re.match(p+r'_\d+_',n) for p in prefixes)]
        assert cells and not any(t in ('DFFC','DFFCE','DFFP','DFFPE') for t,n in cells)
        reset_audit[name]=dict(data_register_count=len(cells),primitive_types=sorted(set(t for t,n in cells)),asynchronous_clear_data_registers=[])
    (evidence/'reset_audit.json').write_text(json.dumps(reset_audit,indent=2)+'\n')
    s['source_snapshot_sha256']=identities
    s['tool_sha256']=sha(Path('C:/Gowin/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe'))
    best=s['selected'];case=evidence/('place_'+str(best['place_option']))
    cst=(case/'system.cst').read_text();sdc=(case/'system.sdc').read_text()
    assert 'IO_LOC "camera_pclk" B1;' in cst and 'period 11.9047' in sdc
    assert 'get_pins {camera/pll/PLL_inst/CLKOUT1}' in sdc and 'create_generated_clock -name eth125' in sdc
    assert all("clock signal 'camera_pclk_d'" in w for w in best['clock_route_warnings'])
    s['waiver_scope_verified']=True;s['regressions']=regression
    (evidence/'waivers.json').write_text(json.dumps(s['waivers'],indent=2)+'\n')
    shutil.copyfile(ROOT/'config/pnr.json',evidence/'selected_profile.json')
    s['artifact_sha256']={p.relative_to(evidence).as_posix():sha(p) for p in evidence.rglob('*') if p.is_file() and p.name!='summary.json'}
    summary_path.write_text(json.dumps(s,indent=2)+'\n')
    (ROOT/'build/system/SELECTED_CANDIDATE.json').write_text(json.dumps(dict(bitstream=str(bit),sha256=sha(bit),evidence=str(evidence),note='impl/pnr contains the last comparison, not necessarily the selected candidate. Use this explicit identity.'),indent=2)+'\n')
    rows=['| Place | Setup | Hold | Worst Setup (ns) |','|---|---:|---:|---:|']
    rows += ['| %d | %d | %d | %+.3f |'%(r['place_option'],r['setup'],r['hold'],r['wns_ns']) for r in s['results']]
    doc=ROOT/'docs/PNR_OPTIMIZATION_20261030.md';text=doc.read_text(encoding='utf-8')
    text+='\n## 最终结果\n\n'+'\n'.join(rows)+'\n\n'
    text+='所选策略Place %d / Route 1，最差Setup %+.3fns；MPW仍为%+.3fns，整体timing_signoff=false。\n'%(best['place_option'],best['wns_ns'],best['min_pulse_slack_ns'])
    text+='位流SHA256：`'+best['bitstream_sha256']+'`。位流路径：`'+s['bitstream_path']+'`。\n'
    text+='全部数字回归通过；报告和源码/网表身份已核验。没有下载硬件。\n'
    doc.write_text(text,encoding='utf-8')
    print(json.dumps(dict(evidence=str(evidence),selected=best,regressions=regression,artifacts=len(s['artifact_sha256'])),indent=2))

if __name__=='__main__':main()
