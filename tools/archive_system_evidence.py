"""Freeze successful offline results only when they match current source bytes."""
from pathlib import Path
import argparse,hashlib,json,re,shutil
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'docs/evidence/system_v1');a=ap.parse_args()
    folder=a.output;folder.mkdir(parents=True,exist_ok=True)
    inputs={'build':ROOT/'build/system/result.json','tests':ROOT/'build/tests/system.json','full_top':ROOT/'build/tests/full_top_result.json'}
    for name,p in inputs.items():
        record=json.loads(p.read_text(encoding='utf-8'))
        if not record.get('passed'):raise SystemExit(name+' has not passed')
        for rel,digest in record['source_sha256'].items():
            if sha(ROOT/rel)!=digest:raise SystemExit(name+' stale source: '+rel)
        if name=='build':
            timing=ROOT/'build/system/impl/pnr/system_tr_content.html';bit=ROOT/'build/system/impl/pnr/system.fs';net=ROOT/'build/system/impl/gwsynthesis/system.vg'
            if record['violations']!={'setup':0,'hold':0} or sha(timing)!=record['timing_sha256'] or sha(bit)!=record['bitstream_sha256']:raise SystemExit('Build artifact mismatch')
            text=net.read_text(encoding='utf-8',errors='replace')
            grounded={}
            for instance,pin in [('legacy_m1_disable','legacy_m1_disable'),('legacy_m2_disable','legacy_m2_disable')]+[(name+'_'+str(bit),name+'['+str(bit)+']') for name,width in [('legacy_m1_pwm_disabled',6),('legacy_m2_pwm_disabled',6),('legacy_cal_disabled',2)] for bit in range(width)]:
                grounded[pin]=bool(re.search(r'OBUF\s+'+re.escape(instance)+r'_obuf\s*\(\s*\.O\('+re.escape(pin)+r'\),\s*\.I\(GND\)',text))
            record['legacy_outputs_grounded']=grounded
            if not all(record['legacy_outputs_grounded'].values()):raise SystemExit('Legacy gate not grounded')
            record['netlist_sha256']=sha(net)
        (folder/(name+'.json')).write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    for name in ('pin_audit.json','system.cst','system.sdc','build.tcl'):shutil.copyfile(ROOT/'build/system'/name,folder/name)
    report=(ROOT/'build/system/impl/pnr/system.rpt.txt').read_text(encoding='utf-8',errors='replace')
    (folder/'pnr_report.txt').write_text('\n'.join(x.rstrip() for x in report.splitlines()).rstrip()+'\n',encoding='utf-8')
    shutil.copyfile(ROOT/'build/tests/lidar_replay.json',folder/'lidar_replay.json')
    print('PASS current source/build/test identities; archived',folder)
if __name__=='__main__':main()
