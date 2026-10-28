"""Archive successful offline evidence, rejecting stale build/test outputs."""
from pathlib import Path
import hashlib
import json
import re

ROOT=Path(__file__).resolve().parents[1]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    build=json.loads((ROOT/'build/motor/result.json').read_text(encoding='utf-8'))
    simulation=json.loads((ROOT/'build/tests/result.json').read_text(encoding='utf-8'))
    if not build['passed'] or not all(c['passed'] for c in simulation['cases']):raise SystemExit('Build/tests did not pass')
    for record in (build,simulation):
        for path,digest in record['source_sha256'].items():
            if sha(ROOT/path)!=digest:raise SystemExit('Stale source: '+path)
    folder=ROOT/'docs/evidence/motor_v1';folder.mkdir(parents=True,exist_ok=True)
    net=ROOT/'build/motor/impl/gwsynthesis/motor.vg'
    text=net.read_text(encoding='utf-8',errors='replace')
    grounded={pin:bool(re.search(r'OBUF\s+'+pin+r'_obuf\s*\(\s*\.O\('+pin+r'\),\s*\.I\(GND\)',text)) for pin in ('legacy_m1_disable','legacy_m2_disable')}
    if not all(grounded.values()):raise SystemExit('Legacy disable output not grounded')
    timing=ROOT/'build/motor/impl/pnr/motor_tr_content.html'
    timing_text=re.sub(r'\s+',' ',re.sub('<[^>]+>',' ',timing.read_text(encoding='utf-8',errors='replace')))
    fmax=re.search(r'Actual Fmax.*?50\.000\(MHz\) ([\d.]+)\(MHz\)',timing_text)
    if sha(timing)!=build['timing_sha256']:raise SystemExit('Timing file hash mismatch')
    bit=ROOT/'build/motor/impl/pnr/motor.fs'
    if sha(bit)!=build['bitstream_sha256']:raise SystemExit('Bitstream hash mismatch')
    build.update(legacy_outputs_grounded=grounded,netlist_sha256=sha(net),fmax_mhz=float(fmax[1]) if fmax else None)
    for name,data in [('build.json',build),('simulation.json',simulation)]:
        (folder/name).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    report=(ROOT/'build/motor/impl/pnr/motor.rpt.txt').read_text(encoding='utf-8',errors='replace')
    (folder/'pnr_report.txt').write_text('\n'.join(line.rstrip() for line in report.splitlines()).rstrip()+'\n',encoding='utf-8')
    print(json.dumps({k:build[k] for k in ('started_utc','violations','fmax_mhz','bitstream_sha256','legacy_outputs_grounded')},indent=2))

if __name__=='__main__':main()
