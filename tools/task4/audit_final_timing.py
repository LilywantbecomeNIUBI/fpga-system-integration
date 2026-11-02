"""Keep Setup/Hold closure separate from full clock/pulse-width signoff."""
import argparse,hashlib,json,re
from pathlib import Path
from report_route_trial import summarize

def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);ap.add_argument('--baseline',type=Path,
        default=Path('build/lidar_ethernet_20261102/system/20261102T172152Z'));a=ap.parse_args()
    trace=(a.build/'impl/pnr/sensor.tr').read_text(errors='replace')
    timing=summarize(a.build);meta=json.loads((a.build/'result.json').read_text())
    tables=trace.rsplit('3.1.3 Recovery Paths Table',1)[1].split('3.1.4 Removal Paths Table')[0]
    rows=[]
    for m in re.finditer(r'^\s+\d+\s+([-\d.]+)\s+(\S+)\s+(\S+)\s+(\S+):',tables,re.M):
        rows.append(dict(slack_ns=float(m[1]),source=m[2],sink=m[3],clock=m[4]))
    reset=json.loads((a.build/'reset_distribution_audit.json').read_text())
    unchanged={kind:(a.build/('sensor.'+kind)).read_bytes()==(a.baseline/('sensor.'+kind)).read_bytes() for kind in ('sdc','cst')}
    manifest=json.loads((a.build/'manifest.json').read_text());base=json.loads((a.baseline/'manifest.json').read_text())
    changed=[p for p,h in manifest['source_sha256'].items() if base['source_sha256'].get(p)!=h]
    global_table=next(t for t in timing['setup_reports'] if t['command'].startswith('report_timing -setup -max_paths'))
    result=dict(setup_hold_closure_passed=meta['violations']==dict(setup=0,hold=0),
        full_system_signoff_passed=meta['passed'],violations=meta['violations'],
        unchanged_constraints=unchanged,changed_rtl=changed,
        worst_setup_path=min(global_table['paths'],key=lambda p:p['slack_ns']),
        worst_reported_recovery=min(rows,key=lambda p:p['slack_ns']) if rows else None,
        worst_perception_recovery=min((r for r in rows if '/perception_reset/' in r['source']),key=lambda p:p['slack_ns']),
        minimum_pulse_width_slack_ns=meta['min_pulse_width_slack_ns'],
        clock_route_warnings=meta['clock_route_warnings'],reset_maximum_leaf_fanout=reset['maximum_fanout'],
        bitstream_sha256=meta['bitstream_sha256'],frozen_sources_unchanged=meta['frozen_sources_unchanged'],
        programmed=False)
    result['passed']=result['setup_hold_closure_passed'] and all(unchanged.values()) and reset['passed'] and meta['frozen_sources_unchanged']
    (a.build/'final_timing_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
if __name__=='__main__':main()
