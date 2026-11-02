"""Check both read-pipeline stages using directed routed setup reports.

This checks data setup only; it does not waive pulse-width or clock routing
violations, and does not replace the full-system timing signoff.
"""
import argparse,json,re
from pathlib import Path
from report_route_trial import summarize


def audit(folder):
    timing=summarize(folder)
    stages={}
    for label,marker,expected in [('bank_output','storage_bank*.read_pixel_pipe',64),
                                  ('cached_word','camera/stream/cached_word',16)]:
        tables=[t for t in timing['setup_reports'] if '-to [get_pins' in t['command'] and marker in t['command']]
        assert len(tables)==1,('Missing/ambiguous directed timing table',label)
        paths=tables[0]['paths'];sinks={p['sink'] for p in paths}
        assert paths and all(p['source_clock'].startswith('eth125:') and
                             p['destination_clock'].startswith('eth125:') for p in paths)
        if label=='bank_output':
            bits={tuple(map(int,re.search(r'storage_bank\[(\d+)\]\.read_pixel_pipe_(\d+)_s\d+/D$',p).groups()))
                  for p in sinks}
            coverage=bits=={(b,i) for b in range(4) for i in range(16)}
        else:
            bits={int(re.search(r'cached_word_(\d+)_s\d+/D$',p)[1]) for p in sinks}
            coverage=bits==set(range(16))
        worst=min(paths,key=lambda p:p['slack_ns'])
        stages[label]=dict(coverage_passed=coverage,expected_endpoints=expected,reported_endpoints=len(sinks),
            reported_paths=len(paths),worst_path=worst,
            worst_slack_ns=worst['slack_ns'],max_data_delay_ns=max(p['data_delay_ns'] for p in paths),
            passed=coverage and all(p['slack_ns']>=0 for p in paths))
    result=dict(scope='Only eth125 setup of bank output registers and cached_word',
                passed=all(s['passed'] for s in stages.values()),stages=stages,
                full_system_timing_signoff=False,
                min_pulse_width_slack_ns=timing['min_pulse_width_slack_ns'])
    (folder/'frame_read_timing_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    result=audit(parser.parse_args().folder)
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
