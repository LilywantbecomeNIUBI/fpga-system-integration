"""Report routed raw/Mecanum closure and retry margin without waiving MPW."""
from pathlib import Path
import argparse,hashlib,json,re,sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/task4'))
from report_route_trial import summarize


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path)
    ap.add_argument('--functional-build',type=Path,required=True)
    args=ap.parse_args();folder=args.build
    meta=json.loads((folder/'manifest.json').read_text())
    result=json.loads((folder/'result.json').read_text())
    tests=json.loads((args.functional_build/'system_test_result.json').read_text())
    assert meta['top']=='system_mecanum_top' and meta['lidar_mode']=='raw_points'
    assert meta['pin_profile']=='mecanum_p7'
    assert tests['passed'] and tests['raw_points_checked']==12000
    assert tests['source_sha256']==meta['source_sha256'],'Functional test used different RTL'
    routes=summarize(folder)
    # A directed report must really resolve counter endpoints; absence is not
    # timing closure. Other reports may repeat the same physical path.
    retry_reports=[r for r in routes['setup_reports'] if '/retry_count*/*' in r['command']]
    assert retry_reports and retry_reports[0]['paths'],'Retry report did not resolve real pins'
    retry=retry_reports[0]['paths']
    raw_reports=[r for r in routes['setup_reports'] if 'ethernet_raw.lidar_display' in r['command']]
    raw_paths=[p for report in raw_reports for p in report['paths']]
    assert raw_paths,'Raw RAM/readout reports did not resolve real pins'
    worst_retry=min(retry,key=lambda p:p['slack_ns'])
    former=[p for p in retry if re.search(r'/retry_count2_4_s\d+/CE$',p['sink'])]
    frozen=all(hashlib.sha256((folder/'source'/p).read_bytes()).hexdigest()==h
               for p,h in meta['source_sha256'].items())
    report=dict(setup_hold_passed=result['violations']==dict(setup=0,hold=0),
        violations=result['violations'],clock_endpoints=routes['clock_endpoints'],
        baseline_retry_margin_ns=.001,worst_retry_path=worst_retry,
        former_critical_sink_path=min(former,key=lambda p:p['slack_ns']) if former else None,
        worst_raw_readout_path=min(raw_paths,key=lambda p:p['slack_ns']),
        retry_margin_hardened=worst_retry['slack_ns']>.001,
        advisory_100ps_guard_band_met=worst_retry['slack_ns']>=.100,
        raw_readout_setup_passed=all(p['slack_ns']>=0 for p in raw_paths),
        resources=routes['resources'],clock_allocation=routes['clock_allocation'],
        minimum_pulse_width_slack_ns=result['min_pulse_width_slack_ns'],
        clock_route_warnings=result['clock_route_warnings'],
        full_system_signoff_passed=result['passed'],
        functional_source_match=True,raw_points_checked=tests['raw_points_checked'],
        frozen_sources_unchanged=frozen,programmed=False,hardware_tested=False,
        bitstream_sha256=result.get('bitstream_sha256'))
    report['requested_setup_hold_and_margin_passed']=(report['setup_hold_passed'] and
        report['retry_margin_hardened'] and report['raw_readout_setup_passed'] and frozen)
    (folder/'raw_timing_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['requested_setup_hold_and_margin_passed'] else 1)


if __name__=='__main__':main()
