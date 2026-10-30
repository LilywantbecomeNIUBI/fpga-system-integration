"""Archive current offline merge evidence, preserving failed timing explicitly."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import html
import json
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'docs/evidence/perception_merge_20261030'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    records = {}
    for name, relative in (('build', 'build/system/result.json'),
                           ('system_tests', 'build/tests/system.json'),
                           ('full_top', 'build/tests/full_top_result.json'),
                           ('perception', 'build/tests/perception_result.json')):
        record = json.loads((ROOT/relative).read_text(encoding='utf-8'))
        if name != 'build' and not record.get('passed'):
            raise SystemExit(name+' has not passed')
        for path, digest in record['source_sha256'].items():
            if sha(ROOT/path) != digest:
                raise SystemExit(name+' stale source: '+path)
        records[name] = record
        (OUT/(name+'.json')).write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    for name in ('system.cst', 'system.sdc', 'build.tcl', 'pin_audit.json'):
        shutil.copyfile(ROOT/'build/system'/name, OUT/name)
    timing = ROOT/'build/system/impl/pnr/system_tr_content.html'
    bitstream = ROOT/'build/system/impl/pnr/system.fs'
    if sha(timing) != records['build']['timing_sha256'] or sha(bitstream) != records['build']['bitstream_sha256']:
        raise SystemExit('Current build artifact identity mismatch')
    content = timing.read_text(encoding='utf-8', errors='replace')
    pulse_section = content.split('name="MIN_PULSE_WIDTH_TABLE"', 1)[1].split('</table>', 1)[0]
    pulses = []
    for row in re.findall(r'<tr>(.*?)</tr>', pulse_section, re.S):
        cells = [html.unescape(re.sub(r'<[^>]+>', '', cell)).strip()
                 for cell in re.findall(r'<td[^>]*>(.*?)</td>', row, re.S)]
        if len(cells) == 7:
            pulses.append(dict(slack_ns=float(cells[1]), actual_ns=float(cells[2]),
                               required_ns=float(cells[3]), kind=cells[4], clock=cells[5], object=cells[6]))
    netlist = ROOT/'build/system/impl/gwsynthesis/system.vg'
    text = netlist.read_text(encoding='utf-8', errors='replace')
    pins = ['legacy_m1_disable', 'legacy_m2_disable']
    pins += [name+'['+str(bit)+']' for name, width in
             (('legacy_m1_pwm_disabled', 6), ('legacy_m2_pwm_disabled', 6), ('legacy_cal_disabled', 2))
             for bit in range(width)]
    grounded = {pin:bool(re.search(r'OBUF\s+\S+\s*\(\s*\.O\('+re.escape(pin)+r'\),\s*\.I\(GND\)', text)) for pin in pins}
    if not all(grounded.values()):
        raise SystemExit('Legacy outputs not grounded')
    report=(ROOT/'build/system/impl/pnr/system.rpt.txt').read_text(encoding='utf-8',errors='replace')
    (OUT/'pnr_report.txt').write_text('\n'.join(line.rstrip() for line in report.splitlines()).rstrip()+'\n',encoding='utf-8')
    shutil.copyfile(timing, OUT/'timing.html')
    attempts = []
    for folder in sorted((ROOT/'build/system').glob('attempt_20261030*')):
        path = folder/'result.json'
        if path.exists():
            item = json.loads(path.read_text(encoding='utf-8'))
            attempts.append({key:item.get(key) for key in
                             ('started_utc', 'violations', 'passed', 'options', 'bitstream_sha256', 'timing_sha256')})
    checks = {}
    for name in ('perception_ui', 'ground_ui', 'motor_tuner_ui', 'imu_ui'):
        log = ROOT/'build/tests'/(name+'.log')
        if not log.exists() or 'PASS' not in log.read_text(encoding='utf-8'):
            raise SystemExit(name+' result missing')
        shutil.copyfile(log, OUT/(name+'.txt'))
        checks[name] = dict(passed=True, log_sha256=sha(log))
    negative_pulses = [p for p in pulses if p['slack_ns'] < 0]
    summary = dict(created_utc=datetime.now(timezone.utc).isoformat(),
        vision_commit='df6f60d07873150d9ea796580adbc17960945f2c',
        merge_mode='observe_only', automatic_motion_connected=False,
        programmed=False, nano_deployed=False, hardware_tested=False,
        regression_passed=True, setup_hold_passed=records['build']['passed'],
        timing_signoff=bool(records['build']['passed'] and not negative_pulses),
        reported_min_pulse_width=pulses, build_attempts=attempts, ui_checks=checks,
        netlist_sha256=sha(netlist), legacy_outputs_grounded=grounded,
        limits=['Digital models and accelerated UART/motor fixture; not physical acceptance',
                'Full-top fixture injects reduced video; concurrent real-camera throughput remains untested',
                'IMU body-axis mapping and balance controller are outside this merge',
                'Board IO/PVT margins and reported pulse-width checks require separate signoff'])
    (OUT/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    print('Archived current-source merge evidence:', OUT)
    print('Setup/hold:', records['build']['violations'], 'negative reported pulse widths:', len(negative_pulses))


if __name__ == '__main__':
    main()
