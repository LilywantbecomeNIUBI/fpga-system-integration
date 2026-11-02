"""Frozen 1kHz wheel candidate, with explicit legacy profile. Never programs.

Examples:
  python tools/corridor/build_wheel.py --profile motor --mode pnr
  python tools/corridor/build_wheel.py --profile task4 --mode syn
  python tools/corridor/build_wheel.py --profile task4 --mode prepare --legacy
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/task4'))
from build import base_constraints, m2_rows, sources as task4_sources


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(profile, legacy=False):
    stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '_' + str(time.time_ns())[-6:]
    folder = ROOT / 'build/corridor' / (profile + ('_100hz' if legacy else '_1khz')) / stamp
    folder.mkdir(parents=True)
    paths = list((ROOT / 'fpga/motor').glob('*.sv'))
    paths += [ROOT / 'fpga/system/imu_status_source.sv']
    paths += [ROOT / 'fpga/imu' / n for n in ('imu_1khz_core.v', 'mpu6050_reader.v', 'imu_fast_i2c_master.v')]
    rows = list(csv.DictReader((ROOT / 'hardware/pins.csv').open(encoding='utf-8-sig')))
    top_path = ROOT / 'fpga/motor/brushed_motor_top.sv'
    if profile == 'task4':
        paths = task4_sources()
        rows = m2_rows(rows)
        top_path = ROOT / 'fpga/navigation/system_task4_top.sv'
    elif profile == 'system':
        paths = task4_sources() + list((ROOT / 'fpga/control').glob('*.sv')) + [ROOT / 'fpga/system/system_top.sv']
        top_path = ROOT / 'fpga/system/system_top.sv'
    else:
        rows = [r for r in rows if r['group'] not in ('camera', 'ethernet')]
    hashes = {}
    frozen_hashes = {}
    for path in sorted(set(paths)):
        name = path.relative_to(ROOT).as_posix()
        hashes[name] = sha(path)
        dest = folder / 'source' / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, dest)
        if path == top_path:
            content = dest.read_text(encoding='utf-8')
            assert content.count('WHEEL_1KHZ=0') == 1
            dest.write_text(content.replace('WHEEL_1KHZ=0', 'WHEEL_1KHZ=' + str(int(not legacy))), encoding='utf-8')
        frozen_hashes[name] = sha(dest)
    cst, sdc = base_constraints(rows)
    if profile == 'motor':
        cst = '\n'.join(line for line in cst.splitlines() if not line.startswith('CLOCK_LOC')) + '\n'
        sdc = 'create_clock -name sys50 -period 20 [get_ports {clk50m}]\n'
        for pin in ('rx/rx_meta_s0/D', 'core/enc_l/ab_meta_0_s0/D', 'core/enc_l/ab_meta_1_s0/D',
                    'core/enc_r/ab_meta_0_s0/D', 'core/enc_r/ab_meta_1_s0/D'):
            sdc += f'set_false_path -to [get_pins {{{pin}}}]\n'
    elif profile == 'task4':
        sdc += 'set_false_path -to [get_pins {lidar_uart/rx_meta_s0/D}]\n'
    (folder / 'wheel.cst').write_text(cst, encoding='utf-8')
    (folder / 'wheel.sdc').write_text(sdc, encoding='utf-8')
    project = ET.Element('Project')
    ET.SubElement(project, 'Template').text = 'FPGA'
    ET.SubElement(project, 'Version').text = '5'
    ET.SubElement(project, 'Device', name='GW5AT-138B', pn='GW5AT-LV138PG484AC1/I0').text = 'gw5at138b-012'
    filelist = ET.SubElement(project, 'FileList')
    for path in sorted((folder / 'source').rglob('*')) + [folder / 'wheel.cst', folder / 'wheel.sdc']:
        if path.is_file() and path.suffix in ('.sv', '.v', '.cst', '.sdc'):
            ET.SubElement(filelist, 'File', path=path.relative_to(folder).as_posix(),
                          type={'.v': 'file.verilog', '.sv': 'file.verilog', '.cst': 'file.cst', '.sdc': 'file.sdc'}[path.suffix], enable='1')
    ET.ElementTree(project).write(folder / 'wheel.gprj', encoding='utf-8', xml_declaration=True)
    manifest = dict(profile=profile, top=top_path.stem, wheel_control_hz=100 if legacy else 1000,
                    legacy=legacy, source_sha256=hashes, frozen_sha256=frozen_hashes,
                    constraints_sha256={name: sha(folder / name) for name in ('wheel.cst', 'wheel.sdc')},
                    build_tool_sha256=sha(Path(__file__)), programmed=False, hardware_tested=False)
    (folder / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    assert all(sha(ROOT / name) == digest for name, digest in hashes.items())
    return folder, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('motor', 'system', 'task4'), default='motor')
    parser.add_argument('--mode', choices=('prepare', 'syn', 'pnr'), default='prepare')
    parser.add_argument('--legacy', action='store_true')
    parser.add_argument('--timeout', type=int, default=1200)
    args = parser.parse_args()
    folder, record = prepare(args.profile, args.legacy)
    tcl = 'set here [file dirname [file normalize [info script]]]\ncd $here\nopen_project wheel.gprj\n'
    for key, value in (('top_module', record['top']), ('verilog_std', 'sysv2017'), ('output_base_name', 'wheel'),
                       ('max_fanout', '32'), ('replicate_resources', '1'), ('timing_driven', '1'),
                       ('retiming_resource', 'bsram'), ('use_sspi_as_gpio', '1'), ('use_cpu_as_gpio', '1'),
                       ('place_option', '2'), ('route_option', '1')):
        tcl += f'set_option -{key} {value}\n'
    (folder / 'build.tcl').write_text(tcl + ('run all\n' if args.mode == 'pnr' else 'run syn\n') + 'exit\n', encoding='utf-8')
    print(folder, flush=True)
    if args.mode == 'prepare':
        return
    if os.name == 'nt':
        active = subprocess.run(['powershell', '-NoProfile', '-Command',
            'Get-Process gw_sh,gw_syn,gw_pnr -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id'], capture_output=True, text=True)
        if active.stdout.strip():
            raise SystemExit('Another Gowin build is running; frozen candidate prepared, other processes untouched.')
    exe = Path(os.environ.get('GOWIN_HOME', 'C:/Gowin/Gowin_V1.9.12.04_x64')) / 'IDE/bin/gw_sh.exe'
    start = time.monotonic()
    completed_at = None
    cleaned_up = False
    timed_out = False
    with (folder / 'build.log').open('w', encoding='utf-8') as log:
        proc = subprocess.Popen([str(exe), str(folder / 'build.tcl')], cwd=folder, stdout=log, stderr=subprocess.STDOUT)
        while proc.poll() is None:
            time.sleep(1)
            text = (folder / 'build.log').read_text(encoding='utf-8', errors='replace')
            complete = ('[100%] Power analysis completed' in text and (folder / 'impl/pnr/wheel.fs').exists()) if args.mode == 'pnr' else ('GowinSynthesis finish' in text and (folder / 'impl/gwsynthesis/wheel.vg').exists())
            if complete and completed_at is None:
                completed_at = time.monotonic()
            if completed_at is not None and time.monotonic() - completed_at > 30:
                proc.terminate()
                proc.wait(timeout=15)
                cleaned_up = True
                break
            if time.monotonic() - start > args.timeout:
                proc.terminate()
                proc.wait(timeout=15)
                timed_out = True
                break
    text = (folder / 'build.log').read_text(encoding='utf-8', errors='replace')
    record.update(mode=args.mode, exit_code=proc.returncode, elapsed_s=time.monotonic() - start,
                  cli_terminated_after_completed_outputs=cleaned_up, timed_out=timed_out,
                  log_sha256=sha(folder / 'build.log'), tool_sha256=sha(exe))
    record['sources_unchanged'] = all(sha(ROOT / name) == digest for name, digest in record['source_sha256'].items())
    record['frozen_unchanged'] = all(sha(folder / 'source' / name) == digest for name, digest in record['frozen_sha256'].items())
    record['synthesis_passed'] = 'GowinSynthesis finish' in text and (folder / 'impl/gwsynthesis/wheel.vg').exists() and not re.search(r'^ERROR\s*\(', text, re.M)
    if args.mode == 'pnr' and (folder / 'impl/pnr/wheel_tr_content.html').exists():
        timing = (folder / 'impl/pnr/wheel_tr_content.html').read_text(encoding='utf-8', errors='replace')
        record['violations'] = {kind.lower(): int(m[1]) if (m := re.search(r'Numbers of ' + kind + r' Violated Endpoints</td>\s*<td[^>]*>(\d+)', timing)) else None for kind in ('Setup', 'Hold')}
        trace = (folder / 'impl/pnr/wheel.tr').read_text(encoding='utf-8', errors='replace')
        pulse = trace.split('3.2 Minimum Pulse Width Table')[-1].split('3.3 Timing Report By Analysis Type')[0]
        widths = [float(m[1]) for m in re.finditer(r'^\s+\d+\s+([-0-9.]+)\s+[-0-9.]+\s+[-0-9.]+\s+(?:High|Low) Pulse Width', pulse, re.M)]
        record['minimum_pulse_width_slack_ns'] = min(widths) if widths else None
        record['clock_route_warnings'] = [line for line in text.splitlines() if 'PR1014' in line]
        record['timing_passed'] = record['violations'] == dict(setup=0, hold=0) and bool(widths) and min(widths) >= 0 and not record['clock_route_warnings']
        if (folder / 'impl/pnr/wheel.fs').exists():
            record['bitstream_sha256'] = sha(folder / 'impl/pnr/wheel.fs')
    record['passed'] = (proc.returncode == 0 or cleaned_up) and not timed_out and record['synthesis_passed'] and record['sources_unchanged'] and record['frozen_unchanged']
    if args.mode == 'pnr':
        record['passed'] = record['passed'] and record.get('timing_passed', False) and 'bitstream_sha256' in record
    # Full camera-clock/CDC signoff still uses the existing independent auditors.
    record['full_system_hardware_signed_off'] = False
    (folder / 'result.json').write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in record.items() if not k.endswith('sha256')}, indent=2), flush=True)
    raise SystemExit(0 if record['passed'] else 1)


if __name__ == '__main__':
    main()
