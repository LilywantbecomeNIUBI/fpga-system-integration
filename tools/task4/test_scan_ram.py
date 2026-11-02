"""Exercise scan RAM ownership, burst input, nearest returns and packet expiry."""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    out = ROOT / 'build/lidar_scan_ram_20261102'
    out.mkdir(parents=True, exist_ok=True)
    sources = ['tests/navigation/tb_lidar_scan_ram.sv',
               'fpga/navigation/lidar_scan_source.sv', 'fpga/motor/frame_transport.sv']
    commands = [
        ['iverilog', '-g2012', '-Wall', '-s', 'tb_lidar_scan_ram', '-o',
         str(out / 'scan_ram.vvp'), *sources],
        ['vvp', str(out / 'scan_ram.vvp')],
    ]
    for name, command in zip(('compile', 'regression'), commands):
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=60)
        (out / (name + '.log')).write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        if name == 'regression':
            print(result.stdout, end='')
    record = dict(passed=True, hardware_tested=False, programmed=False,
                  source_sha256={p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
                                 for p in sources})
    (out / 'regression.json').write_text(json.dumps(record, indent=2) + '\n')


if __name__ == '__main__':
    main()
