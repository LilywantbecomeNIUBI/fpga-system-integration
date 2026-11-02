"""Run the full M2 production hierarchy with the opt-in 1kHz wheel profile."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/corridor/integration'
sys.path[:0] = [str(ROOT), str(ROOT / 'tools/task4')]
from build import sources
from test_display import packet
from shared.streams import MixedStream
from shared.lidar_display import FpgaLidar


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    original = (ROOT / 'tests/navigation/tb_task4_system.sv').read_text(encoding='utf-8')
    assert original.count('system_task4_top #(') == 1
    bench = OUT / 'tb_task4_system.sv'
    bench.write_text(original.replace('system_task4_top #(', 'system_task4_top #(.WHEEL_1KHZ(1),')
                     .replace('build/task4/', 'build/corridor/integration/'), encoding='utf-8')
    damaged = bytearray(packet(0))
    damaged[-1] ^= 1
    data = bytes(damaged) + b''.join(packet(i * 1000) for _ in range(3) for i in range(36))
    (OUT / 'lidar_display_input.hex').write_text('\n'.join('%02x' % b for b in data), encoding='ascii')
    paths = sources()
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    # Compile a frozen copy, so concurrent work cannot alter this simulation.
    files = []
    for path in paths:
        dest = OUT / 'source' / path.relative_to(ROOT)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, dest)
        assert hashlib.sha256(dest.read_bytes()).hexdigest() == hashes[path.relative_to(ROOT).as_posix()]
        if path.name != 'ethernet_pll.v':
            files.append(str(dest))
    prim = Path(os.environ.get('GOWIN_HOME', 'C:/Gowin/Gowin_V1.9.12.04_x64')) / 'IDE/simlib/gw5a/prim_sim.v'
    commands = [
        ['iverilog', '-g2012', '-s', 'tb_task4_system', '-s', 'GSR', '-o', str(OUT / 'full.vvp'),
         str(bench), 'tests/imu/mpu6050_model.sv', 'tests/ethernet_pll_model.v', str(prim)] + files,
        ['vvp', str(OUT / 'full.vvp')],
    ]
    for name, command in zip(('compile', 'full'), commands):
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=600)
        text = result.stdout + result.stderr
        (OUT / (name + '.log')).write_text(text, encoding='utf-8')
        if result.returncode or (name == 'full' and ('PASS' not in text or 'FATAL' in text)):
            raise RuntimeError(text[-4000:])
        print(text[-1000:], flush=True)
    parser = MixedStream()
    rows = parser.feed((OUT / 'full_system_uart.bin').read_bytes())
    assert not parser.bad_crc
    assert {'motor', 'imu', 'encoder_diagnostics', 'encoder_trace'} <= {kind for kind, row in rows}
    display = FpgaLidar()
    for kind, row in rows:
        display.ingest(kind, row, now=1)
    assert display.scan and len(display.scan['points']) == 72
    assert all(p == [i * 5 + 2.5, 1000 + i, 100] for i, p in enumerate(display.scan['points']))
    assert display.diagnostic and display.diagnostic['packets'] == 108 and display.diagnostic['crc_errors'] == 1
    assert all(hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest for name, digest in hashes.items())
    (OUT / 'result.json').write_text(json.dumps(dict(passed=True, wheel_control_hz=1000,
        hardware_tested=False, programmed=False, source_sha256=hashes,
        decoded_records=len(rows), bad_crc=0, scan_sectors=72), indent=2) + '\n', encoding='utf-8')
    print('PASS 1kHz production hierarchy with camera/network/MPU6050/radar and mixed UART', len(rows))


if __name__ == '__main__':
    main()
