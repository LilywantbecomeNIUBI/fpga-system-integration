"""Reproducible navigation kernel checks. No hardware or PnR access."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
OUT = ROOT/'build/mecanum_navigation'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    from tools.mecanum.navigation_model import sweep_vectors
    (OUT/'sweep_vectors.txt').write_text('\n'.join(' '.join(map(str, row)) for row in sweep_vectors())+'\n')
    # Synthetic test geometry only; never written into an enabled board config.
    sys.path.insert(0, str(ROOT/'tests'))
    from test_navigation import calibration
    from shared.navigation_protocol import packet, upload_operations
    operations = upload_operations(calibration(), [[200, 0, 50], [200, 200, 50]], 9)
    messages = [packet(sequence=i+1, **op) for i, op in enumerate(operations)]
    messages += [packet('start', 30, 9), packet('start', 31, 9), packet('heartbeat', 32, 9), packet('resume', 33, 9), packet('cancel', 34, 9)]
    (OUT/'mission.hex').write_text('\n'.join('%02x' % b for message in messages for b in message))
    sources = list((ROOT/'fpga/navigation').glob('mc_nav*.sv'))
    sources += [ROOT/'fpga/navigation'/n for n in ('nav_math.sv', 'nav_histogram.sv', 'nav_vfh.sv')]
    sources += [ROOT/'fpga/mecanum'/n for n in ('cordic_sincos.sv', 'frame_transport.sv', 'mecanum_protocol.sv','wheel_control_1khz.sv')]
    sources += [ROOT/'fpga/vision/src/vision/vision_mailbox.v', ROOT/'fpga/vision/src/reset_sync.v']
    sources += [ROOT/'tests/navigation/reference/mc_nav_system_pre_serial.sv']
    sources += [ROOT/'tests/navigation/reference/nav_math_full_reference.sv']
    sources += [ROOT/'tests/navigation/reference/nav_histogram_reference.sv']
    results = {}
    for bench in sorted((ROOT/'tests/navigation').glob('tb_mc_nav_*.sv')):
        executable = OUT/(bench.stem+'.vvp')
        started = time.monotonic()
        commands = [['iverilog', '-g2012', '-s', bench.stem, '-o', str(executable), str(bench)]+list(map(str, sources)), ['vvp', str(executable)]]
        log = ''
        for command in commands:
            result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=240)
            log += result.stdout+result.stderr
            if result.returncode:
                (OUT/(bench.stem+'.log')).write_text(log, encoding='utf-8')
                raise RuntimeError(log)
        (OUT/(bench.stem+'.log')).write_text(log, encoding='utf-8')
        results[bench.stem] = dict(passed=True, seconds=time.monotonic()-started,
                                  testbench_sha256=hashlib.sha256(bench.read_bytes()).hexdigest())
        print(log.strip(), flush=True)
    result = dict(passed=True, hardware_tested=False, programmed=False, tests=results,
                  source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  model_sha256=hashlib.sha256((ROOT/'tools/mecanum/navigation_model.py').read_bytes()).hexdigest())
    (OUT/'kernel_tests.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
