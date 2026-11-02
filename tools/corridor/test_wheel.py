"""Offline checks of the 1kHz wheel candidate. Never opens hardware."""
from pathlib import Path
import csv
import hashlib
import json
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/corridor/tests'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    sources = sorted((ROOT / 'fpga/motor').glob('*.sv'))
    cases = []
    benches = sorted((ROOT / 'tests/corridor').glob('tb_*.sv'))
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sources + benches}
    variants = [(bench, None) for bench in benches]
    variants.append((ROOT / 'tests/corridor/tb_wheel_1khz.sv', 350))
    for bench, pwm_limit in variants:
        name = bench.stem + ('_pwm350' if pwm_limit else '')
        image = OUT / (name + '.vvp')
        parameters = ['-P' + bench.stem + '.PWM_LIMIT=' + str(pwm_limit)] if pwm_limit else []
        result = subprocess.run(['iverilog', '-g2012', '-Wall', '-s', bench.stem,
                                 *parameters, '-o', str(image), *map(str, sources), str(bench)],
                                cwd=ROOT, capture_output=True, text=True, timeout=60)
        (OUT / (name + '_compile.log')).write_text(result.stdout + result.stderr, encoding='utf-8')
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        result = subprocess.run(['vvp', str(image)], cwd=ROOT, capture_output=True,
                                text=True, timeout=120)
        log = result.stdout + result.stderr
        (OUT / (name + '.log')).write_text(log, encoding='utf-8')
        print(log.strip(), flush=True)
        if result.returncode or 'PASS' not in log or 'FATAL' in log:
            raise RuntimeError(name + ' failed')
        cases.append(dict(case=name, passed=True, output=log))
    assert cases
    assert all(hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest
               for name, digest in hashes.items())
    rows = list(csv.DictReader((OUT / 'low_speed.csv').open(encoding='ascii')))[1:]
    smooth = [int(row['hybrid_q8']) / 256 for row in rows]
    direct = [int(row['one_ms_count_q8']) / 256 for row in rows]
    expected = 2 * 10 / 5.5
    metrics = dict(synthetic_only=True, samples=len(rows), units='position_units/10ms',
                   expected=expected, hybrid_mean=statistics.mean(smooth),
                   hybrid_stddev=statistics.pstdev(smooth), one_ms_count_stddev=statistics.pstdev(direct))
    assert len(rows) >= 30 and abs(metrics['hybrid_mean'] - expected) < expected * .05
    assert metrics['hybrid_stddev'] < metrics['one_ms_count_stddev'] / 5
    print('PASS synthetic low-speed quantization:', metrics, flush=True)
    (OUT / 'result.json').write_text(json.dumps(dict(passed=True, cases=cases,
        hardware_tested=False, programmed=False, low_speed_metrics=metrics, source_sha256=hashes), indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
