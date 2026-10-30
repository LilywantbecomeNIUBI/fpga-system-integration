"""Explicit SRAM-only motor download and STOP-only UART check. No ARM/DRIVE."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import time
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / '.deps')]
from shared.streams import MixedStream
from host.motor_protocol import command


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--power-off-confirmed', action='store_true')
    ap.add_argument('--verify-stop', action='store_true', help='Require direct UART STOP acknowledgement before SRAM download')
    ap.add_argument('--left-encoder-alt', action='store_true')
    ap.add_argument('--expected-bitstream-sha256', help='Explicit reviewed candidate SHA256; default retains the archived left-alt identity')
    args = ap.parse_args()
    if not args.power_off_confirmed and not args.verify_stop:
        ap.error('Confirm motor VM disconnected, or request direct STOP verification')
    build_dir = ROOT / ('build/motor_left_alt' if args.left_encoder_alt else 'build/motor')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    build = json.loads((build_dir / 'result.json').read_text())
    assert build['passed'] and build['violations'] == {'setup': 0, 'hold': 0}
    for path, expected in build['source_sha256'].items():
        assert sha(ROOT / path) == expected, 'Stale build: ' + path
    bit = build_dir / 'impl/pnr/motor.fs'
    assert sha(bit) == build['bitstream_sha256'], 'Bitstream mismatch'
    if args.left_encoder_alt:
        assert build['pin_profile'] == 'left_encoder_p7_31_32'
        expected = args.expected_bitstream_sha256 or 'be049edb9456adf52f59454b9aff5829289c93f32b4ee524b8d79c6b81ddd78c'
        assert re.fullmatch(r'[0-9a-fA-F]{64}', expected), 'Invalid expected candidate SHA256'
        assert sha(bit) == expected.lower(), 'Candidate differs from explicitly expected SHA256'
    elif args.expected_bitstream_sha256:
        assert re.fullmatch(r'[0-9a-fA-F]{64}', args.expected_bitstream_sha256), 'Invalid expected candidate SHA256'
        assert sha(bit) == args.expected_bitstream_sha256.lower(), 'Candidate differs from explicitly expected SHA256'
    folder = ROOT / 'runs/commissioning' / ('speed_static_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True)
    cli = 'C:/Gowin/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe'
    record = dict(bitstream_sha256=sha(bit), operation='SRAM Program', flash_written=False,
                  motion_commands_sent=False, serial_port=args.port,
                  pin_profile=build.get('pin_profile','adc_remap_v2'),
                  motor_power_off_confirmed=args.power_off_confirmed)
    record['encoder_pull_mode']=build.get('encoder_pull_mode','archived_default')

    def run(options):
        result = subprocess.run([cli, *options], capture_output=True, timeout=55)
        output = (result.stdout + result.stderr).decode('utf-8', errors='replace')
        with (folder / 'programmer.log').open('a', encoding='utf-8') as log:
            log.write(repr(options) + '\n' + output + '\n')
        print(output, flush=True)
        assert result.returncode == 0 and not re.search(r'Error:', output, re.I), 'Programmer error'
        return output

    try:
        # Verify UART ownership before replacing SRAM; no bytes are sent here.
        import serial
        with serial.Serial(args.port, 115200, timeout=.02, write_timeout=.1) as port:
            if args.verify_stop:
                port.reset_input_buffer()
                port.write(command(0, 0))
                parser = MixedStream()
                start = time.monotonic()
                acknowledged = False
                while time.monotonic()-start < 2:
                    for kind,row in parser.feed(port.read(min(max(1,port.in_waiting),4096))):
                        if kind == 'motor' and not row['armed'] and row['left_applied']==row['right_applied']==0 and row['flags']==1:
                            acknowledged = True
                    if acknowledged:break
                assert acknowledged, 'Direct STOP acknowledgement missing; download cancelled'
                record['pre_download_stop_verified'] = True
        scan = run(['--scan-cables', 'F'])
        locations = re.findall(r'USB location:\s*(\d+)', scan)
        assert len(locations) == 1, 'Expected exactly one JTAG cable'
        cable = ['--cable-index', '1', '--location', locations[0]]
        device = run(cable + ['--scan'])
        assert re.search(r'(?:0x)?0011081B', device, re.I), 'Unexpected FPGA device'
        output = run(cable + ['--device', 'GW5AT-138B', '--operation_index', '2', '--fsFile', str(bit)])
        assert 'Finished.' in output, 'Download not confirmed'
        record['download_confirmed'] = True
        parser = MixedStream()
        rows = []
        with serial.Serial(args.port, 115200, timeout=.02, write_timeout=.1) as port, (folder / 'uart.bin').open('wb') as raw:
            port.write(command(0, 0))
            start = time.monotonic()
            while time.monotonic() - start < 5:
                data = port.read(min(max(1, port.in_waiting), 4096))
                raw.write(data)
                for kind, row in parser.feed(data):
                    rows.append(dict(elapsed=time.monotonic()-start, kind=kind, data=row))
                    if kind == 'motor':
                        assert not row['armed'] and row['left_applied'] == row['right_applied'] == 0, 'Nonzero output'
                        assert row['flags'] == 1, 'Unexpected firmware mode'
            port.write(command(0, 1))
        (folder / 'frames.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        motors = [r['data'] for r in rows if r['kind'] == 'motor']
        record.update(motor_frames=len(motors), bad_crc=parser.bad_crc, first=motors[0] if motors else None,
                      last=motors[-1] if motors else None)
        assert len(motors) >= 80 and parser.bad_crc == 0, 'Telemetry check failed'
        record['static_passed'] = True
        (ROOT/'runs/commissioning/active_motor_sram.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    except Exception as exc:
        record['error'] = str(exc)
        raise
    finally:
        (folder / 'result.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
        (ROOT/'runs/commissioning/last_motor_sram_attempt.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(dict(folder=str(folder), **record), indent=2), flush=True)


if __name__ == '__main__':
    main()
