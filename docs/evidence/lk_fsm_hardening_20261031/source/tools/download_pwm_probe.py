"""Reload the verified PWM diagnostic into SRAM only; never sends motion commands."""
import hashlib
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CLI = Path('C:/Gowin/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe')
BIT = ROOT / 'build/pwm_probe/impl/pnr/motor.fs'
SHA = '387c4a8c8664bfd6e60ed290146fbb560ec9f4e43a266840a725d7f26963f8c6'

def main():
    if not CLI.is_file() or not BIT.is_file():
        raise RuntimeError('Programmer or motor.fs missing. Keep this script inside the original repository.')
    if hashlib.sha256(BIT.read_bytes()).hexdigest() != SHA:
        raise RuntimeError('Bitstream SHA256 mismatch; download cancelled.')
    folder = ROOT / 'runs' / 'commissioning' / ('manual_download_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True, exist_ok=True)
    def run(args, timeout=55):
        command = [str(CLI), *args]
        print('Running:', ' '.join(command), flush=True)
        with (folder / 'programmer.log').open('a', encoding='utf-8') as log:
            log.write('\n' + repr(command) + '\n')
            try:
                result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
                output = result.stdout.decode('utf-8', errors='replace')
                log.write(output)
                print(output, flush=True)
            except subprocess.TimeoutExpired as error:
                log.write((error.stdout or b'').decode('utf-8', errors='replace'))
                raise RuntimeError('Programmer timed out. Check USB/JTAG connection and retry.') from error
        if result.returncode or re.search(r'Error:', output, re.I):
            raise RuntimeError('Programmer reported an error; download is NOT confirmed.')
        return output
    print('SRAM only. Power-off erases this firmware. No motor commands are sent.', flush=True)
    scan = run(['--scan-cables', 'F'])
    locations = re.findall(r'USB location:\s*(\d+)', scan)
    if len(locations) != 1:
        raise RuntimeError('Expected exactly one Gowin USB cable; connect only the intended board.')
    cable = ['--cable-index', '1', '--location', locations[0]]
    device = run(cable + ['--scan'])
    if not re.search(r'(?:0x)?0011081B', device, re.I):
        raise RuntimeError('Expected FPGA device ID 0x0011081B was not detected.')
    output = run(cable + ['--device', 'GW5AT-138B', '--operation_index', '2', '--fsFile', str(BIT)])
    if 'Finished.' not in output:
        raise RuntimeError('Missing Finished marker; download is NOT confirmed.')
    print('SUCCESS: PWM diagnostic loaded into SRAM. Motors remain stopped until a manual console test.')
    print('Log:', folder / 'programmer.log')

if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('FAILED:', error, file=sys.stderr)
        sys.exit(1)
