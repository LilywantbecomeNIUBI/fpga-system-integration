"""Generate/install Nano boot services. Default only prints the service plan."""
from pathlib import Path
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def systemd_quote(value):
    value = str(value)
    if '\n' in value or '\r' in value or '\x00' in value:
        raise ValueError('Invalid newline in service path')
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%').replace('$', '$$') + '"'


def service(root, config, user, script, live=False):
    if not re.fullmatch(r'[a-z_][a-z0-9_-]*', user):
        raise ValueError('Invalid Linux service user')
    command = [sys.executable, '-u', str(root / script), '--config', str(config)]
    if live:
        command.append('--live')
    return '\n'.join([
        '[Unit]', 'Description=Robot Nano ' + ('LAN viewer' if live else 'vision'),
        'Wants=network-online.target', 'After=network-online.target', '', '[Service]',
        'Type=simple', 'User=' + user, 'WorkingDirectory=' + str(root).replace('%', '%%'),
        'ExecStart=' + ' '.join(systemd_quote(x) for x in command),
        'Restart=on-failure', 'RestartSec=5', 'TimeoutStopSec=20', 'UMask=0077',
        '', '[Install]', 'WantedBy=multi-user.target', ''])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--vision-config', type=Path)
    parser.add_argument('--user', default=os.environ.get('SUDO_USER') or os.environ.get('USER', 'liwei'))
    parser.add_argument('--install', action='store_true')
    parser.add_argument('--start', action='store_true')
    args = parser.parse_args()
    if args.start and not args.install:
        parser.error('--start requires --install')
    config = args.config.resolve()
    data = json.loads(config.read_text(encoding='utf-8'))
    receive_only = data.get('receive_only') is True and data.get('telemetry_transport') == 'udp' and data.get('lidar_source') == 'fpga'
    if not receive_only and (not data.get('fpga_serial') or 'REPLACE' in data['fpga_serial']):
        parser.error('Set the actual FPGA UART path in the local config first')
    units = {'robot-web.service': service(ROOT, config, args.user, 'nano/web_console.py', live=True)}
    if args.vision_config:
        vision = args.vision_config.resolve()
        sys.path.insert(0, str(ROOT))
        from nano.boot_vision import arguments
        arguments(json.loads(vision.read_text(encoding='utf-8')))
        units['robot-vision.service'] = service(ROOT, vision, args.user, 'nano/boot_vision.py')
    if not args.install:
        for name, content in units.items():
            print(name + '\n' + content)
        return
    if sys.platform != 'linux' or os.geteuid() != 0:
        parser.error('Install on the Nano using sudo; omit --install to review the units')
    with tempfile.TemporaryDirectory(prefix='robot-units-') as temporary:
        checks = []
        for name, content in units.items():
            path = Path(temporary) / name
            path.write_text(content, encoding='utf-8')
            checks.append(str(path))
        subprocess.check_call(['systemd-analyze', 'verify'] + checks)
    for name, content in units.items():
        target = Path('/etc/systemd/system') / name
        if target.exists() and target.read_text(encoding='utf-8') != content:
            parser.error('Existing unit differs: ' + str(target) + '; preserve it before installing')
    for name, content in units.items():
        (Path('/etc/systemd/system') / name).write_text(content, encoding='utf-8')
    subprocess.check_call(['systemctl', 'daemon-reload'])
    subprocess.check_call(['systemctl', 'enable'] + list(units))
    if args.start:
        subprocess.check_call(['systemctl', 'start'] + list(reversed(list(units))))
    print('Installed and enabled: ' + ', '.join(units))


if __name__ == '__main__':
    main()
