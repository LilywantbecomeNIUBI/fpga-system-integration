"""Prepare an isolated local console from the current static console; never starts it."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--current',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    target=args.output.resolve();target.mkdir(parents=True,exist_ok=False)
    for name in ('nano','shared','host','vendor/imu/pc','vendor/vision','vendor/notices'):
        shutil.copytree(ROOT/name,target/name,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(args.current/'.deps',target/'.deps',ignore=shutil.ignore_patterns('__pycache__'))
    (target/'tools/task4').mkdir(parents=True)
    shutil.copy2(ROOT/'tools/task4/protocol.py',target/'tools/task4/protocol.py')
    source=(args.current/'board_console.py').read_text(encoding='utf-8')
    source=source.replace("'lidar_source': 'fpga', 'record_dir'", "'lidar_source': 'fpga', 'telemetry_transport': 'udp', 'telemetry_bind': '192.168.50.10', 'fpga_ip': '192.168.50.30', 'record_dir'")
    source=source.replace('def ingest_uart(self, data):\n        nav.ingest(data)\n        super().ingest_uart(data)',
                          'def ingest_uart(self, data, now=None):\n        nav.ingest(data, now=now)\n        super().ingest_uart(data, now=now)')
    source=source.replace('M2 static sensor diagnostic / RAM scan','M2 static sensors / all parsed lidar points / Ethernet UDP')
    source=source.replace('M2 static sensors / 360 measured points / Ethernet UDP','M2 static sensors / all parsed lidar points / Ethernet UDP')
    source=source.replace("'startup': 'STOP only'", "'telemetry_transport': 'udp', 'telemetry_port': 6205, 'startup': 'STOP only'")
    if "'telemetry_transport': 'udp'" not in source or 'def ingest_uart(self, data, now=None)' not in source:
        raise RuntimeError('Current console structure changed; no startup performed')
    (target/'board_console.py').write_text(source,encoding='utf-8')
    files={str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest() for p in target.rglob('*.py')}
    (target/'prepared.json').write_text(json.dumps(dict(prepared=True,started=False,telemetry_transport='udp',source_sha256=files),indent=2))
    print(target)


if __name__=='__main__':main()
