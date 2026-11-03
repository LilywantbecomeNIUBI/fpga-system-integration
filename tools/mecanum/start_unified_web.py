"""Choose the current car LAN route, configure PC replies, open the unified web UI."""
from pathlib import Path
import argparse
import json
import subprocess
import sys
import threading
import webbrowser
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.mecanum.start_router_gui import lan_address
from tools.task4.telemetry_endpoint import transact


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fpga-ip', default='192.168.50.30')
    parser.add_argument('--port', type=int, default=8874)
    args = parser.parse_args()
    try:
        url = 'http://127.0.0.1:%s' % args.port
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(url + '/api/state', timeout=.5) as response:
                existing = json.load(response)
            if existing.get('profile_id') == 'unified_mecanum_v1':
                webbrowser.open(url)
                return 0
        except (OSError, ValueError):
            pass
        local = lan_address(args.fpga_ip)
        state = transact(local, board=args.fpga_ip, target=local, slot='pc')
        if state['target'] != local or not state['enabled']:
            raise RuntimeError('FPGA 未确认 PC 回传地址')
        print('PC telemetry: %s; open web UI and click connect. No automatic ARM.' % local, flush=True)
        timer = threading.Timer(1, webbrowser.open, args=(url,))
        timer.daemon = True
        timer.start()
        return subprocess.call([sys.executable, str(ROOT / 'host/mecanum/web.py'),
                                '--telemetry-bind', local, '--fpga-ip', args.fpga_ip,
                                '--port', str(args.port)], cwd=ROOT)
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        print('启动失败：%s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
