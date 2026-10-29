"""Dedicated HTTP tuner; reuses the sole UART gateway, never opens serial/video ports."""
import argparse
import json
import os
from pathlib import Path
import secrets
import threading
from dashboard import GatewayClient, make_server


class TunerSession:
    def __init__(self, url, token, live=False):
        self.gateway = GatewayClient(url, token, live)
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self.poll, daemon=True)
        self.worker.start()

    def poll(self):
        while not self.stop.is_set():
            self.gateway.poll()
            self.stop.wait(.05)

    def state(self):
        status = self.gateway.snapshot()
        data = status.get('data') or {}
        control = data.get('control') if status['online'] else None
        return dict(gateway=status, control=control, mode='live' if self.gateway.live else 'offline')

    def record(self, enabled):
        raise ValueError('Use gateway recording and tuner CSV export')

    def image(self, *args):
        return None

    def close(self):
        self.gateway.release()
        self.stop.set()
        self.worker.join(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-url', default='http://127.0.0.1:8873')
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--port', type=int, default=8874)
    parser.add_argument('--token-file', type=Path, help='Local JSON token file; never printed')
    args = parser.parse_args()
    token = os.environ.get('ROBOT_GATEWAY_TOKEN', '')
    if args.token_file:
        token = json.loads(args.token_file.read_text(encoding='utf-8'))['token']
    if args.live and len(token) < 24:
        parser.error('Live mode requires gateway session token')
    session = TunerSession(args.gateway_url, token, args.live)
    server = make_server(session, args.port, secrets.token_hex(24), page='motor_tuner.html')
    try:
        print('Wheel speed tuner http://127.0.0.1:%d (%s)' % (args.port, 'live' if args.live else 'offline'), flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        session.close()
        server.server_close()


if __name__ == '__main__':
    main()
