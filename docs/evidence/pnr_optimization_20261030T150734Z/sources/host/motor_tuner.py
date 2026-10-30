"""Dedicated HTTP tuner; reuses the sole UART gateway, never opens serial/video ports."""
import argparse
import json
import os
import ipaddress
import time
from pathlib import Path
import secrets
import threading
from urllib.parse import urlsplit
from dashboard import GatewayClient, make_server


class TunerSession:
    def __init__(self, url, token, live=False, config_path=None):
        self.gateway = GatewayClient(url, token, live)
        self.connection_lock = threading.RLock()
        self.config_path = config_path
        self.connection_attempts = 0
        self.connection_generation = 0
        self.connection_latency_ms = None
        self.reconnect_event = threading.Event()
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self.poll, daemon=True)
        self.worker.start()

    def poll(self):
        delay = .05
        while not self.stop.is_set():
            with self.connection_lock:
                started = time.monotonic()
                self.gateway.poll()
                self.connection_attempts += 1
                self.connection_latency_ms = round((time.monotonic()-started)*1000,1)
                online = self.gateway.snapshot()['online']
            delay = .05 if online else min(max(delay*2,.2),3.0)
            self.reconnect_event.wait(delay)
            self.reconnect_event.clear()

    def configure_connection(self, data):
        url = data.get('gateway_url', '')
        parsed = urlsplit(url)
        try:
            address = ipaddress.ip_address(parsed.hostname or '')
            allowed = any(address in ipaddress.ip_network(net) for net in
                          ('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16','100.64.0.0/10'))
            valid = parsed.scheme == 'http' and parsed.port == 8873 and allowed
        except ValueError:
            valid = False
        if not valid or parsed.username or parsed.password or parsed.path not in ('','/') or parsed.query or parsed.fragment:
            raise ValueError('请输入Nano局域网IPv4地址，格式 http://192.168.50.20:8873')
        url = 'http://%s:8873' % parsed.hostname
        with self.connection_lock:
            self.gateway.reconnect(url)
            self.reconnect_event.set()
            self.connection_generation += 1
            self.connection_attempts = 0
            if self.config_path:
                cfg = json.loads(self.config_path.read_text(encoding='utf-8-sig'))
                cfg['gateway_url'] = url
                self.config_path.write_text(json.dumps(cfg,ensure_ascii=False,indent=2),encoding='utf-8')
        return dict(ok=True,gateway_url=url,generation=self.connection_generation)

    def state(self):
        status = self.gateway.snapshot()
        data = status.get('data') or {}
        control = data.get('control') if status['online'] else None
        return dict(gateway=status, gateway_host=urlsplit(self.gateway.url).hostname if self.gateway.url else None,
                    connection=dict(gateway_url=self.gateway.url,generation=self.connection_generation,
                                    attempts=self.connection_attempts,latency_ms=self.connection_latency_ms,
                                    auto_reconnect=True),
                    control=control, mode='live' if self.gateway.live else 'offline')

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
    parser.add_argument('--config', type=Path, help='Local JSON containing gateway_url and token_file')
    parser.add_argument('--home', action='store_true', help='Open the host entry page at /')
    args = parser.parse_args()
    if args.config:
        cfg = json.loads(args.config.read_text(encoding='utf-8-sig'))
        args.gateway_url = cfg['gateway_url']
        args.token_file = (args.config.resolve().parent / cfg['token_file']).resolve()
    token = os.environ.get('ROBOT_GATEWAY_TOKEN', '')
    if args.token_file:
        token = json.loads(args.token_file.read_text(encoding='utf-8'))['token']
    if args.live and len(token) < 24:
        parser.error('Live mode requires gateway session token')
    session = TunerSession(args.gateway_url, token, args.live, args.config)
    server = make_server(session, args.port, secrets.token_hex(24), page='home.html' if args.home else 'motor_tuner.html')
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
