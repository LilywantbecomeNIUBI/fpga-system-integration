"""Local unified chassis/Nano console. Opening it never sends a command."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import secrets
import sys
import threading
import time
import urllib.request
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gui
from console import motor_profile


class Console:
    def __init__(self, bind='192.168.50.100', peer='192.168.50.30', nano='http://192.168.50.20:8872', clock=time.monotonic):
        self.clock = clock
        self.profile = motor_profile()
        self.bind, self.peer, self.nano = bind, peer, nano.rstrip('/')
        self.events = queue.Queue()
        self.lock = threading.RLock()
        self.link = None
        self.connected = self.running = self.pending = False
        self.frames = {}
        self.log = deque(maxlen=40)
        self.owner = None
        self.expires = 0
        self.closed = threading.Event()
        self.nano_state = dict(online=False, error='尚未读取 Nano')

    def stop(self, reason):
        if self.link and self.link.is_alive():
            self.link.requests.put(('stop', None))
        self.owner = None
        self.pending = False
        self.log.append(reason)

    def tick(self):
        with self.lock:
            while True:
                try:
                    kind, value = self.events.get_nowait()
                except queue.Empty:
                    break
                if kind == 'frame':
                    self.frames[value['kind']] = (self.clock(), value)
                elif kind == 'connected':
                    self.connected = True
                elif kind == 'disconnected':
                    self.connected = self.running = self.pending = False
                    self.owner = None
                elif kind == 'running':
                    self.running = value
                    self.pending = False
                elif kind == 'log':
                    self.pending = False
                    self.log.append(value)
            if self.owner and self.clock() >= self.expires:
                self.stop('浏览器控制心跳中断，已请求 STOP')

    def run(self):
        while not self.closed.wait(.02):
            self.tick()

    def poll_nano(self):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while not self.closed.is_set():
            try:
                with opener.open(self.nano + '/api/state', timeout=1) as response:
                    data = json.loads(response.read(4 * 1024 * 1024))
                telemetry = data.get('telemetry') or {}
                age = telemetry.get('age_ms')
                state = dict(online=True, version=data.get('software_version'),
                             telemetry_fresh=isinstance(age, (int, float)) and 0 <= age < 500 and not telemetry.get('restart_required'),
                             restart_required=bool(telemetry.get('restart_required')),
                             error=data.get('telemetry_error'))
            except (OSError, ValueError) as exc:
                state = dict(online=False, telemetry_fresh=False, error=str(exc))
            with self.lock:
                self.nano_state = state
            self.closed.wait(1)

    def action(self, data):
        with self.lock:
            self.tick()
            action = data.get('action')
            client = data.get('client')
            if action == 'stop':
                self.stop('操作员 STOP')
                return
            if action == 'connect':
                if self.link and self.link.is_alive():
                    raise ValueError('已有连接；如需重连请先断开')
                self.frames.clear()
                gui.TELEMETRY_OPTIONS.update(bind=self.bind, peer=self.peer)
                gui.COMMAND_TRANSPORT = 'router'
                self.link = gui.SerialLink(None, self.events, self.profile)
                self.link.start()
                return
            if action == 'disconnect':
                if self.link:
                    self.stop('断开底盘连接')
                    self.link.close_requested.set()
                return
            if not isinstance(client, str) or not 16 <= len(client) <= 128:
                raise ValueError('无效浏览器会话')
            if action == 'acquire':
                if self.owner and self.owner != client:
                    raise ValueError('另一个网页正在操作')
                if not self.connected:
                    raise ValueError('请先连接底盘')
                self.owner, self.expires = client, self.clock() + .6
                return
            if self.owner != client or self.clock() >= self.expires:
                raise ValueError('请先接管控制')
            if action == 'heartbeat':
                self.expires = self.clock() + .6
                return
            if action not in ('motion', 'calibrate'):
                raise ValueError('未知操作')
            if not self.connected or not self.link or not self.link.is_alive():
                raise ValueError('底盘未连接')
            if self.running or self.pending:
                raise ValueError('请先停止当前动作')
            if action == 'calibrate':
                self.pending = True
                self.link.requests.put(('calibrate', None))
                return
            kind = data.get('kind')
            if kind not in ('定角', '位置', '任意角度'):
                raise ValueError('无效运动模式')
            dx, dy, angle = (float(data.get(k, 0)) for k in ('dx', 'dy', 'angle'))
            if not all(math.isfinite(v) for v in (dx, dy, angle)) or abs(dx) > 4000 or abs(dy) > 4000 or abs(angle) > 720:
                raise ValueError('X/Y 限制 ±4000 mm，转角 ±720°，参数必须有限')
            spec = gui.motion_spec(kind, dx, dy, angle,
                                   int(data.get('limit', self.profile['default_speed_limit'])),
                                   4, float(data.get('duration', 60)),
                                   dict(motor=0, encoder=0, kp=int(data.get('kp', 2048)), ki=int(data.get('ki', 128))),
                                   self.profile, distance_mm=float(data.get('distance', 200)), bearing_deg=float(data.get('bearing', 45)))
            self.pending = True
            self.link.requests.put(('start', spec))

    def state(self, client=None):
        with self.lock:
            self.tick()
            now = self.clock()
            data = dict(profile_id='unified_mecanum_v1', connected=self.connected, running=self.running or self.pending,
                        operator=bool(self.owner and self.owner == client), occupied=bool(self.owner),
                        nano=dict(self.nano_state), log=list(self.log), profile=self.profile)
            for kind in ('motor', 'pose'):
                item = self.frames.get(kind)
                data[kind] = item[1] if item else None
                data[kind + '_age_ms'] = (now-item[0])*1000 if item else None
            return data

    def close(self):
        with self.lock:
            if self.running or self.pending or self.owner:
                self.stop('控制台退出')
        # Let the command worker consume STOP before closing the socket.
        if self.link:
            deadline = time.monotonic() + .3
            while not self.link.requests.empty() and self.link.is_alive() and time.monotonic() < deadline:
                time.sleep(.01)
            self.link.close_requested.set()
            self.link.join(1)
        self.closed.set()


def make_server(console, port=8874):
    token = secrets.token_hex(32)
    page = Path(__file__).with_name('web.html').read_text(encoding='utf-8')
    page = page.replace('__TOKEN__', token).replace('__NANO__', console.nano)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, data, status=200, kind='application/json; charset=utf-8'):
            raw = data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()
            try:
                self.wfile.write(raw)
            except OSError:
                pass

        def allowed(self):
            return self.headers.get('Host') in ('127.0.0.1:%s' % self.server.server_port, 'localhost:%s' % self.server.server_port)

        def do_GET(self):
            if not self.allowed():
                return self.reply(dict(error='Invalid host'), 403)
            path = urlsplit(self.path)
            if path.path == '/':
                self.reply(page.encode('utf-8'), kind='text/html; charset=utf-8')
            elif path.path == '/favicon.ico':
                self.reply(b'', 204, 'image/x-icon')
            elif path.path == '/api/state':
                self.reply(console.state(self.headers.get('X-Client')))
            else:
                self.reply(dict(error='Not found'), 404)

        def do_POST(self):
            if (not self.allowed() or self.path != '/api/action' or self.headers.get('X-Console-Token') != token
                    or self.headers.get('Origin') not in (None, 'http://' + self.headers.get('Host', ''))):
                return self.reply(dict(error='请求来源或令牌无效'), 403)
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 4096:
                    raise ValueError('请求过大或为空')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('请求必须为对象')
                console.action(data)
                self.reply(dict(ok=True))
            except (ValueError, TypeError, OverflowError) as exc:
                self.reply(dict(error=str(exc)), 400)

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8874)
    parser.add_argument('--telemetry-bind', default='192.168.50.100')
    parser.add_argument('--fpga-ip', default='192.168.50.30')
    args = parser.parse_args()
    console = Console(args.telemetry_bind, args.fpga_ip)
    server = make_server(console, args.port)
    for target in (console.run, console.poll_nano):
        threading.Thread(target=target, daemon=True).start()
    print('Unified console http://127.0.0.1:%s (starts disconnected)' % server.server_port, flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        console.close()
        server.server_close()


if __name__ == '__main__':
    main()
