"""Local unified chassis/Nano console. Opening it never sends a command."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

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
import runtime as gui
from console import motor_profile
from shared.navigation_protocol import upload_operations
from remote import RemoteLease
from teaching import Teaching

SOFTWARE_VERSION = 'unified-teach-20261103'


class Console:
    def __init__(self, bind='192.168.50.100', peer='192.168.50.30', nano='http://192.168.50.20:8872', clock=time.monotonic, routes_dir=None):
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
        self.mode = 'task2'
        self.lidar = None
        self.lidar_received = self.clock()
        self.remote = RemoteLease()
        self.remote_epoch = None
        self.jog_active = False
        self.teaching = Teaching(self.profile, routes_dir or Path(__file__).resolve().parents[1]/'runs'/'routes')
        self.saved_routes = self.teaching.routes()
        self.prepared_teach_revision = None

    def stop(self, reason):
        self.remote.cancel()
        self.jog_active = False
        if self.link and self.link.is_alive():
            self.link.requests.put(('stop', None))
        self.owner = None
        self.remote_epoch = None
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
                    value = dict(value)
                    received = value.pop('_received_monotonic', self.clock())
                    if self.clock()-received >= .3:
                        self.teaching.invalidate('遥测处理积压，示教坐标已失效')
                        continue
                    self.frames[value['kind']] = (received, value)
                    if value['kind'] == 'motor':
                        pose = self.frames.get('pose')
                        self.teaching.observe(value, pose[1] if pose else None, received, pose[0] if pose else 0)
                elif kind == 'connected':
                    self.connected = True
                elif kind == 'disconnected':
                    self.connected = self.running = self.pending = False
                    self.owner = None
                    self.remote.cancel()
                    self.remote_epoch = None
                    self.jog_active = False
                    self.teaching.invalidate('底盘连接中断，请重新设置示教起点或确认回放起点')
                elif kind == 'fault':
                    self.stop('动作故障：'+value)
                elif kind == 'running':
                    self.running = value
                    self.pending = False
                elif kind == 'log':
                    self.pending = False
                    self.log.append(value)
            if self.owner and self.clock() >= self.expires:
                self.stop('浏览器控制心跳中断，已请求 STOP')
            if self.jog_active and self.remote.snapshot(self.clock()) is None:
                self.stop('遥控按住心跳中断，已请求 STOP')
            if self.teaching.valid and self.teaching.last_time is not None and self.clock()-self.teaching.last_time >= .35:
                self.teaching.invalidate('示教遥测过期，请重新设置示教起点或确认回放起点')

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
                lidar = data.get('lidar') or {}
                age = telemetry.get('age_ms')
                state = dict(online=True, version=data.get('software_version'),
                             telemetry_fresh=isinstance(age, (int, float)) and 0 <= age < 500 and not telemetry.get('restart_required'),
                             restart_required=bool(telemetry.get('restart_required')),
                             error=data.get('telemetry_error'))
            except (OSError, ValueError) as exc:
                state = dict(online=False, telemetry_fresh=False, error=str(exc))
            with self.lock:
                self.nano_state = state
                self.lidar = lidar if state['online'] and lidar.get('online') and (lidar.get('age_ms') or 0)<500 else None
                self.lidar_received = self.clock()
            self.closed.wait(.2)

    def action(self, data):
        with self.lock:
            self.tick()
            action = data.get('action')
            client = data.get('client')
            if not isinstance(action, str):
                raise ValueError('无效操作')
            if action == 'stop':
                self.stop('操作员 STOP')
                return
            if action == 'connect':
                if self.link and self.link.is_alive():
                    raise ValueError('已有连接；如需重连请先断开')
                self.frames.clear()
                self.teaching.invalidate('已重连；请重新设置示教起点或确认回放起点')
                gui.TELEMETRY_OPTIONS.update(bind=self.bind, peer=self.peer)
                gui.COMMAND_TRANSPORT = 'router'
                self.link = gui.SerialLink(None, self.events, self.profile)
                self.link.remote_lease = self.remote
                self.link.auto_configure_pc = True
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
                if self.owner != client:
                    self.remote = RemoteLease()
                    if self.link:self.link.remote_lease = self.remote
                    self.remote_epoch = secrets.token_hex(16)
                self.owner, self.expires = client, self.clock() + .6
                return dict(control_epoch=self.remote_epoch)
            if self.owner != client or self.clock() >= self.expires:
                raise ValueError('请先接管控制')
            if action == 'heartbeat':
                self.expires = self.clock() + .6
                return
            if action in ('jog_begin','jog_pulse','jog_end'):
                self._jog(action, data)
                return
            if action.startswith('teach_'):
                return self._teach(action, data)
            if action in ('nav_upload', 'nav_start', 'nav_resume', 'mode'):
                if not self.connected or not self.link or not self.link.is_alive():
                    raise ValueError('请先连接底盘')
                if self.running or self.pending:
                    raise ValueError('请先停止当前动作再切换/上传')
                navigation = self.frames.get('navigation')
                if not navigation or self.clock()-navigation[0] >= .5:
                    raise ValueError('需要支持导航的新固件及新鲜N4遥测；当前固件不能启动自主任务')
                if action == 'mode' and data.get('mode') not in ('task2', 'navigation'):
                    raise ValueError('无效验收模式')
                mid = int(data.get('mission_id', navigation[1]['mission_id']))
                spec = dict(kind='navigation', action=dict(nav_upload='upload', nav_start='start', nav_resume='resume', mode='mode')[action],
                            mission_id=mid, options=dict(motor=0, encoder=0, kp=2048, ki=128))
                if action == 'nav_upload':
                    upload_operations(data.get('calibration'), data.get('points'), mid)
                    spec.update(calibration=data['calibration'], points=data['points'])
                    self.prepared_teach_revision = None
                if action != 'mode' and self.mode != 'navigation':
                    raise ValueError('请先切换到任务3/4导航模式')
                if action == 'mode':
                    self.mode = data['mode']
                self.pending = True
                self.link.requests.put(('start', spec))
                return
            if action not in ('motion', 'calibrate'):
                raise ValueError('未知操作')
            if not self.connected or not self.link or not self.link.is_alive():
                raise ValueError('底盘未连接')
            if self.running or self.pending:
                raise ValueError('请先停止当前动作')
            if action == 'calibrate':
                if self.frames.get('navigation') and self.frames['navigation'][1]['active']:
                    raise ValueError('请先取消自主任务再校零；校零会改变任务坐标基准')
                self.pending = True
                self.teaching.invalidate('IMU校零改变坐标基准，请重新设置示教起点或确认回放起点')
                self.link.requests.put(('calibrate', None))
                return
            if self.mode != 'task2' or (self.frames.get('navigation') and self.frames['navigation'][1]['active']):
                raise ValueError('请先停止并切换到任务2独立验收模式')
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
            self.teaching.replay_ready = False
            self.link.requests.put(('start', spec))

    def _fresh_link(self):
        if not self.connected or not self.link or not self.link.is_alive():
            raise ValueError('请先连接底盘')
        now = self.clock()
        motor, pose = self.frames.get('motor'), self.frames.get('pose')
        if not motor or not pose or now-motor[0] >= .3 or now-pose[0] >= .3:
            raise ValueError('底盘或姿态遥测过期，不能执行')
        return motor, pose

    def _jog(self, action, data):
        if data.get('control_epoch') != self.remote_epoch or self.remote_epoch is None:
            raise ValueError('遥控会话已失效，请重新接管')
        gesture = data.get('gesture')
        if isinstance(gesture, bool) or not isinstance(gesture, int) or not 1 <= gesture <= 9007199254740991:
            raise ValueError('无效遥控按键序号')
        if action == 'jog_end':
            # Tombstone even if release overtakes a delayed begin request.
            self.remote.end(gesture)
            self.jog_active = self.remote.snapshot(self.clock()) is not None
            return
        if action == 'jog_pulse':
            self.remote.pulse(gesture, self.clock())
            return
        motor, pose = self._fresh_link()
        if self.mode != 'task2' or (self.frames.get('navigation') and self.frames['navigation'][1]['active']):
            raise ValueError('请先停止自主任务并切换到任务2/遥控模式')
        if (self.running or self.pending) and not self.jog_active:
            raise ValueError('请先停止当前动作')
        if not pose[1].get('healthy') or pose[1].get('imu_stale'):
            raise ValueError('IMU未就绪，不能遥控')
        direction = data.get('direction')
        if (not isinstance(direction, list) or len(direction) != 3 or
                any(type(v) is not int or v not in (-1,0,1) for v in direction) or
                direction == [0,0,0] or (direction[2] and (direction[0] or direction[1]))):
            raise ValueError('无效遥控方向')
        limit = data.get('limit', 2)
        if type(limit) is not int or not 1 <= limit <= 2:
            raise ValueError('遥控低速档仅支持1或2')
        self.remote.begin(gesture, direction, limit, self.clock())
        self.jog_active = True
        self.teaching.replay_ready = False

    def _teach(self, action, data):
        if action in ('teach_save','teach_export'):
            if action == 'teach_save':
                try:route_id = self.teaching.save(data.get('name'))
                except OSError as exc:raise ValueError('路线保存失败：'+str(exc)) from exc
                self.saved_routes = self.teaching.routes()
                return dict(route_id=route_id, route=self.teaching.route())
            return dict(route=self.teaching.route())
        if action in ('teach_calibration','teach_load','teach_import'):
            if self.running or self.pending or self.jog_active:
                raise ValueError('请先停止当前动作')
            nav = self.frames.get('navigation')
            if nav and nav[1]['active']:
                raise ValueError('请先取消自主任务再修改示教')
            if action == 'teach_calibration':self.teaching.configure(data.get('calibration'))
            elif action == 'teach_load':self.teaching.load_id(data.get('route_id'))
            else:self.teaching.load(data.get('route'))
            self.prepared_teach_revision = None
            return
        motor, pose = self._fresh_link()
        if self.running or self.pending or self.jog_active:
            raise ValueError('请先松开遥控并等待停车')
        nav = self.frames.get('navigation')
        if nav and nav[1]['active']:
            raise ValueError('请先取消自主任务再示教')
        if action in ('teach_origin','teach_replay_origin'):
            if data.get('confirmed') is not True:
                raise ValueError('请确认车已放到固定起点且车头对准起点箭头')
            self.teaching.set_origin(motor, pose, self.clock(), replay=action=='teach_replay_origin')
            self.prepared_teach_revision = None
        elif action == 'teach_record':
            self.teaching.record(motor, pose, self.clock(), data.get('speed',30))
            self.prepared_teach_revision = None
        elif action == 'teach_undo':
            self.teaching.require_stationary(motor, pose, self.clock())
            if self.teaching.points:self.teaching.points.pop()
            self.teaching.revision += 1
            self.prepared_teach_revision = None
        elif action in ('teach_upload','teach_start'):
            self.teaching.require_stationary(motor, pose, self.clock())
            if not self.teaching.valid or not self.teaching.replay_ready:
                raise ValueError('请先把车放回固定起点、对准车头并确认回放起点')
            upload_operations(self.teaching.calibration, self.teaching.points, int(data.get('mission_id',1)))
            if action == 'teach_start':
                n = nav[1] if nav else {}
                if (self.prepared_teach_revision != (self.teaching.revision,int(data.get('mission_id',1))) or
                        n.get('status') != 2 or n.get('mission_id') != int(data.get('mission_id',1)) or
                        n.get('ack_error') != 0):
                    raise ValueError('请先上传示教路线并等待FPGA校验完成')
            previous_mode = self.mode
            if action == 'teach_upload':self.mode = 'navigation'
            try:
                self.action(dict(action='nav_upload' if action=='teach_upload' else 'nav_start',
                                 client=data['client'], mission_id=int(data.get('mission_id',1)),
                                 calibration=self.teaching.calibration, points=self.teaching.points))
            except (ValueError, TypeError):
                self.mode = previous_mode
                raise
            if action == 'teach_upload':
                self.prepared_teach_revision = (self.teaching.revision,int(data.get('mission_id',1)))
            else:self.teaching.replay_ready = False
        else:raise ValueError('未知示教操作')

    def state(self, client=None):
        with self.lock:
            self.tick()
            now = self.clock()
            data = dict(profile_id='unified_mecanum_v1', software_version=SOFTWARE_VERSION,connected=self.connected, running=self.running or self.pending or self.jog_active,
                        operator=bool(self.owner and self.owner == client), occupied=bool(self.owner),
                        nano=dict(self.nano_state), log=list(self.log), profile=self.profile,mode=self.mode,
                        control_epoch=self.remote_epoch if self.owner==client else None,
                        remote_active=self.jog_active,teaching=self.teaching.snapshot(now),saved_routes=self.saved_routes,
                        lidar=self.lidar if self.nano_state['online'] and self.lidar and now-self.lidar_received+(self.lidar.get('age_ms') or 0)/1000<.5 else None)
            for kind in ('motor', 'pose', 'navigation'):
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
            elif path.path == '/remote.js':
                self.reply(Path(__file__).with_name('remote.js').read_bytes(), kind='text/javascript; charset=utf-8')
            else:
                self.reply(dict(error='Not found'), 404)

        def do_POST(self):
            if (not self.allowed() or self.path != '/api/action' or self.headers.get('X-Console-Token') != token
                    or self.headers.get('Origin') not in (None, 'http://' + self.headers.get('Host', ''))):
                return self.reply(dict(error='请求来源或令牌无效'), 403)
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 16384:
                    raise ValueError('请求过大或为空')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('请求必须为对象')
                result = console.action(data)
                self.reply(dict(ok=True, **(result or {})))
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
