"""Standalone Nano LAN viewer. One gateway owner; browsers never send commands.

Python 3.6+, standard library only. Reuses the existing vision process over HTTP
instead of opening its UDP sockets or loading another inference engine.
"""
from pathlib import Path
import argparse
import base64
import copy
from collections import deque
import json
import os
import secrets
import signal
import sys
import threading
import time
import urllib.request
from urllib.parse import urlsplit, urlencode
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from nano.gateway import Gateway, Recorder, make_server as gateway_server


def fresh(age, limit=500):
    return isinstance(age, (int, float)) and 0 <= age < limit


class ViewerGateway(Gateway):
    """UDP observation can run without a command UART or any serial writes."""
    def __init__(self, config=None, live=False):
        super().__init__(config, live)
        self.receive_only = self.config.get('receive_only', False)
        if not isinstance(self.receive_only, bool):
            raise ValueError('receive_only must be boolean')
        if self.receive_only:
            if self.telemetry_transport != 'udp' or self.lidar_source != 'fpga':
                raise ValueError('receive_only requires UDP telemetry and FPGA lidar')
            self.control.request = self.reject_command

    @staticmethod
    def reject_command(data):
        raise ValueError('Receive-only gateway has no command path')

    def start(self):
        if not self.receive_only:
            return super().start()
        if not self.live:
            return
        self.recorder = Recorder(Path(self.config.get('record_dir', 'runs/gateway')) /
                                 (time.strftime('%Y%m%d_%H%M%S') + '_' + secrets.token_hex(4)))
        self.control.connect()
        self.control.reason = 'receive_only'
        for target in (self.telemetry_loop, self.record_loop):
            worker = threading.Thread(target=target, daemon=True)
            self.threads.append(worker)
            worker.start()


class VisionClient:
    def __init__(self, url, live=False, clock=time.monotonic):
        parsed = urlsplit(url)
        if parsed.scheme != 'http' or not parsed.hostname or parsed.username or parsed.password:
            raise ValueError('vision_url must be an HTTP service URL without credentials')
        self.url = url.rstrip('/')
        self.clock = clock
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.data = self.received = self.camera = self.inference = None
        self.error = self.camera_error = None
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.camera_times = deque(maxlen=120)
        self.inference_times = deque(maxlen=120)
        self.thread = None
        if live:
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def get(self, path, limit):
        with self.opener.open(self.url + path, timeout=.6) as response:
            raw = response.read(limit + 1)
            if len(raw) > limit:
                raise ValueError('Vision response too large')
            return raw

    def poll(self):
        try:
            with self.lock:
                previous = self.inference
            query = '?' + urlencode({'after': previous[0]}) if previous else ''
            data = json.loads(self.get('/api/result' + query, 2 * 1024 * 1024).decode('utf-8'))
            if data.get('profile') != 'fpga_yolo_dashboard_v1':
                raise ValueError('Unexpected vision profile')
            received = self.clock()
            result = data.get('result') or {}
            key = str(result.get('session')) + '-' + str(result.get('sequence'))
            encoded = result.pop('jpeg_b64', None)
            inference = previous if previous and previous[0] == key else None
            if encoded:
                payload = base64.b64decode(encoded, validate=True)
                self.check_jpeg(payload)
                inference = (key, payload)
            camera = None
            camera_error = None
            if fresh((data.get('capture') or {}).get('age_ms')) and not data.get('offline'):
                try:
                    payload = self.get('/camera.jpg', 2 * 1024 * 1024)
                    self.check_jpeg(payload)
                    camera = (str(data.get('session')) + '-' + str((data.get('capture') or {}).get('frame_id')), payload)
                except (OSError, ValueError) as exc:
                    camera_error = str(exc)
            with self.lock:
                if camera and (not self.camera or camera[0] != self.camera[0]):
                    self.camera_times.append(received)
                if inference and (not previous or inference[0] != previous[0]):
                    self.inference_times.append(received)
                self.data, self.received = data, received
                self.inference, self.camera = inference, camera
                self.error, self.camera_error = None, camera_error
        except (OSError, ValueError, TypeError, KeyError) as exc:
            with self.lock:
                self.data = self.received = self.camera = self.inference = None
                self.error = str(exc)

    @staticmethod
    def check_jpeg(payload):
        if not payload.startswith(b'\xff\xd8') or not payload.endswith(b'\xff\xd9'):
            raise ValueError('Invalid JPEG preview')

    def run(self):
        while not self.stop.is_set():
            started = self.clock()
            self.poll()
            self.stop.wait(max(0, .1 - (self.clock() - started)))

    def snapshot(self):
        with self.lock:
            age = (self.clock() - self.received) * 1000 if self.received is not None else None
            data = copy.deepcopy(self.data) if self.data else {}
            error, camera_error = self.error, self.camera_error
            rates = []
            for times in (self.camera_times, self.inference_times):
                recent = [t for t in times if self.clock() - t < 2]
                rates.append((len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0.)
            camera_available = self.camera is not None
        online = fresh(age) and not error
        for row, name in ((data.get('capture'), 'age_ms'), (data.get('result'), 'age_ms'), (data, 'event_age_ms')):
            if row and row.get(name) is not None:
                row[name] += age or 0
        result = data.get('result') or {}
        flow = data.get('flow')
        if flow:
            control = flow.get('control') or {}
            if control.get('age_ms') is not None:
                control['age_ms'] += age or 0
            if not (online and control.get('fresh') and fresh(control.get('age_ms'), 200)):
                control.update(fresh=False, confident=False, state='unknown', stop_request=True, steer=0, speed_limit=0)
                flow['vectors'] = []
            flow['control'] = control
            if flow.get('diagnostic') and flow['diagnostic'].get('age_ms') is not None:
                flow['diagnostic']['age_ms'] += age or 0
            flow.update(observe_only=True, automatic_control_enabled=False)
            if not online:
                flow['vectors'] = []
        capture = data.get('capture') or {}
        capture['online'] = bool(online and not data.get('offline') and fresh(capture.get('age_ms')) and camera_available)
        capture['preview_fps'] = rates[0] if capture['online'] else 0.
        data['preview_fps'] = rates[1] if online and fresh(result.get('age_ms')) else 0.
        if not capture['online']:
            capture['fps'] = capture['receiver_fps'] = 0.
        if not (online and fresh(result.get('age_ms'))):
            data['inference_fps'] = 0.
        capture['error'] = camera_error or capture.get('error') or error
        return dict(online=bool(online), poll_age_ms=age, error=error, data=data,
                    result_fresh=bool(online and not data.get('error') and not data.get('offline') and fresh(result.get('age_ms')))), capture

    def image(self, inference=False):
        # Video handlers must not deepcopy the full radar/flow state per frame.
        with self.lock:
            if self.received is None or not fresh((self.clock()-self.received)*1000) or self.error:
                return None
            data = self.data or {}
            if data.get('offline') or (inference and data.get('error')):
                return None
            row = data.get('result' if inference else 'capture') or {}
            age = row.get('age_ms')
            if age is None or not fresh(age + (self.clock()-self.received)*1000):
                return None
            return self.inference if inference else self.camera

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(2)


class Session:
    def __init__(self, gateway, vision):
        self.gateway, self.vision = gateway, vision
        self.stop = threading.Event()

    def state(self):
        data = self.gateway.state()
        nano, capture = self.vision.snapshot()
        vision = nano['data']
        event_age = vision.get('event_age_ms')
        control = data['control']
        return dict(profile='system_integration_v1', host='nano', read_only=True,
                    software_version='nano-video-20261103',
                    mode=data['mode'], simulation=False, replay=False,
                    motor_outputs_enabled=bool(control.get('motor_fresh') and (control.get('motor') or {}).get('armed')),
                    pc=capture, nano=nano, imu=data['imu'], control=control,
                    gateway=dict(online=True, age_ms=0, error=data.get('serial_error')),
                    lidar=data['lidar'], odometry=data['odometry'], telemetry=data.get('telemetry'),
                    telemetry_error=data.get('telemetry_error'), navigation=data.get('navigation'),
                    encoder_diagnostics=data.get('encoder_diagnostics'), encoder_traces=data.get('encoder_traces'),
                    tracking=dict(event=vision.get('event'), age_ms=event_age,
                                  fresh=bool(nano['online'] and fresh(event_age)), error=vision.get('event_receiver_error')),
                    flow=vision.get('flow'), recording=dict(data['recording'], path=str(self.gateway.recorder.path) if self.gateway.recorder else None))


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_server(session, bind='0.0.0.0', port=8872):
    # The shared display keeps radar/flow/IMU rendering identical to the PC UI.
    page = (ROOT / 'host/index.html').read_text(encoding='utf-8')
    page = page.replace('<!-- viewer-style -->', '<style>[data-operator],[data-api]{display:none!important}</style>')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, value, status=200, kind='application/json'):
            body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (OSError, TimeoutError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path in ('/', '/home', '/imu'):
                self.reply(page.encode('utf-8'), kind='text/html; charset=utf-8')
            elif path in ('/api/state', '/health'):
                self.reply(session.state())
            elif path in ('/camera.jpg', '/infer.jpg'):
                item = session.vision.image(path == '/infer.jpg')
                if item:
                    self.reply(item[1], kind='image/jpeg')
                else:
                    self.reply(dict(error='No fresh frame'), 503)
            elif path in ('/camera.mjpg', '/infer.mjpg'):
                # Video comes directly from the owner; JSON polling cannot throttle it.
                self.send_response(302)
                self.send_header('Location', session.vision.url + path)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', '0')
                self.end_headers()

            else:
                self.reply(dict(error='Not found'), 404)

        def do_POST(self):
            self.reply(dict(error='Read-only viewer'), 403)

    return Server((bind, port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding='utf-8'))
    gateway = ViewerGateway(config, live=args.live)
    vision = VisionClient(config.get('vision_url', 'http://192.168.50.20:8766'), live=args.live)
    session = Session(gateway, vision)
    web = private = None
    private_started = False
    try:
        # Bind before starting workers, so a duplicate launch cannot touch UART.
        web = make_server(session, config.get('web_bind', '0.0.0.0'), config.get('web_port', 8872))
        token = os.environ.get('ROBOT_GATEWAY_TOKEN') or secrets.token_hex(24)
        if not gateway.receive_only:
            private = gateway_server(gateway, '127.0.0.1', config.get('port', 8873), token)
        def shutdown(*unused):
            threading.Thread(target=web.shutdown, daemon=True).start()
        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)
        gateway.start()
        if private:
            threading.Thread(target=private.serve_forever, daemon=True).start()
            private_started = True
        print('Nano viewer http://%s:%d mode=%s (read-only)' %
              (config.get('web_bind', '0.0.0.0'), web.server_port, 'live' if args.live else 'offline'), flush=True)
        web.serve_forever(poll_interval=.1)
    finally:
        session.stop.set()
        vision.close()
        gateway.close()
        if private:
            if private_started:
                private.shutdown()
            private.server_close()
        if web:
            web.server_close()


if __name__ == '__main__':
    main()
