import ast
import http.client
import base64
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path, PurePosixPath
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from nano.web_console import Gateway, ViewerGateway, Session, VisionClient, make_server
from nano.preview import LatestPreview
from tools.install_nano_web import service

JPEG = b'\xff\xd8test-only-jpeg\xff\xd9'


class NanoWebTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.
        self.vision = VisionClient('http://127.0.0.1:8766', clock=lambda: self.clock)

    def fixture(self):
        return dict(profile='fpga_yolo_dashboard_v1', offline=False, error=None,
                    capture=dict(age_ms=10, frames=20, fps=30, frame_id=4),
                    result=dict(session='test', sequence=1, age_ms=20, jpeg_b64=base64.b64encode(JPEG).decode()),
                    event=dict(track_valid=True), event_age_ms=30,
                    flow=dict(vectors=[dict(valid=True)], control=dict(age_ms=15, fresh=True, state='left')))

    def test_cached_results_age_and_never_publish_jpeg_in_state(self):
        data = self.fixture()
        def get(path, limit):
            return JPEG if path == '/camera.jpg' else json.dumps(data).encode()
        with patch.object(self.vision, 'get', side_effect=get) as upstream:
            self.vision.poll()
            for _ in range(30):
                snapshot, capture = self.vision.snapshot()
                self.assertTrue(snapshot['result_fresh'])
                self.assertTrue(capture['online'])
                self.assertNotIn('jpeg_b64', snapshot['data']['result'])
                self.assertEqual(self.vision.image(True)[1], JPEG)
            self.assertEqual(upstream.call_count, 2, 'Viewers must share one upstream poll')
            self.clock += .25
            snapshot, _ = self.vision.snapshot()
            self.assertFalse(snapshot['data']['flow']['control']['fresh'])
            self.assertEqual(snapshot['data']['flow']['vectors'], [])
            self.clock += .3
            self.assertFalse(self.vision.snapshot()[0]['online'])
            self.assertIsNone(self.vision.image())
            self.assertIsNone(self.vision.image(True))

    def test_conditional_result_reuses_exact_image_and_disconnect_clears(self):
        responses = []
        def get(path, limit):
            if path == '/camera.jpg':
                return JPEG
            responses.append(path)
            data = self.fixture()
            if len(responses) > 1:
                data['result'].pop('jpeg_b64')
            return json.dumps(data).encode()
        with patch.object(self.vision, 'get', side_effect=get):
            self.vision.poll()
            self.vision.poll()
            self.assertIn('after=test-1', responses[-1])
            self.assertEqual(self.vision.image(True)[1], JPEG)
        with patch.object(self.vision, 'get', side_effect=OSError('disconnected')):
            self.vision.poll()
        self.assertIsNone(self.vision.image(True))
        self.assertEqual(self.vision.snapshot()[0]['error'], 'disconnected')

    def test_offline_fixture_and_wrong_profile_never_show_live_video(self):
        data = self.fixture()
        data['offline'] = True
        with patch.object(self.vision, 'get', return_value=json.dumps(data).encode()):
            self.vision.poll()
        self.assertFalse(self.vision.snapshot()[0]['result_fresh'])
        self.assertIsNone(self.vision.image())
        with patch.object(self.vision, 'get', return_value=b'{"profile":"wrong"}'):
            self.vision.poll()
        self.assertFalse(self.vision.snapshot()[0]['online'])

    def test_real_upstream_and_multiple_http_viewers_are_read_only(self):
        from http.server import BaseHTTPRequestHandler, HTTPServer
        owner = self
        class Upstream(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass
            def do_GET(self):
                body = JPEG if self.path == '/camera.jpg' else json.dumps(owner.fixture()).encode()
                self.send_response(200)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        upstream = HTTPServer(('127.0.0.1', 0), Upstream)
        thread = threading.Thread(target=upstream.serve_forever, daemon=True)
        thread.start()
        self.vision.url = 'http://127.0.0.1:' + str(upstream.server_port)
        self.vision.poll()
        gateway = Gateway()
        session = Session(gateway, self.vision)
        web = make_server(session, '127.0.0.1', 0)
        worker = threading.Thread(target=web.serve_forever, daemon=True)
        worker.start()
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        url = 'http://127.0.0.1:' + str(web.server_port)
        def state(index):
            with opener.open(url + '/api/state', timeout=2) as response:
                return json.load(response)
        try:
            with ThreadPoolExecutor(max_workers=6) as clients:
                snapshots = list(clients.map(state, range(12)))
            self.assertTrue(all(s['read_only'] and s['host'] == 'nano' for s in snapshots))
            self.assertIsNone(gateway.control.owner)
            for path in ('/api/control', '/api/shutdown', '/api/record/start'):
                with self.assertRaises(urllib.error.HTTPError) as failure:
                    opener.open(urllib.request.Request(url + path, data=b'{"action":"claim"}'))
                self.assertEqual(failure.exception.code, 403)
            for path in ('/camera.jpg', '/infer.jpg'):
                with opener.open(url + path) as response:
                    self.assertEqual(response.read(), JPEG)
            connection = http.client.HTTPConnection('127.0.0.1', web.server_port)
            for path in ('/camera.mjpg', '/infer.mjpg'):
                connection.request('GET', path)
                response = connection.getresponse()
                self.assertEqual(response.status, 302)
                self.assertEqual(response.getheader('Location'), self.vision.url + path)
                response.read()
            connection.close()
            with opener.open(url + '/') as response:
                html = response.read().decode('utf8')
                self.assertIn('[data-operator],[data-api]{display:none!important}', html)
            self.assertIsNone(gateway.control.owner)
        finally:
            session.stop.set()
            web.shutdown(); web.server_close(); worker.join()
            upstream.shutdown(); upstream.server_close(); thread.join()

    def test_preview_encodes_once_for_many_clients_and_expires(self):
        class Capture:
            def frame(inner):
                return (2, object(), 10., None)
        calls = []
        preview = LatestPreview(Capture(), lambda image: calls.append(image) or JPEG, clock=lambda: self.clock)
        for _ in range(10):
            self.assertEqual(preview.image(), JPEG)
        self.assertEqual(len(calls), 1)
        self.clock += .51
        self.assertIsNone(preview.image())

    def test_preview_follows_new_frames_without_ten_fps_throttle(self):
        class Capture:
            key = 0
            def frame(inner):
                return (inner.key, object(), self.clock, None)
        capture = Capture()
        calls = []
        preview = LatestPreview(capture, lambda image: calls.append(image) or JPEG,
                                clock=lambda: self.clock)
        for i in range(30):
            capture.key = i
            self.clock = 10 + i/30
            self.assertEqual(preview.image(), JPEG)
            self.assertEqual(preview.image(), JPEG)
        self.assertEqual(len(calls), 30)

    def test_stopped_upstream_rates_expire_even_when_http_stays_online(self):
        data = self.fixture()
        data['inference_fps'] = 24
        data['capture']['receiver_fps'] = 30
        data['capture']['age_ms'] = 600
        data['result']['age_ms'] = 600
        with patch.object(self.vision, 'get', return_value=json.dumps(data).encode()):
            self.vision.poll()
        state, capture = self.vision.snapshot()
        self.assertTrue(state['online'])
        self.assertEqual(capture['fps'], 0)
        self.assertEqual(capture['receiver_fps'], 0)
        self.assertEqual(state['data']['inference_fps'], 0)
        self.assertEqual(capture['preview_fps'], 0)

    def test_offline_launcher_requires_no_desktop_dependencies_and_exits_cleanly(self):
        with tempfile.TemporaryDirectory() as temp:
            ports = []
            for _ in range(2):
                sock = socket.socket(); sock.bind(('127.0.0.1', 0))
                ports.append(sock)
            config = Path(temp) / 'config.json'
            config.write_text(json.dumps(dict(web_bind='127.0.0.1', web_port=ports[0].getsockname()[1],
                                               port=ports[1].getsockname()[1])), encoding='utf8')
            code = "import sys,runpy;sys.modules['cv2']=None;sys.modules['numpy']=None;script=sys.argv[2];sys.argv=['web','--config',sys.argv[1]];runpy.run_path(script,run_name='__main__')"
            for sock in ports:
                sock.close()
            process = subprocess.Popen([sys.executable, '-I', '-c', code, str(config), str(ROOT/'nano/web_console.py')],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                target = 'http://127.0.0.1:' + str(json.loads(config.read_text())['web_port']) + '/health'
                deadline = time.monotonic() + 5
                while True:
                    try:
                        with opener.open(target, timeout=.2) as response:
                            self.assertEqual(json.load(response)['mode'], 'offline')
                        break
                    except OSError:
                        if process.poll() is not None or time.monotonic() > deadline:
                            self.fail('Offline subprocess failed: ' + process.communicate(timeout=1)[1])
                        time.sleep(.05)
                if sys.platform != 'win32':
                    process.terminate()
                    self.assertEqual(process.wait(timeout=5), 0)
            finally:
                if process.poll() is None:
                    process.terminate()
                process.communicate(timeout=5)

    def test_boot_service_uses_nano_paths_and_python36_syntax(self):
        unit = service(PurePosixPath('/home/liwei/robot_web'), PurePosixPath('/home/liwei/robot_web/config/nano.web.local.json'),
                       'liwei', 'nano/web_console.py', live=True)
        self.assertIn('User=liwei', unit)
        self.assertIn('network-online.target', unit)
        self.assertIn('WorkingDirectory=/home/liwei/robot_web\n', unit)
        self.assertIn('"--live"', unit)
        self.assertNotIn('192.168.50.10', unit)
        for folder in ('nano', 'shared'):
            for path in (ROOT/folder).rglob('*.py'):
                ast.parse(path.read_text(encoding='utf-8'), feature_version=(3, 6))

    def test_udp_observer_needs_no_uart_and_cannot_send_commands(self):
        with tempfile.TemporaryDirectory() as temp:
            gateway = ViewerGateway(dict(receive_only=True, telemetry_transport='udp', lidar_source='fpga', record_dir=temp), live=True)
            try:
                with patch.object(gateway, 'serial_loop') as serial, patch.object(gateway, 'telemetry_loop'), patch.object(gateway, 'record_loop'):
                    gateway.start()
                    for worker in gateway.threads:
                        worker.join(1)
                    self.assertFalse(serial.called)
                    self.assertEqual(gateway.serial_connections, 0)
                    self.assertFalse(gateway.state()['control']['motor_fresh'])
                    with self.assertRaises(ValueError):
                        gateway.control.request(dict(action='claim'))
            finally:
                gateway.close()


if __name__ == '__main__':
    unittest.main()
