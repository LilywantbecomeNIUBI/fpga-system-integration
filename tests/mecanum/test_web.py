"""Offline browser lease and command admission regressions; no hardware IO."""
import importlib.util
import json
from pathlib import Path
import queue
import sys
import threading
import unittest
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('mecanum_web', ROOT / 'host/mecanum/web.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)


class Link:
    def __init__(self):
        self.requests = queue.Queue()
        self.close_requested = threading.Event()

    def is_alive(self):
        return True


class WebTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.console = web.Console(clock=lambda: self.now)
        self.console.link = Link()
        self.console.connected = True
        self.client = 'a' * 32

    def action(self, action, **kwargs):
        return self.console.action(dict(action=action, client=self.client, **kwargs))

    def test_open_and_read_never_send_commands(self):
        self.console.state()
        self.console.tick()
        self.assertTrue(self.console.link.requests.empty())

    def test_lease_is_exclusive_and_heartbeat_cannot_acquire(self):
        with self.assertRaises(ValueError):
            self.action('heartbeat')
        self.action('acquire')
        with self.assertRaises(ValueError):
            self.console.action(dict(action='acquire', client='b' * 32))
        self.assertTrue(self.console.state(self.client)['operator'])

    def test_browser_loss_stops_and_cannot_resume_with_heartbeat(self):
        self.action('acquire')
        self.now += .61
        self.console.tick()
        self.assertEqual(self.console.link.requests.get_nowait(), ('stop', None))
        self.assertIsNone(self.console.owner)
        with self.assertRaises(ValueError):
            self.action('heartbeat')

    def test_heartbeat_renews_lease_without_motor_commands(self):
        self.action('acquire')
        self.now += .4
        self.action('heartbeat')
        self.now += .4
        self.console.tick()
        self.assertIsNotNone(self.console.owner)
        self.assertTrue(self.console.link.requests.empty())

    def test_stop_revokes_owner_even_from_other_page(self):
        self.action('acquire')
        self.console.action(dict(action='stop', client='another'))
        self.assertIsNone(self.console.owner)
        self.assertEqual(self.console.link.requests.get_nowait()[0], 'stop')

    def test_motion_requires_lease_and_rejects_nonfinite_or_out_of_range(self):
        with self.assertRaises(ValueError):
            self.action('motion', kind='定角', angle=360)
        self.action('acquire')
        for key, value in [('dx', float('nan')), ('angle', float('inf')), ('angle', 721), ('dx', 4001)]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.action('motion', kind='位置', **{key: value})
        self.assertTrue(self.console.link.requests.empty())

    def test_extended_turn_uses_existing_directed_handshake(self):
        self.action('acquire')
        self.action('motion', kind='定角', angle=360)
        kind, spec = self.console.link.requests.get_nowait()
        self.assertEqual(kind, 'start')
        self.assertEqual(spec['op'], 9)
        self.assertEqual(spec['values'], (0, 0, 3600))
        with self.assertRaises(ValueError):
            self.action('motion', kind='定角', angle=90)

    def test_navigation_needs_fresh_status_and_blocks_task2_while_owned(self):
        self.action('acquire')
        with self.assertRaises(ValueError):
            self.action('mode', mode='navigation')
        self.console.frames['navigation']=(self.now,dict(active=False, calibrated=False, mission_id=9))
        self.action('mode', mode='navigation')
        kind,spec=self.console.link.requests.get_nowait()
        self.assertEqual((kind,spec['kind'],spec['action']), ('start','navigation','mode'))
        self.console.pending=False
        with self.assertRaises(ValueError):
            self.action('motion', kind='定角', angle=90)
        self.console.mode='task2'
        self.console.frames['navigation']=(self.now,dict(active=True, calibrated=True, mission_id=9))
        with self.assertRaises(ValueError):
            self.action('calibrate')
        with self.assertRaises(ValueError):
            self.action('motion', kind='定角', angle=90)

    def test_firmware_reset_disconnect_does_not_reacquire(self):
        self.action('acquire')
        self.console.events.put(('disconnected', None))
        self.console.tick()
        self.assertFalse(self.console.connected)
        self.assertIsNone(self.console.owner)

    def test_existing_motion_session_rejects_stale_telemetry_before_arm(self):
        sent = []
        spec = web.gui.motion_spec('定角', 0, 0, 360, 6, 4, 60, dict(motor=0, encoder=0, kp=2048, ki=128), self.console.profile)
        with self.assertRaises(ValueError):
            web.gui.MotionSession(lambda *args: sent.append(args), spec, 10,
                                  (9., dict(control_shift=6, capabilities=3)), None, 6)
        self.assertEqual(sent, [])

    def test_http_requires_token_and_same_origin_and_valid_host(self):
        server = web.make_server(self.console, 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        url = 'http://127.0.0.1:%s' % server.server_port
        try:
            with opener.open(url) as response:
                page = response.read().decode()
            token = page.split("const token='")[1].split("'")[0]
            for headers in ({}, {'X-Console-Token': token, 'Origin': 'http://evil.example'}, {'X-Console-Token': token, 'Host': 'evil.example'}):
                request = urllib.request.Request(url + '/api/action', data=b'{"action":"stop"}', headers=headers)
                with self.assertRaises(urllib.error.HTTPError) as ctx:
                    opener.open(request)
                self.assertEqual(ctx.exception.code, 403)
            self.assertTrue(self.console.link.requests.empty())
            request = urllib.request.Request(url + '/api/action', data=b'{"action":"stop"}', headers={'X-Console-Token': token})
            with opener.open(request) as response:
                self.assertTrue(json.load(response)['ok'])
            self.assertEqual(self.console.link.requests.get_nowait()[0], 'stop')
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
