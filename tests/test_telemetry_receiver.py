import socket
import time
import unittest

from nano.telemetry_receiver import TelemetryReceiver
from shared.telemetry_udp import HEADER
from test_lidar_raw import point_frame


class ReceiverTests(unittest.TestCase):
    def test_real_udp_process_publishes_complete_points_and_expires_on_exit(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.bind(('127.0.0.1', 0))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as available:
                available.bind(('127.0.0.1', 0))
                port = available.getsockname()[1]
            receiver = TelemetryReceiver(dict(receive_only=True, record_telemetry=False,
                telemetry_transport='udp', lidar_source='fpga', telemetry_bind='127.0.0.1',
                telemetry_port=port, fpga_ip='127.0.0.1', telemetry_source_port=sender.getsockname()[1]))
            receiver.start()
            try:
                deadline = time.monotonic()+5
                while receiver.state() is None and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertIsNotNone(receiver.state(), 'receiver failed to bind UDP')
                started = time.monotonic()
                for seq in range(1, 122):
                    tick = int((time.monotonic()-started)*1000)
                    for half in (0, 1):
                        packet = HEADER.pack(b'TLM1', 1, bytes(3), seq*2+half, tick) + point_frame(seq, half, capture=tick)
                        sender.sendto(packet, ('127.0.0.1', port))
                    time.sleep(.001)
                deadline = time.monotonic()+.4
                state = receiver.state()
                while not state['lidar']['online'] and time.monotonic() < deadline:
                    time.sleep(.01)
                    state = receiver.state()
                self.assertTrue(state['lidar']['online'])
                self.assertEqual(len(state['lidar']['scan']['points']), 360)
                self.assertFalse(state['recording']['active'])
                self.assertTrue(state['telemetry']['receiver_process'])
                receiver.process.terminate()
                receiver.process.join(2)
                self.assertFalse(receiver.state()['lidar']['online'])
                self.assertIsNone(receiver.state()['lidar']['scan'])
            finally:
                receiver.close()


if __name__ == '__main__':
    unittest.main()
