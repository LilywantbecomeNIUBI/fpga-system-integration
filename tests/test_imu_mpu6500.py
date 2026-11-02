import struct
import unittest
from shared import imu_protocol
from shared.imu_state import ImuState
from shared.streams import MixedStream


def packet(who=0x70, config=6, seq=1):
    body = imu_protocol.BODY.pack(b'\xa5\x5a', 2, 1, seq*20, seq*20000,
        8192, -4096, 8192, 3280, -3280, 0, 334, 0x29, who, 0, config,
        0, 0, 0, seq, 440, 1000, 20, 150)
    return body + struct.pack('>H', imu_protocol.crc16(body))


class Mpu6500Tests(unittest.TestCase):
    def test_units_temperature_and_identity(self):
        rows = MixedStream().feed(packet())
        self.assertEqual(len(rows), 1)
        kind, row = rows[0]
        self.assertEqual(kind, 'imu')
        self.assertEqual(row['config_id'], 6)
        self.assertEqual((row['ax_g'], row['ay_g']), (1, -.5))
        self.assertAlmostEqual(row['gx_dps'], 100)
        self.assertAlmostEqual(row['temp_c'], 21 + 334/333.87)
        state = ImuState(); state.feed(packet(), now=1)
        self.assertTrue(state.snapshot(now=1)['healthy'])
        for who, config in [(0x68, 6), (0x70, 4), (0x47, 6)]:
            state = ImuState(); state.feed(packet(who, config), now=1)
            self.assertFalse(state.snapshot(now=1)['healthy'])

    def test_read_only_capture_acceptance(self):
        from tools.verify_mpu6050 import evaluate
        stream = b''.join(packet(seq=n) for n in range(1, 7))
        self.assertTrue(evaluate(stream, require_motor_telemetry=False)['passed'])
