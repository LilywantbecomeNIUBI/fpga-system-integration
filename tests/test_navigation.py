import json
import math
from pathlib import Path
import struct
import sys
import unittest

from host.motor_protocol import crc16
from shared.navigation_protocol import packet, upload_operations, validate_calibration, decode
from shared.telemetry_udp import HEADER, TelemetryUDP
from shared.streams import MixedStream

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'host/mecanum'))
from navigation import NavigationSession


def calibration():
    return dict(confirmed=True, front_mm=150, back_mm=150, half_width_mm=120,
                radius_mm=200, pad_mm=100, forward_mm_per_raw_count=257/65536,
                lateral_mm_per_raw_count=257/65536, rotation_lever_mm=200,
                laser_x_mm=0, laser_y_mm=0, laser_yaw_deg=0, max_speed_mm_s=100,
                accel_mm_s2=100, measured_stop_mm=40, deceleration_mm_s2=200)


class NavigationTests(unittest.TestCase):
    def test_prepare_never_arms_and_table_crc_covers_every_point(self):
        ops = upload_operations(calibration(), [[200, 0, 50], [200, 200, 50]], 9)
        self.assertEqual(len(ops), 11)
        self.assertFalse(any(o['operation'] in ('start', 'resume') for o in ops))
        self.assertEqual(ops[-1]['c'], crc16(struct.pack('<iiHiiH', 200, 0, 50, 200, 200, 50)))
        for seq, operation in enumerate(ops):
            raw = packet(sequence=seq, **operation)
            self.assertEqual(len(raw), 24)
            self.assertEqual(crc16(raw[:22]), struct.unpack_from('<H', raw, 22)[0])

    def test_uncalibrated_geometry_and_unbounded_stop_are_rejected(self):
        for key, value in [('confirmed', False), ('radius_mm', 150), ('pad_mm', 35),
                           ('forward_mm_per_raw_count', 0), ('laser_yaw_deg', float('nan')),
                           ('measured_stop_mm', 500), ('deceleration_mm_s2', 1)]:
            cfg = calibration(); cfg[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_calibration(cfg)
        for points in ([], [[4001, 0]], [[0, 0, 0]], [[0, float('nan')]], [[0, 0]]*17):
            with self.assertRaises(ValueError):
                upload_operations(calibration(), points, 9)

    def test_n4_crc_and_udp_demultiplex(self):
        body = bytearray(46); body[:3] = b'N4\x01'; body[3] = 27
        struct.pack_into('<H', body, 4, 9)
        struct.pack_into('<3i', body, 12, 65536, -131072, 0)
        frame = bytes(body)+struct.pack('<H', crc16(body))
        self.assertEqual(decode(frame)['x_mm'], 1)
        decoded = MixedStream().feed(frame)
        self.assertEqual(decoded[0][0], 'mecanum_navigation')
        self.assertEqual(decoded[0][1]['y_mm'], -2)
        udp = TelemetryUDP()
        self.assertEqual(udp.feed(HEADER.pack(b'TLM1', 1, bytes(3), 1, 100)+frame, ('192.168.50.30', 6205), 1), frame)
        corrupt = bytearray(frame); corrupt[12] ^= 1
        with self.assertRaises(ValueError):
            decode(corrupt)

    def test_no_navigation_session_on_stale_status(self):
        sent=[]
        with self.assertRaises(ValueError):
            NavigationSession(lambda *a: sent.append(a), sent.append, dict(action='start'), 10, None, None, {})
        self.assertEqual(sent, [])

    def test_upload_retries_only_identical_unarmed_packet(self):
        sent=[]; motor=dict(armed=False, faults=0, velocity=(0,0,0,0), seq=1)
        nav=dict(ack_sequence=0, ack_error=0, mission_id=9, calibrated=False)
        spec=dict(action='upload', mission_id=9, calibration=calibration(), points=[[200,0,50]])
        session=NavigationSession(lambda *a: sent.append(('motor', a)), sent.append, spec, 0,
                                  (0,motor), (0,nav), {})
        session.step(.1, (.1,motor), None, (.1,nav))
        session.step(.41, (.41,motor), None, (.41,nav))
        first=sent[-1]
        session.step(1.22, (1.22,motor), None, (1.22,nav))
        self.assertEqual(sent[-1], first)
        self.assertEqual(first[3], 0x40)
        self.assertEqual([s[1][0] for s in sent if isinstance(s,tuple)], [0])

    def test_stopping_requires_continuous_stationary_feedback(self):
        sent=[]; motor=dict(armed=False, faults=0, velocity=(0,0,0,0), seq=1)
        nav=dict(ack_sequence=0, ack_error=0, mission_id=9, calibrated=False)
        spec=dict(action='upload', mission_id=9, calibration=calibration(), points=[[200,0,50]])
        session=NavigationSession(lambda *a: sent.append(('motor',a)), sent.append, spec, 0, (0,motor), (0,nav), {})
        session.step(.1, (.1,motor), None, (.1,nav))
        moving=dict(motor, velocity=(0,1,0,0))
        session.step(.3, (.3,moving), None, (.3,nav))
        session.step(.4, (.4,motor), None, (.4,nav))
        session.step(.65, (.65,motor), None, (.65,nav))
        self.assertEqual(session.phase, 'stop')
        session.step(.71, (.71,motor), None, (.71,nav))
        self.assertEqual(session.phase, 'upload')

    def test_self_mask_never_accepts_nonfinite_or_out_of_bounds(self):
        for mask in ([0,360,50], [0,10,501], [False,10,50], [0,10,float('nan')]):
            with self.subTest(mask=mask), self.assertRaises(ValueError):
                validate_calibration(dict(calibration(), self_masks=[mask]))


if __name__ == '__main__':
    unittest.main()
