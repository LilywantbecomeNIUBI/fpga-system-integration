import sys
from pathlib import Path
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'vendor/vision'))
sys.path.insert(0, str(ROOT/'host'))
from shared.motion import MotionIntent, DifferentialDrive, constrained
from shared.perception import PerceptionEvents
from optical_flow_protocol import PACKET
from vision_v23_protocol import EVENT


def status(fid=10, flags=2, state=1, control=65280):
    return PACKET.pack(b'OFST', 1, 2, flags, state, fid, 10, 10, 10, 0, 0, 0, control, 100, 0)


class PerceptionMergeTests(unittest.TestCase):
    def test_mixed_protocol_retains_tracking_and_flow_independently(self):
        events = PerceptionEvents()
        target = EVENT.pack(b'FSTA', 2, 7, 0, 2, 10, 9, 39, 800, 10, 10, 20, 20, 400, 1000, 1, 0)
        events.ingest(target, 1)
        events.ingest(status(), 1)
        events.ingest(PACKET.pack(b'OFLW', 1, 2, 1, 0, 10, 8, 8, 65500, 0, 0, 160, 90, 100, 0), 1.01)
        self.assertTrue(events.tracking['track_valid'])
        self.assertTrue(events.flow_snapshot(1.1)['control']['fresh'])
        self.assertLess(events.flow_snapshot(1.1)['vectors'][0]['u'], 0)
        events.ingest(status(), 1.15)  # Duplicate FORWARD never extends its lease.
        stale = events.flow_snapshot(1.21)
        self.assertTrue(stale['control']['stop_request'])
        self.assertFalse(stale['control']['fresh'])
        self.assertFalse(stale['automatic_control_enabled'])
        with self.assertRaises(ValueError):events.ingest(b'????'+bytes(28), 1.22)

    def test_same_frame_stop_is_immediate(self):
        events = PerceptionEvents()
        events.ingest(status(), 1)
        events.ingest(status(flags=3, state=4, control=0), 1.01)
        self.assertTrue(events.flow_snapshot(1.02)['control']['stop_request'])

    def test_motion_adapter_units_curvature_polarity_and_expiry(self):
        chassis = DifferentialDrive(65, 180, 1500, (-1, 1), 40)
        self.assertEqual(chassis.wheel_targets(MotionIntent(100, 0, 10, 100, 'manual'), 11), (-7, 7))
        self.assertEqual(chassis.wheel_targets(MotionIntent(0, 1000, 10, 100, 'manual'), 11), (7, 7))
        self.assertEqual(chassis.wheel_targets(MotionIntent(100, 0, 10, 100, 'manual'), 110), (0, 0))
        self.assertEqual(chassis.wheel_targets(MotionIntent(100, 0, 10, 100, 'manual'), 9), (0, 0))
        a,b = chassis.wheel_targets(MotionIntent(1000, 1000, 10, 100, 'manual'), 11)
        self.assertEqual(max(abs(a), abs(b)), 40)
        self.assertGreater(b, -a)
        with self.assertRaises(ValueError):DifferentialDrive(65, None, 1500, (-1, 1), 40)

    def test_future_policy_constraints_fail_closed_and_do_not_treat_scale_as_pwm(self):
        intent = MotionIntent(1000, 100, 0, 200, 'future_policy')
        c = dict(fresh=True, confident=True, age_ms=20, stop_request=False, speed_limit=96)
        self.assertEqual(constrained(intent, c, 20), (376, 38))
        self.assertEqual(constrained(intent, dict(c, age_ms=201), 20), (0, 0))
        self.assertEqual(constrained(intent, c, 201), (0, 0))

    def test_nano_cached_flow_ages_at_pc_without_chassis_dependency(self):
        import sensing_base
        events = PerceptionEvents();events.ingest(status(), 1)
        with patch.object(sensing_base.Session, 'spawn'):
            session = sensing_base.Session()
        try:
            session.mode='live'
            class Nano:
                def state(self):return dict(online=True,poll_age_ms=210,data=dict(flow=events.flow_snapshot(1.01)))
            session.nano=Nano()
            c=session.state()['flow']['control']
            self.assertFalse(c['fresh']);self.assertTrue(c['stop_request']);self.assertEqual(c['speed_limit'],0)
        finally:session.nano=None;session.close()


if __name__ == '__main__':unittest.main()
