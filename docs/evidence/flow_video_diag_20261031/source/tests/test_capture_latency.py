import sys,struct,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'vendor/vision')]
from shared.live_capture import EpochAssembler
from shared.perception import PerceptionEvents
from optical_flow_protocol import PACKET

def packet(fid):
    return struct.pack('<2sBBHHHHHHH',b'CV',2,1,fid,0,0,1,1,1,0x1234)

class CaptureLatencyTests(unittest.TestCase):
    def test_reset_after_silence_with_id_above_32(self):
        a=EpochAssembler();self.assertIsNotNone(a.push(packet(20000),now=0))
        self.assertIsNone(a.push(packet(80),now=2))
        self.assertEqual(a.last_completed,20000)
        self.assertIsNotNone(a.push(packet(81),now=2.04))
        self.assertEqual(a.last_completed,81);self.assertEqual(a.reset_events,1)

    def test_delayed_one_frame_does_not_replace_current_epoch(self):
        a=EpochAssembler();a.push(packet(20000),now=0)
        self.assertIsNone(a.push(packet(80),now=2))
        self.assertIsNotNone(a.push(packet(20001),now=2.04))
        self.assertEqual(a.last_completed,20001);self.assertEqual(a.reset_events,0)

    def test_regular_wrap_is_not_reset(self):
        a=EpochAssembler();a.push(packet(65535),now=0)
        self.assertIsNotNone(a.push(packet(0),now=.04));self.assertEqual(a.reset_events,0)

    def test_reset_when_old_id_is_above_half_range(self):
        a=EpochAssembler();a.push(packet(42000),now=0)
        self.assertIsNone(a.push(packet(80),now=2))
        self.assertIsNotNone(a.push(packet(81),now=2.04))
        self.assertEqual(a.last_completed,81);self.assertEqual(a.reset_events,1)
        self.assertEqual(a.missing_frames,0)

    def test_diagnostic_isolated_from_control_and_tracking(self):
        p=struct.pack('!4s4B3H4B2H2B2I',b'OFDG',1,2,1,2,123,40,32,14,63,8,2,65535,65535,71,70,10000000,44)
        consumer=PerceptionEvents();kind,event=consumer.ingest(p,now=1)
        self.assertEqual(kind,'diagnostic');self.assertEqual(event['patch_state'],14)
        self.assertEqual(event['patch_starts']-event['patch_completions'],1)
        s=consumer.flow_snapshot(now=1.1)
        self.assertFalse(s['control']['fresh']);self.assertTrue(s['control']['stop_request'])
        self.assertIsNone(consumer.tracking);self.assertAlmostEqual(s['diagnostic']['age_ms'],100)
        with self.assertRaises(ValueError):consumer.ingest(p[:-1])

    def test_sticky_solver_fault_masks_advice_until_reset_report(self):
        c=PerceptionEvents()
        status=PACKET.pack(b'OFST',1,2,2,1,123,4,4,4,0,0,0,255<<8,1000,0)
        c.ingest(status,now=1);self.assertTrue(c.flow_snapshot(now=1.01)['control']['fresh'])
        diagnostic=struct.pack('!4s4B3H4B2H2B2I',b'OFDG',1,2,0,0,123,8,8,0,63,0,128,0,0,1,1,1000,0)
        c.ingest(diagnostic,now=1.02);snapshot=c.flow_snapshot(now=1.03)
        self.assertTrue(snapshot['health_fault']);self.assertTrue(snapshot['control']['stop_request'])
        self.assertFalse(snapshot['control']['fresh']);self.assertEqual(snapshot['control']['speed_limit'],0)

if __name__=='__main__':unittest.main()
