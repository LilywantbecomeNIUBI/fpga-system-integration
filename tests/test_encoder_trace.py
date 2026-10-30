import struct,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from shared.streams import MixedStream,decode_encoder_trace
from shared.encoder_trace import TraceCollector,replay
from host.motor_protocol import crc16
from nano.gateway import Gateway

def trace(values,count=None,axis=1):
    packed=0
    for v in values:packed=(packed<<4)|v
    body=b'\xc3\x3c\x03'+bytes([axis])+struct.pack('<HBB',1,0xb2,0)+packed.to_bytes(32,'little')+struct.pack('<IH',len(values) if count is None else count,0)
    return body+struct.pack('<H',crc16(body))

class TraceTests(unittest.TestCase):
    def test_order_crc_fragmentation_and_no_control_refresh(self):
        values=[1,7,14,8];frame=trace(values);p=MixedStream();rows=[]
        for i in range(0,48,5):rows+=p.feed(frame[i:i+5])
        self.assertEqual(rows[0][0],'encoder_trace');self.assertEqual(rows[0][1]['transitions'],values)
        bad=bytearray(frame);bad[12]^=1
        with self.assertRaises(ValueError):decode_encoder_trace(bad)
        g=Gateway(dict(lidar_enabled=False));g.ingest_uart(frame);s=g.state()
        self.assertFalse(s['control']['motor_fresh']);self.assertEqual(s['encoder_traces']['1']['event_count'],4)
        self.assertEqual(s['imu']['statistics']['samples'],0)
    def test_overlap_overflow_reset_and_other_axis(self):
        c=TraceCollector(1);self.assertEqual(c.feed(decode_encoder_trace(trace([]))),[])
        self.assertEqual(c.feed(decode_encoder_trace(trace([1,7]))),[(1,1),(2,7)])
        self.assertEqual(c.feed(decode_encoder_trace(trace([1,7,14,8]))),[(3,14),(4,8)])
        self.assertEqual(c.feed(decode_encoder_trace(trace([1],axis=2))),[])
        c.feed(decode_encoder_trace(trace([1]*64,count=100)));self.assertEqual(c.gaps[0]['missing'],32)
        c.feed(decode_encoder_trace(trace([],count=0)));self.assertEqual(c.restarts,1)
    def test_clean_directions_and_incomplete_chains(self):
        pos=replay(list(enumerate([1,7,14,8]*5,1)))
        neg=replay(list(enumerate([2,11,13,4]*5,1)))
        for key in ('x4_valid_only','a_scaled2','b_scaled2','experimental_last_legal_direction'):
            self.assertEqual(pos['totals'][key],20);self.assertEqual(neg['totals'][key],-20)
        self.assertEqual(replay([(1,1),(2,8)])['chain_discontinuities'],1)
        # A diagonal by itself contains no direction information.
        self.assertEqual(replay([(1,3)])['totals']['experimental_last_legal_direction'],0)

if __name__=='__main__':unittest.main()
