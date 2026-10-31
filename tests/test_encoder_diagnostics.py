import struct,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'host')]
from shared.streams import MixedStream,decode_encoder_diagnostics
from host.motor_protocol import crc16
from nano.gateway import Gateway
from shared.encoder_analysis import difference

def diagnostic(axis=1,sequence=9,mode=0xa2):
    body=b'\xc3\x3c\x02'+bytes([axis])+struct.pack('<HBB9IH',sequence,mode,0b1001,*range(10,19),0)
    return body+struct.pack('<H',crc16(body))

class DiagnosticTests(unittest.TestCase):
    def test_counter_analysis_direction_cancellation_and_reset(self):
        base=decode_encoder_diagnostics(diagnostic())
        end=dict(base,sequence=11)
        end.update(a_positive=base['a_positive']+10,a_negative=base['a_negative']+10,
                   b_negative=base['b_negative']+20)
        delta=difference(base,end)
        self.assertEqual(delta['a_scaled_net'],0)
        self.assertEqual(delta['a_cancellation_fraction'],1)
        self.assertEqual(delta['b_scaled_net'],-40)
        self.assertEqual(delta['b_cancellation_fraction'],0)
        with self.assertRaises(ValueError):difference(base,dict(end,sequence=0))
    def test_wire_order_crc_and_fragmentation(self):
        frame=diagnostic();self.assertEqual(len(frame),48)
        parser=MixedStream();rows=[]
        for i in range(0,len(frame),3):rows+=parser.feed(frame[i:i+3])
        self.assertEqual(len(rows),1);kind,row=rows[0]
        self.assertEqual(kind,'encoder_diagnostics')
        self.assertEqual((row['axis'],row['sequence'],row['synchronized_ab'],row['filtered_ab']),(1,9,2,1))
        self.assertEqual((row['sync_a_edges'],row['diagonal']),(10,18))
        self.assertEqual(decode_encoder_diagnostics(diagnostic(mode=0xb2))['mode'],'b_edge_x2_scaled4')
        self.assertEqual(decode_encoder_diagnostics(diagnostic(mode=0xd4))['mode'],'quadrature_continuity_x4')
        bad=bytearray(frame);bad[11]^=1
        self.assertEqual(parser.feed(bad),[]);self.assertEqual(parser.bad_crc,1)
        with self.assertRaises(ValueError):decode_encoder_diagnostics(bad)
    def test_invalid_axis_or_mode_does_not_kill_stream(self):
        parser=MixedStream();rows=parser.feed(diagnostic(axis=3)+diagnostic(mode=0)+diagnostic(axis=2))
        self.assertEqual(len(rows),1);self.assertEqual(rows[0][1]['axis'],2)
    def test_diagnostics_do_not_refresh_motor_or_feed_imu(self):
        gateway=Gateway(dict(lidar_enabled=False),live=False)
        gateway.ingest_uart(diagnostic())
        state=gateway.state()
        self.assertFalse(state['control']['motor_fresh'])
        self.assertIsNone(state['control']['motor'])
        self.assertEqual(state['imu']['statistics']['samples'],0)
        self.assertEqual(state['encoder_diagnostics']['1']['a_positive'],14)
        self.assertGreaterEqual(state['encoder_diagnostics']['1']['age_ms'],0)

if __name__=='__main__':unittest.main()
