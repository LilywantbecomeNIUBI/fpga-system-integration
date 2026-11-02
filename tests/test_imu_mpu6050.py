import struct
import unittest
from shared import imu_protocol
from shared.imu_state import ImuState
from shared.streams import MixedStream

def packet(config=4, who=0x68, flags=0x29, seq=1, kind=1):
    body=imu_protocol.BODY.pack(b'\xa5\x5a',2,kind,seq*20,seq*20000,8192,-4096,0,3280,-3280,0,340,
        flags,who,0,config,0,0,0,seq,440,1000,20,150)
    return body+struct.pack('>H',imu_protocol.crc16(body))

class Mpu6050Tests(unittest.TestCase):
    def test_units_profile_and_gateway_health(self):
        rows=MixedStream().feed(packet())
        self.assertEqual(len(rows),1)
        kind,row=rows[0]
        self.assertEqual(kind,'imu')
        self.assertEqual(row['config_id'],4)
        self.assertEqual((row['ax_g'],row['ay_g']),(1,-.5))
        self.assertAlmostEqual(row['gx_dps'],100)
        self.assertAlmostEqual(row['gy_dps'],-100)
        self.assertAlmostEqual(row['temp_c'],37.53)
        self.assertFalse(row['fast_bus'])
        state=ImuState();state.feed(packet(),now=1)
        snapshot=state.snapshot(now=1.1)
        self.assertTrue(snapshot['healthy'] and snapshot['attitude']['valid'])
        self.assertFalse(state.snapshot(now=2)['attitude']['valid'])

    def test_profile_identity_pairs_and_stale(self):
        for config,who,expected in ((2,0x47,True),(3,0x47,True),(4,0x68,True),(5,0x68,True),
                                    (4,0x47,False),(2,0x68,False),(4,0x69,False)):
            state=ImuState();state.feed(packet(config,who),now=1)
            self.assertEqual(state.snapshot(now=1)['healthy'],expected)
        for flags in (0x2d,0x2b):
            state=ImuState();state.feed(packet(flags=flags),now=1)
            self.assertFalse(state.snapshot(now=1)['healthy'])

    def test_read_only_acceptance_rejects_gaps_late_faults_and_short_capture(self):
        from tools.verify_mpu6050 import evaluate
        body=b'\xc3\x3c\x01\x00\x00\x00'+struct.pack('<H',1)+struct.pack('<iiiihhIIIIH',1,2,3,4,0,0,0,0,0,0,1)
        motor=body+struct.pack('<H',imu_protocol.crc16(body))
        stream=motor+b''.join(packet(seq=n) for n in range(1,7))
        self.assertTrue(evaluate(stream)['passed'])
        self.assertFalse(evaluate(stream,min_duration_s=1)['passed'])
        self.assertFalse(evaluate(stream+packet(flags=0x2b,kind=2,seq=7))['passed'])
        self.assertFalse(evaluate(stream+packet(seq=8))['passed'])
        self.assertFalse(evaluate(stream[len(motor):])['passed'])
        self.assertTrue(evaluate(stream[len(motor):],require_motor_telemetry=False)['passed'])
        body=bytearray(motor[:46]);body[4]=1
        moving=bytes(body)+struct.pack('<H',imu_protocol.crc16(body))
        self.assertFalse(evaluate(moving+stream[len(motor):],require_motor_telemetry=False)['passed'])
