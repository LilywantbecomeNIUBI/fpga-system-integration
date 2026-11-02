import sys
from pathlib import Path
import struct
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from console import packet,crc16,Stream,mm_to_control_counts,motor_profile


class HostTests(unittest.TestCase):
    def test_crc_reference(self):
        self.assertEqual(crc16(b'123456789'),0x29b1)

    def test_frame(self):
        data=packet(6,255,motor=0,encoder=0,x=-123,y=24,z=-8,kp=512,ki=16)
        self.assertEqual(len(data),24)
        self.assertEqual(data[:8],bytes.fromhex('a55a0306ff000000'))
        self.assertEqual(struct.unpack_from('<3h2H',data,8),(-123,24,-8,512,16))
        self.assertEqual(crc16(data[:22]),int.from_bytes(data[22:],'little'))

    def test_validation(self):
        for kwargs in [dict(x=16001),dict(motor=1),dict(encoder=1),dict(kp=0),dict(ki=1025)]:
            with self.assertRaises(ValueError):packet(6,1,**kwargs)

    def test_removed_opcodes_rejected(self):
        for op in [2,4,5,255]:
            with self.assertRaises(ValueError):packet(op,1)

    def test_stream(self):
        body=bytearray(46);body[:4]=b'M4\x03\x01';body[6]=19
        struct.pack_into('<4i4h4h',body,8,-12345,2,3,4,-12,13,14,15,-100,101,102,103)
        frame=body+struct.pack('<H',crc16(body))
        parser=Stream();bad=bytearray(frame);bad[10]^=1
        self.assertEqual(parser.feed(b'noise'+bad+frame[:20]),[])
        rows=parser.feed(frame[20:]+frame)
        self.assertEqual(len(rows),2)
        self.assertEqual(rows[0]['position'][0],-12345)
        self.assertEqual(rows[0]['velocity'][0],-12)
        self.assertEqual(rows[0]['pwm'][0],-100)

    def test_outer_packet(self):
        data=packet(6,4,x=15000,y=-200,z=9000,limit=40,tolerance=8)
        self.assertEqual(struct.unpack_from('<3h',data,8),(15000,-200,9000))
        self.assertEqual(struct.unpack_from('<2H',data,18),(40,8))
        heartbeat=packet(7,5)
        self.assertEqual(heartbeat[18:22],b'\0'*4)
        for kwargs in [dict(op=6,x=16001),dict(op=5,x=129),dict(op=6,z=18001),dict(op=6,limit=0),dict(op=6,tolerance=0)]:
            with self.assertRaises(ValueError):packet(seq=1,**kwargs)

    def test_pose_stream(self):
        body=bytearray(46);body[:4]=b'P4\x03\x17';body[6]=10
        struct.pack_into('<4i',body,8,100*65536,-50*65536,102944,-102944)
        struct.pack_into('<hhI',body,24,100,1310,3000000)
        body[42]=0x68
        data=body+struct.pack('<H',crc16(body))
        row=Stream().feed(data)[0]
        self.assertEqual(row['kind'],'pose');self.assertTrue(row['healthy'] and row['arrived'])
        self.assertEqual(row['mode'],2);self.assertEqual(row['x_counts'],100)
        self.assertAlmostEqual(row['heading_deg'],90,places=2)
        self.assertEqual(row['who'],0x68)

    def test_imu_bus_diagnostics(self):
        body=bytearray(46);body[:4]=b'P4\x03\x03';body[33]=1;body[42]=0x70
        struct.pack_into('<H',body,38,0x1234);body[40]=0x31;body[41]=15
        row=Stream().feed(body+struct.pack('<H',crc16(body)))[0]
        self.assertEqual((row['imu_diag_version'],row['imu_fault_count'],row['imu_bus_error'],row['imu_bus_lines']),(1,0x1234,1,3))
        self.assertTrue(row['imu_fault_armed'] and row['imu_fault_pwm'] and row['imu_fault_outer'] and row['imu_stale'])

    def test_gmr_profile_and_units(self):
        profile=motor_profile()
        self.assertEqual(profile['raw_counts_per_rev'],4*500*30)
        self.assertEqual(mm_to_control_counts(200,60000,75,4),3183)
        self.assertEqual(mm_to_control_counts(-200,60000,75,4),-3183)
        self.assertEqual(mm_to_control_counts(1000,60000,75,4),15915)
        for n,d in [(0,75),(60000,0),(-1,75)]:
            with self.assertRaises(ValueError):mm_to_control_counts(100,n,d,4)

    def test_legacy_frames_not_misinterpreted(self):
        body=bytearray(46);body[:4]=b'M4\x02\x01'
        old=body+struct.pack('<H',crc16(body))
        self.assertEqual(Stream().feed(old),[])


if __name__=='__main__':unittest.main()
