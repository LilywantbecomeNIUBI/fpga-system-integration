import struct,time,unittest
from unittest.mock import patch
from shared.streams import MixedStream
from shared.lidar_display import FpgaLidar
from host.motor_protocol import crc16
from nano.gateway import Gateway


def chunk(sequence,index,age=0):
    body=b'\xc3\x3c\x05\x01'+struct.pack('<HBBH',sequence,index,6,age)
    body+=b''.join(struct.pack('<HB',1000+index*12+i,100) for i in range(12))
    return body+struct.pack('<H',crc16(body))


def diagnostic(device_ms=100):
    body=b'\xc3\x3c\x06\x01'+struct.pack('<HBB9IH',1,19,0,device_ms,5000,0,100,0,0,0,2,0,72)
    return body+struct.pack('<H',crc16(body))


class DisplayTests(unittest.TestCase):
    def ingest(self,state,frame,now):
        rows=MixedStream().feed(frame)
        self.assertEqual(len(rows),1)
        state.ingest(*rows[0],now=now)

    def test_fragment_crc_and_recovery(self):
        parser=MixedStream();damaged=bytearray(chunk(1,0));damaged[12]^=1
        data=b'noise'+damaged+b''.join(chunk(1,i) for i in range(6))+diagnostic();rows=[]
        for byte in data:rows+=parser.feed(bytes([byte]))
        self.assertEqual(parser.bad_crc,1)
        self.assertEqual([kind for kind,_ in rows],['lidar_scan']*6+['lidar_diagnostic'])

    def test_missing_reordered_and_duplicate_chunks(self):
        state=FpgaLidar()
        for i in (4,2,0,0,5,1):self.ingest(state,chunk(1,i),10)
        self.assertFalse(state.snapshot(10)['online'])
        self.ingest(state,chunk(1,3),10.05)
        self.assertEqual(state.snapshot(10.05)['scan']['points'][0],[2.5,1000,100])
        for i in range(6):self.ingest(state,chunk(1,i),10.4)
        self.assertFalse(state.snapshot(10.6)['online'],'Duplicate complete frames cannot refresh age')
        for i in range(5):self.ingest(state,chunk(2,i),11)
        for i in range(6):self.ingest(state,chunk(3,i),11.1)
        self.assertEqual(state.sequence,3)
        self.assertEqual(state.incomplete,1)

    def test_source_age_wrap_reset_and_old_frames(self):
        state=FpgaLidar()
        for seq in (65535,0):
            for i in range(6):self.ingest(state,chunk(seq,i,350),20)
        self.assertTrue(state.snapshot(20.1)['online'])
        self.assertFalse(state.snapshot(20.2)['online'])
        self.ingest(state,diagnostic(100),20)
        self.ingest(state,diagnostic(1),20.3)
        self.assertIsNone(state.sequence)
        self.assertIsNone(state.snapshot(20.3)['scan'])

    def test_old_firmware_and_config_sources(self):
        state=FpgaLidar();self.ingest(state,diagnostic(),10)
        self.assertFalse(state.snapshot(10)['online'],'Receive diagnostics alone are not a scan')
        self.assertEqual(Gateway({'lidar_source':'fpga'}).state()['lidar']['source'],'fpga_m2')
        self.assertEqual(Gateway().state()['lidar']['source'],'nano_usb')
        with self.assertRaises(ValueError):Gateway({'lidar_source':'guess'})

    def test_gateway_scan_does_not_touch_imu_or_control(self):
        gateway=Gateway({'lidar_source':'fpga'})
        with patch.object(gateway.imu,'feed',side_effect=AssertionError('LiDAR routed to IMU')), \
             patch.object(gateway.control,'ingest',side_effect=AssertionError('LiDAR routed to motor')):
            gateway.ingest_uart(diagnostic()+b''.join(chunk(1,i) for i in range(6)))
        result=gateway.state()
        self.assertTrue(result['lidar']['online'])
        self.assertEqual(len(result['lidar']['scan']['points']),72)
        self.assertFalse(result['control']['motor_fresh'])

    def test_headers_inside_other_valid_packets_are_not_frames(self):
        from tools.task4.protocol import StatusStream
        # A distance/intensity tuple can legitimately contain a nav signature.
        body=bytearray(chunk(1,0)[:46]);body[10:14]=b'\xc3\x3c\x04\x01'
        outer=bytes(body)+struct.pack('<H',crc16(body))
        nav=StatusStream()
        for byte in outer: self.assertEqual(nav.feed(bytes([byte])),[])
        self.assertEqual(nav.bad_crc,0)
        # Likewise a navigation payload must not be parsed as a scan packet.
        body=bytearray(46);body[:4]=b'\xc3\x3c\x04\x01';body[10:14]=b'\xc3\x3c\x05\x01'
        outer=bytes(body)+struct.pack('<H',crc16(body))
        mixed=MixedStream();self.assertEqual(mixed.feed(outer),[])
        self.assertEqual(mixed.bad_crc,0)

    def test_static_verifier_requires_progress_and_stopped_motor(self):
        from tools.task4.verify_lidar import Check
        check=Check();self.assertFalse(check.result(10)['passed'])
        motor=b'\xc3\x3c\x01\x00\x00\x00'+struct.pack('<H',1)+struct.pack('<iiiihhIIIIH',0,0,0,0,0,0,0,0,0,0,1)
        motor+=struct.pack('<H',crc16(motor))
        check.feed(diagnostic(),10)
        for i in range(6):check.feed(chunk(1,i),10)
        later=bytearray(diagnostic(300)[:46]);struct.pack_into('<I',later,12,6000);struct.pack_into('<I',later,20,120)
        check.feed(bytes(later)+struct.pack('<H',crc16(later))+motor,10.2)
        for i in range(6):check.feed(chunk(2,i),10.2)
        self.assertTrue(check.result(10.21)['passed'])
        self.assertEqual(check.result(10.21)['commands_sent'],0)
        self.assertFalse(check.result(11)['passed'])


if __name__=='__main__':unittest.main()
