import struct
import unittest
from host.motor_protocol import crc16
from shared.telemetry_udp import HEADER, TelemetryUDP
from shared.lidar_display import FpgaLidar
from shared.streams import MixedStream
from nano.gateway import Gateway

PEER = ('192.168.50.30', 6205)


def scan_frame(seq, index, age=0, capture=100):
    body = b'\xc3\x3c\x05\x02'+struct.pack('<HBBHIBB', seq,index,60,age,capture,6,1)
    body += b''.join(struct.pack('<HHB',(index*6+i)*100+37,1000+index*6+i,77) for i in range(6))
    return body+struct.pack('<H',crc16(body))


def datagram(frame, seq=1, tick=100):
    return HEADER.pack(b'TLM1',1,bytes(3),seq,tick)+frame


class UDPTests(unittest.TestCase):
    def test_source_crc_sequence_and_no_refresh_by_duplicates(self):
        udp = TelemetryUDP()
        data = datagram(scan_frame(1,0))
        self.assertIsNone(udp.feed(data,('192.168.50.31',6205),1))
        self.assertIsNone(udp.feed(data[:-1],PEER,1))
        bad = bytearray(data);bad[-1] ^= 1
        self.assertIsNone(udp.feed(bad,PEER,1))
        self.assertEqual(udp.feed(data,PEER,1),data[16:])
        self.assertIsNone(udp.feed(data,PEER,1.1))
        self.assertEqual(udp.received,1)
        self.assertIsNone(udp.feed(datagram(scan_frame(1,0),0,99),PEER,1.2))
        self.assertFalse(udp.restart_required,'ordinary reordering must not look like a restart')
        self.assertEqual(udp.feed(datagram(scan_frame(1,1),3,200),PEER,1.1),scan_frame(1,1))
        self.assertEqual(udp.missing,1)

    def test_wrap_stale_packets_and_reset_require_reconnect(self):
        udp = TelemetryUDP();frame=scan_frame(1,0)
        self.assertIsNotNone(udp.feed(datagram(frame,0xffffffff,0xfffffff0),PEER,1))
        self.assertIsNotNone(udp.feed(datagram(frame,0,84),PEER,1.1))
        self.assertIsNone(udp.feed(datagram(frame,1,85),PEER,2))
        self.assertEqual(udp.stale,1)
        self.assertIsNone(udp.feed(datagram(frame,2,1),PEER,2.1))
        self.assertTrue(udp.restart_required)

    def test_360_scan_needs_all_chunks_and_preserves_actual_angle(self):
        lidar=FpgaLidar();decoder=MixedStream()
        for index in list(range(59,0,-1)):
            lidar.ingest(*decoder.feed(scan_frame(65535,index))[0],now=1)
        self.assertFalse(lidar.snapshot(1)['online'])
        lidar.ingest(*decoder.feed(scan_frame(65535,0))[0],now=1.01)
        scan=lidar.snapshot(1.02)['scan']
        self.assertEqual(len(scan['points']),360)
        self.assertEqual(scan['points'][0],[.37,1000,77])
        self.assertEqual(scan['points'][-1],[359.37,1359,77])
        self.assertEqual(scan['resolution_degrees'],1)
        for i in range(60):lidar.ingest(*decoder.feed(scan_frame(65535,i))[0],now=1.4)
        self.assertFalse(lidar.snapshot(1.6)['online'])
        for i in range(60):lidar.ingest(*decoder.feed(scan_frame(0,i,100,300))[0],now=2)
        self.assertEqual(lidar.sequence,0)

    def test_long_running_clock_drift_does_not_mask_queue_delay(self):
        udp=TelemetryUDP();frame=scan_frame(1,0)
        # 100ppm source/host clock mismatch over 100 minutes. Sampling every
        # 0.5s keeps this fixture short while exercising the accumulated offset.
        for i in range(12000):
            self.assertIsNotNone(udp.feed(datagram(frame,i,round(i*499.95)),PEER,now=10+i*.5))
        self.assertIsNone(udp.feed(datagram(frame,12000,round(12000*499.95)),PEER,now=10+6000+.5))
        self.assertEqual(udp.stale,1)

    def test_missing_chunk_new_scan_and_mixed_capture_ids(self):
        lidar=FpgaLidar();decoder=MixedStream()
        def ingest(seq,index,capture=100):
            lidar.ingest(*decoder.feed(scan_frame(seq,index,capture=capture))[0],now=1)
        for i in range(59):ingest(1,i)
        ingest(1,59,999)
        self.assertIsNone(lidar.scan)
        for i in range(60):ingest(2,i)
        self.assertEqual(lidar.sequence,2)
        self.assertEqual(lidar.incomplete,1)
        invalid=bytearray(scan_frame(3,0));struct.pack_into('<H',invalid,16,700)
        invalid[46:]=struct.pack('<H',crc16(invalid[:46]))
        self.assertEqual(decoder.feed(invalid),[])

    def test_gateway_receives_udp_without_opening_or_writing_serial(self):
        gateway=Gateway(dict(telemetry_transport='udp',lidar_source='fpga'))
        for i in range(60):
            self.assertTrue(gateway.ingest_udp(datagram(scan_frame(1,i),i,100),PEER))
        state=gateway.state()
        self.assertEqual(len(state['lidar']['scan']['points']),360)
        self.assertEqual(state['telemetry']['accepted'],60)
        self.assertFalse(state['control']['motor_fresh'])
        self.assertFalse(gateway.threads)
        with self.assertRaises(ValueError):Gateway(dict(telemetry_transport='guess'))


if __name__ == '__main__':unittest.main()
