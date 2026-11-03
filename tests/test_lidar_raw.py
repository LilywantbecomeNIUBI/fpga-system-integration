import struct
import unittest
from unittest.mock import patch
from host.motor_protocol import crc16
from shared.streams import MixedStream
from shared.lidar_display import FpgaLidar
from nano.gateway import Gateway
from nano.web_console import ViewerGateway
from shared.telemetry_udp import HEADER


def native_points(seq):
    points=[]
    for i in range(12):
        index=(seq-1)*12+i
        angle=(index*100+37)%36000
        if seq%30==5 and i==2:angle=((index-1)*100+37)%36000
        points.append([angle,0 if seq%30==6 and i==2 else 1000+index,0 if seq%30==7 and i==3 else 50+i])
    return points


def point_frame(seq,half,capture=None,flags=0,age=0,points=None):
    capture=seq if capture is None else capture
    points=native_points(seq) if points is None else points
    body=b'\xc3\x3c\x07\x01'+struct.pack('<I H I BB',seq,age,capture,half,flags)
    body+=b''.join(struct.pack('<HHB',*p) for p in points[half*6:half*6+6])
    return body+struct.pack('<H',crc16(body))


class RawTests(unittest.TestCase):
    def test_observer_fast_path_matches_uart_decoding_and_rejects_bad_crc(self):
        observer = ViewerGateway(dict(receive_only=True, telemetry_transport='udp',
                                      lidar_source='fpga', record_telemetry=False))
        reference = Gateway(dict(telemetry_transport='udp', lidar_source='fpga'))
        for seq in range(1, 122):
            for half in (0, 1):
                if seq == 47 or (seq == 45 and half == 1):
                    continue
                packet = HEADER.pack(b'TLM1', 1, bytes(3), seq*2+half, seq) + point_frame(seq, half)
                with patch('time.monotonic', return_value=1+seq/1000.):
                    self.assertEqual(observer.ingest_udp(packet, ('192.168.50.30', 6205)),
                                     reference.ingest_udp(packet, ('192.168.50.30', 6205)))
        self.assertGreater(observer.fpga_lidar.scans, 0)
        self.assertEqual(observer.fpga_lidar.snapshot(1.122), reference.fpga_lidar.snapshot(1.122))
        before = observer.fpga_lidar.raw.packets
        corrupt = bytearray(packet); corrupt[-1] ^= 1
        self.assertFalse(observer.ingest_udp(bytes(corrupt), ('192.168.50.30', 6205)))
        self.assertFalse(observer.ingest_udp(packet, ('192.168.50.31', 6205)))
        self.assertEqual(observer.fpga_lidar.raw.packets, before)

    def ingest(self,state,frame,now):
        rows=MixedStream().feed(frame)
        self.assertEqual(len(rows),1)
        state.ingest(*rows[0],now=now)

    def test_all_points_duplicates_zero_returns_and_measured_angles_preserved(self):
        state=FpgaLidar()
        for seq in range(1,92):
            for half in (0,1):self.ingest(state,point_frame(seq,half),1+seq/1000.)
        self.assertEqual(state.scans,2)
        expected=[[a/100.,d,q] for seq in range(61,91) for a,d,q in native_points(seq)]
        self.assertEqual(state.scan['points'],expected)
        self.assertEqual(state.scan['representation'],'raw_points')
        self.assertIsNone(state.snapshot(1.1)['resolution_degrees'])
        self.assertTrue(any(p[1]==0 for p in expected))
        self.assertTrue(any(p[2]==0 for p in expected))
        self.assertEqual(expected[49][0],expected[50][0])
        for half in (0,1):self.ingest(state,point_frame(91,half),1.5)
        self.assertFalse(state.snapshot(1.6)['online'],'duplicate packets must not renew scan time')

    def test_lost_half_whole_packet_and_new_clean_revolution_recovers(self):
        state=FpgaLidar()
        for seq in range(1,92):
            for half in (0,1):
                if seq==47 or (seq==45 and half==1):continue
                self.ingest(state,point_frame(seq,half),1+seq/1000.)
        self.assertEqual(state.scans,1)
        self.assertEqual(state.raw.missing_packets,2)
        self.assertGreater(state.incomplete,0)
        self.assertEqual(state.scan['points'][0],[.37,1720,50])

    def test_parser_error_flag_rejects_affected_circle(self):
        state=FpgaLidar()
        for seq in range(1,92):
            for half in (0,1):self.ingest(state,point_frame(seq,half,flags=int(seq==40)),1+seq/1000.)
        self.assertEqual(state.scans,1)
        self.assertEqual(state.raw.missing_packets,0)

    def test_bad_layout_age_and_mixed_capture_are_rejected(self):
        decoder=MixedStream()
        wrong_half=bytearray(point_frame(1,0));wrong_half[14]=2
        wrong_half[46:]=struct.pack('<H',crc16(wrong_half[:46]))
        for frame in (point_frame(1,0,age=400),point_frame(1,0,flags=2),bytes(wrong_half)):
            self.assertEqual(decoder.feed(frame),[])
            decoder=MixedStream()
        state=FpgaLidar();self.ingest(state,point_frame(1,0,capture=1),1)
        self.ingest(state,point_frame(1,1,capture=2),1.001)
        self.assertEqual(state.raw.packets,0)

    def test_old_queued_capture_cannot_refresh_scan(self):
        state=FpgaLidar()
        for seq in range(1,62):
            for half in (0,1):self.ingest(state,point_frame(seq,half),1+seq/1000.)
        original_time=state.scan_time
        for seq,now in ((62,1.26),(63,1.46),(64,1.66)):
            for half in (0,1):self.ingest(state,point_frame(seq,half),now)
        self.assertGreaterEqual(state.raw.stale_packets,1)
        self.assertEqual(state.scan_time,original_time)
        self.assertFalse(state.snapshot(1.8)['online'])

    def test_sequence_wrap_timeout_and_fifo_error_diagnostic(self):
        state=FpgaLidar()
        for seq in (0xffffffff,0):
            for half in (0,1):
                self.ingest(state,point_frame(seq,half,capture=1,points=native_points(1)),1)
        self.assertEqual(state.raw.packets,2)
        self.assertEqual(state.raw.last_sequence,0)
        self.ingest(state,point_frame(1,0,capture=2),1)
        self.ingest(state,point_frame(1,1,capture=2),1.3)
        self.assertEqual(state.raw.packets,2)
        body=b'\xc3\x3c\x06\x03'+struct.pack('<HBB9IH',12,1,0,100,5000,0,100,0,0,0,90,10,8)
        self.ingest(state,body+struct.pack('<H',crc16(body)),1.4)
        self.assertEqual(state.diagnostic['dropped_packets'],10)
        self.assertEqual(state.diagnostic['queued_packets'],8)

    def test_gateway_uses_complete_raw_frames_without_serial_or_control_io(self):
        gateway=Gateway(dict(telemetry_transport='udp',lidar_source='fpga'))
        transport_seq=0
        for seq in range(1,62):
            for half in (0,1):
                data=HEADER.pack(b'TLM1',1,bytes(3),transport_seq,100)+point_frame(seq,half)
                self.assertTrue(gateway.ingest_udp(data,('192.168.50.30',6205)))
                transport_seq+=1
        snapshot=gateway.state()
        self.assertEqual(snapshot['lidar']['scan']['representation'],'raw_points')
        self.assertEqual(len(snapshot['lidar']['scan']['points']),360)
        self.assertFalse(snapshot['control']['motor_fresh'])
        self.assertFalse(gateway.threads)


if __name__=='__main__':unittest.main()
