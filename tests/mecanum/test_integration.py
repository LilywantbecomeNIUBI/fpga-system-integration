import struct, unittest
from unittest.mock import Mock, patch
from host.mecanum.console import crc16, packet
from host.mecanum.transport import SplitLink
from shared.streams import MixedStream
from shared.telemetry_udp import TelemetryUDP, HEADER
from nano.gateway import Gateway


def status(kind=b'M4'):
    data=bytearray(46)
    data[:3]=kind+b'\x03'
    data[6]=253
    if kind==b'M4':
        struct.pack_into('<4i4h4h',data,8,123,-456,789,-1011,12,-34,56,-78,90,-120,150,-180)
        data[44]=4
    else:
        data[3]=3|16
        struct.pack_into('<4i',data,8,65536,-131072,12345,-23456)
        data[32]=4
        data[41]=8
        data[42]=0x70
    return bytes(data)+struct.pack('<H',crc16(data))


def udp(frame,seq=0,tick=1):
    return HEADER.pack(b'TLM1',1,b'\0'*3,seq,tick)+frame


class Integration(unittest.TestCase):
    def test_signed_four_wheels_and_pose_fragmentation(self):
        raw=status()+status(b'P4')
        stream=MixedStream();rows=[]
        for i in range(0,len(raw),7):
            rows+=stream.feed(raw[i:i+7])
        self.assertEqual([r[0] for r in rows],['mecanum_motor','mecanum_pose'])
        motor=rows[0][1];pose=rows[1][1]
        self.assertEqual(motor['position'],(123,-456,789,-1011))
        self.assertEqual(motor['velocity'],(12,-34,56,-78))
        self.assertEqual(motor['pwm'],(90,-120,150,-180))
        self.assertEqual((pose['x_counts'],pose['y_counts'],pose['who']),(1,-2,0x70))
        self.assertTrue(pose['imu_stale'])

    def test_udp_identity_crc_peer_replay_and_legacy(self):
        decoder=TelemetryUDP();peer=('192.168.50.30',6205)
        self.assertIsNone(decoder.feed(udp(status()),('192.168.50.99',6205),now=1))
        bad=bytearray(status());bad[8]^=1
        self.assertIsNone(decoder.feed(udp(bad),peer,now=1))
        self.assertEqual(decoder.feed(udp(status()),peer,now=1),status())
        self.assertIsNone(decoder.feed(udp(status()),peer,now=1))
        self.assertEqual(decoder.feed(udp(status(b'P4'),1,2),peer,now=1.001),status(b'P4'))
        legacy=bytearray(46);legacy[:3]=b'\xc3\x3c\x01'
        legacy+=struct.pack('<H',crc16(legacy))
        self.assertEqual(decoder.feed(udp(legacy,2,3),peer,now=1.002),legacy)

    def test_gateway_does_not_feed_four_wheels_to_two_wheel_control(self):
        gateway=Gateway(dict(telemetry_transport='udp',lidar_source='fpga'))
        gateway.ingest_uart(status()+status(b'P4'),now=10)
        state=gateway.state()
        self.assertEqual(state['mecanum']['mecanum_motor']['position'],(123,-456,789,-1011))
        self.assertIsNone(gateway.control.motor)
        self.assertEqual(gateway.threads,[])

    def test_split_link_read_never_uses_uart_status(self):
        link=SplitLink('FAKE');link.device=Mock();link.udp=Mock()
        link.udp.recvfrom.return_value=(udp(status()),('192.168.50.30',6205))
        self.assertEqual(link.read(256),status())
        link.device.read.assert_not_called()
        command=packet(0,1)
        link.write(command)
        link.device.write.assert_called_once_with(command)
        link.udp.sendto.assert_not_called()

    def test_removed_modes_are_still_rejected(self):
        for op in (2,4,5):
            with self.assertRaises(ValueError):packet(op,1)

    def test_mecanum_gateway_never_starts_legacy_command_worker(self):
        import nano.gateway as module
        gateway=Gateway(dict(chassis_type='mecanum',telemetry_transport='udp',lidar_source='fpga'),live=True)
        with patch.object(module,'Recorder'),patch.object(module.threading,'Thread') as thread:
            gateway.start()
        targets=[call.kwargs['target'] for call in thread.call_args_list]
        self.assertNotIn(gateway.serial_loop,targets)
        self.assertIn(gateway.telemetry_loop,targets)
        with self.assertRaises(ValueError):Gateway(dict(chassis_type='mecanum'))


if __name__=='__main__':
    unittest.main()
