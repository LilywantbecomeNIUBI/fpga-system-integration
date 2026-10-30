import sys
from pathlib import Path
import struct
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from motor_protocol import crc16,command,decode_status,StatusStream

class ProtocolTests(unittest.TestCase):
    def test_crc_and_command_layout(self):
        self.assertEqual(crc16(b'123456789'),0x29b1)
        frame=command(2,255,1,-80,60,256,0)
        self.assertEqual(len(frame),16)
        self.assertEqual(struct.unpack_from('<hhHH',frame,6),(-80,60,256,0))
        self.assertEqual(crc16(frame[:14]),struct.unpack_from('<H',frame,14)[0])
        self.assertEqual(command(0,256)[4],0)

    def test_validation(self):
        for args in [(1,0,0,1,0),(2,0,0,1001,0),(2,0,1,1025,0),(2,0,32),(2,0,0,0,0,4097,0)]:
            with self.assertRaises(ValueError):command(*args)

    def test_stream_recovers_after_noise_corruption_and_partial_frame(self):
        frame=b'\xc3\x3c\x01\x07\x01\x00'+struct.pack('<H',2)
        frame+=struct.pack('<iiiihhIIIIH',-1,2,-3,4,-100,200,5,6,7,8,9)
        frame+=struct.pack('<H',crc16(frame))
        decoded=decode_status(frame)
        self.assertEqual(decoded['left_velocity'],-3)
        self.assertEqual(decoded['right_applied'],200)
        self.assertEqual(decoded['fault_names'],['command_timeout'])
        corrupt=frame[:-1]+bytes([frame[-1]^1])
        stream=StatusStream()
        self.assertEqual(stream.feed(b'noise'+corrupt+frame[:17]),[])
        self.assertEqual(stream.feed(frame[17:]),[decoded])

if __name__=='__main__':unittest.main()
