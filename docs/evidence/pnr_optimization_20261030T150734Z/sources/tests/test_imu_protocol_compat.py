"""Headless parser must retain the frozen viewer's wire behavior."""
import importlib.util
from pathlib import Path
import struct
import unittest
from shared import imu_protocol as compatible


class CompatibilityTests(unittest.TestCase):
    def test_headless_parser_matches_frozen_source(self):
        path=Path(__file__).resolve().parents[1]/'vendor/imu/pc/imu_viewer.py'
        spec=importlib.util.spec_from_file_location('frozen_imu',str(path))
        original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
        frames=[]
        for seq,kind,config,decimation in [(1,1,2,10),(2,2,3,5),(4,1,2,10),(5,3,2,10),(6,1,2,0)]:
            body=original.BODY.pack(b'\xa5\x5a',2,kind,seq*10,seq*10000,-8192,4096,7000,-100,200,300,100,0x39,0x47,0,config,0,0,0,seq,145,1000,decimation,150)
            self.assertEqual(original.crc16(body),compatible.crc16(body))
            frames.append(body+struct.pack('>H',original.crc16(body)))
        bad=bytearray(frames[0]);bad[10]^=1
        stream=b'noise'+bytes(bad)+b''.join(frames)
        a,b=original.Parser(),compatible.Parser();sa,sb=original.Statistics(),compatible.Statistics()
        for i in range(0,len(stream),7):
            left,right=a.feed(stream[i:i+7]),b.feed(stream[i:i+7]);self.assertEqual(left,right)
            for row in left:sa.add(row)
            for row in right:sb.add(row)
        self.assertEqual(sa.report(a),sb.report(b))
        self.assertEqual(a.bad_crc,b.bad_crc);self.assertEqual(a.unsupported,b.unsupported)
