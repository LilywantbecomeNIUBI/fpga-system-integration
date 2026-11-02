"""Exercise the CLI against a fake UART; no OS serial device is opened."""
from pathlib import Path
import contextlib
import io
import struct
import sys
import types
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
import console


class FakeUART:
    def __init__(self,reach=True):
        self.clock=0.;self.armed=False;self.fault=1;self.seq=0;self.mode=0;self.beats=0;self.writes=[];self.reach=reach;self.shift=4
    def __enter__(self):return self
    def __exit__(self,*args):pass
    def write(self,data):
        assert console.crc16(data[:22])==int.from_bytes(data[22:],'little')
        self.writes.append(data);op=data[3]
        if op==0:self.armed=False;self.mode=0
        elif op==3:self.fault=0
        elif op==1:self.armed=True;self.seq=data[4]
        elif op in (5,6):self.mode=op-4;self.seq=data[4];self.beats=0
        elif op==7:self.seq=data[4];self.beats+=1
    def read(self,size):
        self.clock+=0.02
        motor=bytearray(46);motor[:4]=b'M4\x03'+bytes([self.armed]);motor[6]=self.seq;motor[44]=self.shift
        struct.pack_into('<H',motor,4,self.fault)
        pose=bytearray(46);pose[:3]=b'P4\x03';pose[3]=3|(8 if self.mode==1 else 16 if self.mode==2 else 0);pose[32]=4
        if self.reach and self.beats>=4 and self.mode==2:pose[3]|=4
        pose[6]=self.seq;struct.pack_into('<H',pose,4,self.fault)
        return bytes(motor)+struct.pack('<H',console.crc16(motor))+bytes(pose)+struct.pack('<H',console.crc16(pose))


class ConsoleFlow(unittest.TestCase):
    def run_cli(self,uart,args):
        fake=types.SimpleNamespace(Serial=lambda *a,**kw:uart,SerialException=OSError)
        output=io.StringIO()
        with patch.dict(sys.modules,{'serial':fake}),patch.object(sys,'argv',['console','--port','FAKE',*args]),\
             patch.object(console.time,'monotonic',lambda:uart.clock),patch.object(console.time,'sleep',lambda n:None),contextlib.redirect_stdout(output):
            console.main()
        return output.getvalue()

    def test_goal_once_and_heartbeat(self):
        uart=FakeUART()
        output=self.run_cli(uart,['--action','turn','--angle','90','--seconds','5'])
        ops=[p[3] for p in uart.writes]
        self.assertEqual(ops.count(6),1);self.assertGreaterEqual(ops.count(7),4)
        self.assertEqual(ops[-3:],[0,0,0]);self.assertIn('Target reached',output)
        goal=next(p for p in uart.writes if p[3]==6)
        self.assertEqual(struct.unpack_from('<3h',goal,8),(0,0,9000))

    def test_vector_direction_and_heading_hold(self):
        uart=FakeUART()
        self.run_cli(uart,['--action','vector','--distance','200','--bearing','45','--seconds','5'])
        goal=next(p for p in uart.writes if p[3]==6)
        self.assertEqual(struct.unpack_from('<3h',goal,8),(2251,2251,0))
        self.assertEqual(sum(p[3]==6 for p in uart.writes),1)
        self.assertNotIn(2,[p[3] for p in uart.writes])

    def test_position_timeout_stops(self):
        uart=FakeUART(reach=False)
        with self.assertRaisesRegex(RuntimeError,'not reached'):
            self.run_cli(uart,['--action','position','--dx-counts','100','--seconds','0.3'])
        self.assertEqual([p[3] for p in uart.writes][-3:],[0,0,0])
        self.assertEqual(sum(p[3]==6 for p in uart.writes),1)

    def test_invalid_calibration_rejected_before_port_open(self):
        with patch.object(sys,'argv',['console','--port','FAKE','--action','position','--dx','100','--counts-per-rev','0']),\
             patch.dict(sys.modules,{'serial':types.SimpleNamespace(Serial=lambda *a,**k:self.fail('opened serial'))}),\
             contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            console.main()

    def test_mm_uses_confirmed_gmr_profile(self):
        uart=FakeUART()
        self.run_cli(uart,['--action','position','--dx','200','--dy','-100','--seconds','5'])
        goal=next(p for p in uart.writes if p[3]==6)
        self.assertEqual(struct.unpack_from('<3h',goal,8),(3183,-1592,0))

    def test_removed_cli_actions_rejected_before_serial(self):
        for args in [['--action','drive'],['--action','heading'],['--wheel','0'],['--speed','20']]:
            with patch.object(sys,'argv',['console','--port','FAKE',*args]), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                console.main()

    def test_unit_mismatch_prevents_arming(self):
        uart=FakeUART();uart.shift=0
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            self.run_cli(uart,['--action','position','--dx','100'])
        self.assertNotIn(1,[p[3] for p in uart.writes])
        self.assertEqual([p[3] for p in uart.writes][-3:],[0,0,0])


if __name__=='__main__':unittest.main()
