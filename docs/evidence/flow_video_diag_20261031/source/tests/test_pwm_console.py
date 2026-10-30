import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from pwm_console import Control,Session,encode,make_server,crc16


def status():
    return dict(flags=34,faults=1,armed=False,last_sequence=0,packet_count=1,
        left_applied=0,right_applied=0,left_position=0,right_position=0,
        left_velocity=0,right_velocity=0,left_invalid=0,right_invalid=0,bad_frames=0,rejected_commands=0)


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.c=Control(lambda:self.now);self.m=status();self.seq=0;self.client='test-client-abcdefghijkl';self.run=None;self.sent=[]
        self.c.connect();self.c.poll();self.now=.01;self.c.ingest(self.m);self.c.poll()
        self.assertEqual(self.c.phase,'stopped')
    def request(self,action,**kwargs):
        self.seq+=1
        return self.c.request(dict(action=action,client_id=self.client,sequence=self.seq,run_id=self.run,**kwargs))
    def step(self,heartbeat=True):
        self.now+=.01
        if heartbeat and self.c.owner:self.request('heartbeat')
        for frame in self.c.poll():
            self.sent.append(frame);op=frame[3]
            if op==0:self.m.update(armed=False,left_applied=0,right_applied=0)
            elif op==3:self.m['faults']=0
            elif op==1:self.m.update(armed=True,last_sequence=frame[4])
            elif op==2:
                left,right=struct.unpack_from('<hh',frame,6)
                self.m.update(left_applied=left,right_applied=right,last_sequence=frame[4])
        for side in ('left','right'):
            if self.m[side+'_applied']:
                self.m[side+'_position']+=100;self.m[side+'_invalid']+=100;self.m[side+'_velocity']+=self.m[side+'_applied']*500
        self.m['packet_count']+=1;self.c.ingest(self.m)
    def begin(self,channel='M1',duty=200,duration=300):
        self.run=self.request('test',channel=channel,duty_permille=duty,duration_ms=duration,setup_confirmed=True)['run_id']
        for _ in range(20):
            self.step()
            if self.c.phase=='running':return
        self.fail('No ARM/DRIVE')
    def test_m1_m2_serialization_and_no_feedback_control(self):
        for channel,expected in [('M1',(200,0)),('M2',(0,200))]:
            self.begin(channel)
            drive=[f for f in self.sent if f[3]==2][-1]
            self.assertEqual(struct.unpack_from('<hh',drive,6),expected)
            self.assertEqual(drive[5],34);self.assertEqual(struct.unpack_from('<HH',drive,10),(0,0))
            self.assertEqual(struct.unpack_from('<H',drive,14)[0],crc16(drive[:14]))
            self.c.stop();self.step(False);self.step(False);self.step(False)
            self.assertEqual(self.c.phase,'stopped')
    def test_timer_fixed_despite_heartbeats_and_metrics(self):
        self.begin();deadline=self.c.run_until
        for _ in range(40):self.step()
        self.assertEqual(self.c.phase,'stopped');self.assertIsNone(self.c.owner)
        self.assertGreater(self.c.last_result['left_pad_edges'],0);self.assertEqual(self.c.last_result['right_pad_edges'],0)
        self.assertEqual(self.c.last_result['run_id'],self.run)
        self.assertLessEqual(deadline,self.now)
        self.assertTrue(all(struct.unpack_from('<hh',f,6)==(200,0) for f in self.sent if f[3]==2))
    def test_client_timeout_stop_no_auto_restart(self):
        self.begin();self.now+=.201
        self.assertEqual(self.c.poll()[0][3],0);self.assertIsNone(self.c.owner)
        with self.assertRaises(ValueError):self.request('heartbeat')
        self.c.disconnect();self.c.connect();self.assertIsNone(self.c.owner)
        self.assertEqual(self.c.poll()[0][3],0)
    def test_five_second_duration_and_old_firmware_rejected(self):
        self.m['flags']=32;self.m['packet_count']+=1;self.c.ingest(self.m)
        with self.assertRaises(ValueError):self.begin(duration=5000)
        self.m['flags']=34;self.m['packet_count']+=1;self.c.ingest(self.m)
        self.begin(duration=5000)
        for _ in range(490):self.step()
        self.assertEqual(self.c.phase,'running')
        for _ in range(15):self.step()
        self.assertEqual(self.c.phase,'stopped')
    def test_invalid_and_overlapping_requests_do_not_renew(self):
        for kw in [dict(duty_permille=201),dict(duty_permille=True),dict(duration_ms=5001),dict(duration_ms=99),dict(channel='both'),dict(setup_confirmed=False)]:
            data=dict(channel='M1',duty_permille=100,duration_ms=500,setup_confirmed=True);data.update(kw)
            with self.assertRaises(ValueError):self.request('test',**data)
        self.begin();deadline=self.c.deadline
        with self.assertRaises(ValueError):self.request('test',channel='M2',duty_permille=200,duration_ms=500,setup_confirmed=True)
        self.assertEqual(deadline,self.c.deadline)
        with self.assertRaises(ValueError):self.c.request(dict(action='heartbeat',client_id=self.client,run_id=self.run,sequence=self.c.client_seq))
    def test_wrong_firmware_and_stale_status_block_output(self):
        self.m['flags']=1;self.m['packet_count']+=1;self.c.ingest(self.m)
        with self.assertRaises(ValueError):self.begin()
        self.m['flags']=34;self.m['packet_count']+=1;self.c.ingest(self.m);self.now+=.201
        with self.assertRaises(ValueError):self.begin()
    def test_encoder_named_fields_never_regulate_raw_duty(self):
        self.begin();self.m['left_velocity']=2000000;self.m['left_invalid']=1234
        for _ in range(8):self.step()
        self.assertEqual(self.c.phase,'running')
        self.assertEqual(struct.unpack_from('<hh',[f for f in self.sent if f[3]==2][-1],6),(200,0))
    def test_encode_rejects_dual_channel_and_out_of_range(self):
        for args in [(2,1,200,1),(2,1,-1,0),(2,1,0,201),(1,1,200,0)]:
            with self.assertRaises(ValueError):encode(*args)


class HttpWorkerTests(unittest.TestCase):
    def test_real_worker_http_fake_uart_and_startup_stop_only(self):
        class Device:
            def __init__(self):self.m=status();self.writes=[];self.last=0
            def __enter__(self):return self
            def __exit__(self,*args):pass
            @property
            def in_waiting(self):return 0
            def write(self,packet):
                self.writes.append(packet);op=packet[3]
                if op==0:self.m.update(armed=False,left_applied=0,right_applied=0)
                elif op==3:self.m['faults']=0
                elif op==1:self.m.update(armed=True,last_sequence=packet[4])
                elif op==2:
                    left,right=struct.unpack_from('<hh',packet,6);self.m.update(left_applied=left,right_applied=right,last_sequence=packet[4])
                return len(packet)
            def read(self,n):
                time.sleep(.003)
                if time.monotonic()-self.last<.02:return b''
                self.last=time.monotonic();m=self.m;m['packet_count']+=1
                for side in ('left','right'):
                    if m[side+'_applied']:m[side+'_position']+=200;m[side+'_invalid']+=200
                body=b'\xc3\x3c'+struct.pack('<BBBBH',1,m['last_sequence'],int(m['armed']),34,m['faults'])
                body+=struct.pack('<iiiihhIIIIH',m['left_position'],m['right_position'],0,0,m['left_applied'],m['right_applied'],m['left_invalid'],m['right_invalid'],0,0,m['packet_count'])
                return body+struct.pack('<H',crc16(body))
        device=Device()
        with tempfile.TemporaryDirectory() as folder:
            session=Session('FAKE_UART_ONLY',True,folder,lambda *args,**kwargs:device);server=make_server(session,0,'t'*24)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();url='http://127.0.0.1:'+str(server.server_port)
            def request(data):
                req=urllib.request.Request(url+'/api/control',data=json.dumps(data).encode(),headers={'X-Session-Token':'t'*24})
                return json.loads(urllib.request.urlopen(req,timeout=1).read())
            try:
                until=time.monotonic()+1
                while session.control.phase!='stopped' and time.monotonic()<until:time.sleep(.01)
                self.assertEqual(session.control.phase,'stopped');self.assertTrue(all(p[3]==0 for p in device.writes))
                with self.assertRaises(urllib.error.HTTPError):urllib.request.urlopen(urllib.request.Request(url+'/api/control',data=b'{"action":"stop"}'))
                self.assertIn('M1 / M2',urllib.request.urlopen(url).read().decode())
                run=request(dict(action='test',client_id='client-abcdefghijklmnop',sequence=1,channel='M2',duty_permille=150,duration_ms=100,setup_confirmed=True))['run_id']
                seq=1;until=time.monotonic()+2
                while not session.control.history and time.monotonic()<until:
                    seq+=1
                    try:request(dict(action='heartbeat',client_id='client-abcdefghijklmnop',run_id=run,sequence=seq))
                    except urllib.error.HTTPError:pass
                    time.sleep(.025)
                self.assertTrue(session.control.history)
                self.assertTrue(all(struct.unpack_from('<hh',p,6)==(0,150) for p in device.writes if p[3]==2))
                self.assertFalse(device.m['armed']);self.assertEqual(device.m['right_applied'],0)
                self.assertIn(b'M2',urllib.request.urlopen(url+'/api/log.csv').read())
                self.assertTrue((Path(folder)/'uart.bin').stat().st_size)
            finally:server.shutdown();server.server_close();session.close();thread.join(1)


if __name__=='__main__':unittest.main()
