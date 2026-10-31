import ast,json,math,struct,sys,threading,time,types,unittest,urllib.request,urllib.error
from unittest.mock import patch
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'host'),str(ROOT/'nano')]
from shared.streams import MixedStream,LidarStream,imu_protocol,crc8
from shared.calibration import Calibration,Odometry
from motor_protocol import crc16
from control import Control
from gateway import Gateway,make_server

def motor_frame(count=1):
    frame=b'\xc3\x3c\x01\x00\x00\x00'+struct.pack('<H',1)+struct.pack('<iiiihhIIIIH',1,2,3,4,0,0,0,0,0,0,count)
    return frame+struct.pack('<H',crc16(frame))
def imu_frame(seq=1):
    body=imu_protocol.BODY.pack(b'\xa5\x5a',2,1,seq*10,seq*10000,0,0,8192,0,0,0,0,0x39,0x47,0,2,0,0,0,seq,145,1000,10,150)
    return body+struct.pack('>H',crc16(body))
class StreamTests(unittest.TestCase):
    def test_mixed_fragment_crc_recovery(self):
        parser=MixedStream();bad=bytearray(imu_frame());bad[10]^=1
        stream=b'noise'+motor_frame()+bad+imu_frame(2)+motor_frame(2);rows=[]
        for i in range(0,len(stream),7):rows+=parser.feed(stream[i:i+7])
        self.assertEqual([x[0] for x in rows],['motor','imu','motor']);self.assertEqual(parser.bad_crc,1)
        self.assertEqual(rows[1][1]['raw_seq'],20)
    def test_lidar_captured_golden_and_scan(self):
        parser=LidarStream();golden=bytes.fromhex('542cf40d05162415443d0a30320a50300a6b320a822e0a912d0a972e0a982e0a9b2e0a982f0a97300a96bb1639652b')
        self.assertEqual(crc8(golden[:-1]),golden[-1]);parser.feed(golden)
        self.assertEqual(parser.frames,1);self.assertAlmostEqual(parser.last_packet['speed_hz'],3572/360)
        broken=bytearray(golden);broken[9]^=1;parser.feed(broken);self.assertEqual(parser.bad_crc,1)
        def packet(start):
            p=b'\x54\x2c'+struct.pack('<HH',3600,start)+b''.join(struct.pack('<HB',1000,100) for _ in range(12))+struct.pack('<HH',(start+2999)%36000,100)
            return p+bytes([crc8(p)])
        scans=[]
        for n in range(26):scans+=parser.feed(packet((n%12)*3000))
        self.assertTrue(scans);self.assertGreater(len(scans[-1]['points']),100)
    def test_calibration_required_and_rollover(self):
        self.assertFalse(Calibration().valid)
        with self.assertRaises(ValueError):Calibration().targets(.1,0)
        c=Calibration(dict(confirmed=True,left_cpr=1000,right_cpr=1000,left_diameter_m=.1,right_diameter_m=.1,track_m=.2,motor_reverse_mask=0,encoder_reverse_mask=0))
        self.assertTrue(c.valid);self.assertEqual(c.targets(.1,0),[3,3])
        odom=Odometry(c);odom.update(dict(left_position=2147483647,right_position=2147483647));odom.update(dict(left_position=-2147483648,right_position=-2147483648))
        self.assertAlmostEqual(odom.x,math.pi*.1/1000)

class ControlTests(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.control=Control(clock=lambda:self.now);self.control.connect();self.pcseq=0;self.packet_count=0
        self.status=dict(armed=False,faults=1,last_sequence=0,packet_count=0)
        self.control.ingest(self.status);self.lease=self.control.request(dict(action='claim'))['lease']
    def request(self,action,**kw):
        self.pcseq+=1;return self.control.request(dict(action=action,lease=self.lease,sequence=self.pcseq,**kw))
    def step(self,keep=True):
        self.now+=.01
        if keep and self.control.owner:self.request('heartbeat',held=False)
        for frame in self.control.poll():
            op=frame[3]
            if op==0:self.status['armed']=False
            elif op==3:self.status['faults']=0
            elif op==1:self.status.update(armed=True,last_sequence=frame[4])
            elif op==2:self.status['last_sequence']=frame[4]
        self.packet_count+=1;self.status['packet_count']=self.packet_count;self.control.ingest(self.status)
    def arm(self,**kw):
        self.request('arm',**kw)
        for _ in range(20):
            self.step()
            if self.control.phase=='active':return
        self.fail('ARM did not acknowledge')
    def test_handshake_latest_targets_timeout_and_no_rearm(self):
        self.arm();self.request('heartbeat',held=True,left=30,right=-20);self.request('heartbeat',held=True,left=40,right=-10)
        self.assertEqual(self.control.targets,[40,-10]);old=self.lease
        self.now+=.201;packets=self.control.poll();self.assertEqual(packets[0][3],0);self.assertIsNone(self.control.owner)
        with self.assertRaises(ValueError):self.request('heartbeat',held=True,left=50,right=50)
        self.assertEqual(self.control.targets,[0,0]);self.assertEqual(self.control.phase,'stopped')
    def test_held_false_releases_targets_and_replay_rejected(self):
        self.arm();self.request('heartbeat',held=True,left=10,right=20);self.request('heartbeat',held=False)
        self.assertEqual(self.control.targets,[0,0])
        with self.assertRaises(ValueError):self.control.request(dict(action='heartbeat',lease=self.lease,sequence=self.pcseq,held=True,left=99,right=99))
        self.request('release');self.assertIsNone(self.control.owner)
    def test_bad_ranges_dont_renew_and_physical_disabled(self):
        self.arm();deadline=self.control.deadline
        for kw in (dict(left=1025,right=0),dict(left=True,right=0),dict(units='physical',v=.1,w=0)):
            with self.assertRaises(ValueError):self.request('heartbeat',held=True,**kw)
        self.assertEqual(deadline,self.control.deadline)
    def test_stale_motor_and_disconnect_revokes(self):
        self.arm();self.now+=.19;self.request('heartbeat',held=False);self.now+=.02
        self.control.poll();self.assertIsNone(self.control.owner)
        self.control.disconnect();self.control.connect();self.assertIsNone(self.control.owner);self.assertEqual(self.control.phase,'stopped')
    def test_duplicate_telemetry_does_not_refresh(self):
        stamp=self.control.motor_time;self.now+=.15;self.control.ingest(self.status)
        self.assertEqual(stamp,self.control.motor_time)
    def test_counter_jump_revokes_and_requests_new_odometry_reference(self):
        self.arm();row=dict(self.status);row['packet_count']=(row['packet_count']+100)&65535
        self.assertEqual(self.control.ingest(row),'restart');self.assertIsNone(self.control.owner)
        self.assertEqual(self.control.phase,'stopped')
    def test_closed_only_and_timed_test_cannot_extend_or_rearm(self):
        with self.assertRaises(ValueError):self.request('arm',flags=0)
        self.arm()
        self.request('test',duration_ms=100,left=20,right=0)
        deadline=self.control.test_until
        for _ in range(8):self.step()
        self.assertEqual(self.control.targets,[20,0]);self.assertEqual(self.control.test_until,deadline)
        with self.assertRaises(ValueError):self.request('test',duration_ms=5000,left=30,right=0)
        for _ in range(4):self.step(keep=False)
        self.assertIsNone(self.control.owner);self.assertEqual(self.control.targets,[0,0])
        self.assertEqual(self.control.reason,'timed_test_complete')
    def test_permille_api_removed(self):
        self.arm()
        with self.assertRaises(ValueError):self.request('heartbeat',held=True,units='permille',left=20,right=0)
    def test_continuous_counts_heartbeat_runs_past_one_second_and_times_out(self):
        self.arm(flags=3,kp=4096,ki=128)
        for _ in range(150):
            self.request('heartbeat',held=True,left=-4,right=4)
            self.step(keep=False)
        self.assertEqual(self.control.phase,'active')
        self.assertEqual(self.control.targets,[-4,4])
        self.assertIsNone(self.control.test_until)
        self.now+=.201
        packets=self.control.poll()
        self.assertEqual(packets[0][3],0)
        self.assertIsNone(self.control.owner)
        self.assertEqual(self.control.targets,[0,0])
    def test_ground_mapping_limits_deadline_and_release(self):
        self.arm(flags=3,kp=4096,ki=128)
        for kw in (dict(speed=0),dict(speed=9),dict(speed=True),dict(duration_ms=1001),dict(direction='turn')):
            data=dict(speed=4,direction='forward',duration_ms=1000);data.update(kw)
            deadline=self.control.deadline
            with self.assertRaises(ValueError):self.request('ground_test',**data)
            self.assertEqual(self.control.deadline,deadline)
        self.request('ground_test',speed=4,direction='forward',duration_ms=100)
        self.assertEqual(self.control.targets,[-4,4]);deadline=self.control.test_until
        for _ in range(8):self.step()
        self.assertEqual(self.control.test_until,deadline)
        for _ in range(4):self.step(keep=False)
        self.assertIsNone(self.control.owner);self.assertEqual(self.control.targets,[0,0])
        self.lease=self.control.request(dict(action='claim'))['lease'];self.pcseq=0
        self.arm(flags=3)
        self.request('ground_test',speed=8,direction='backward',duration_ms=1000)
        self.assertEqual(self.control.targets,[8,-8])
        self.request('release');self.assertEqual(self.control.targets,[0,0]);self.assertIsNone(self.control.test_until)
    def test_ground_requires_correct_polarity_and_explicit_arm(self):
        with self.assertRaises(ValueError):self.request('ground_test',speed=4,direction='forward',duration_ms=1000)
        self.arm()
        with self.assertRaises(ValueError):self.request('ground_test',speed=4,direction='forward',duration_ms=1000)
    def test_gateway_restart_old_lease_rejected(self):
        other=Control(clock=lambda:self.now);other.connect();other.ingest(self.status)
        with self.assertRaises(ValueError):other.request(dict(action='arm',lease=self.lease,sequence=1))

class RecordingTests(unittest.TestCase):
    def test_raw_indices_use_capture_time_and_exact_offsets(self):
        from nano.gateway import Recorder
        import tempfile
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'record';recorder=Recorder(path)
            before=time.monotonic()
            recorder.put('uart',b'abc');recorder.put('lidar',b'xyz');recorder.put('uart',b'de')
            recorder.put('control_event',dict(kind='stop'));recorder.close()
            rows=[json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
            self.assertEqual((path/'fpga_uart.bin').read_bytes(),b'abcde')
            self.assertEqual((path/'lidar_uart.bin').read_bytes(),b'xyz')
            self.assertEqual([(r['kind'],r['data'].get('offset'),r['data'].get('length')) for r in rows[:3]],[('uart',0,3),('lidar',0,3),('uart',3,2)])
            self.assertTrue(all(before<=r['monotonic']<=time.monotonic() and r['wall_time']>0 for r in rows))
            self.assertEqual(rows[3]['data']['kind'],'stop')

class HttpTests(unittest.TestCase):
    def test_actual_serial_worker_with_fake_device(self):
        class Device:
            def __init__(self):self.buffer=bytearray();self.last=0;self.armed=False;self.faults=1;self.sequence=0;self.count=0;self.silent=False;self.writes=[]
            def __enter__(self):return self
            def __exit__(self,*args):pass
            @property
            def in_waiting(self):return len(self.buffer)
            def write(self,frame):
                self.assert_crc(frame);self.writes.append(bytes(frame));op=frame[3]
                if op==0:self.armed=False
                elif op==3:self.faults=0
                elif op==1:self.armed=True;self.sequence=frame[4]
                elif op==2:self.sequence=frame[4]
                return len(frame)
            def assert_crc(self,frame):
                if crc16(frame[:14])!=int.from_bytes(frame[14:],'little'):raise AssertionError('Outgoing CRC')
            def read(self,size):
                if not self.buffer and not self.silent and time.monotonic()-self.last>=.03:
                    self.count+=1;self.last=time.monotonic();body=bytearray(motor_frame(self.count)[:46]);body[3]=self.sequence;body[4]=int(self.armed);struct.pack_into('<H',body,6,self.faults)
                    self.buffer.extend(body+struct.pack('<H',crc16(body)))
                if not self.buffer:time.sleep(.002)
                result=bytes(self.buffer[:size]);del self.buffer[:size];return result
        device=Device();gateway=Gateway(dict(fpga_serial='FAKE_ONLY'),live=True)
        with patch.dict(sys.modules,{'serial':types.SimpleNamespace(Serial=lambda *a,**k:device)}):
            thread=threading.Thread(target=gateway.serial_loop,daemon=True);thread.start()
            try:
                end=time.monotonic()+.5
                while not gateway.control.fresh(time.monotonic()) and time.monotonic()<end:time.sleep(.005)
                lease=gateway.control.request(dict(action='claim'))['lease'];seq=1
                gateway.control.request(dict(action='arm',lease=lease,sequence=seq))
                end=time.monotonic()+.7
                while gateway.control.phase!='active' and time.monotonic()<end:
                    seq+=1;gateway.control.request(dict(action='heartbeat',lease=lease,sequence=seq,held=False));time.sleep(.02)
                self.assertEqual(gateway.control.phase,'active')
                seq+=1;gateway.control.request(dict(action='heartbeat',lease=lease,sequence=seq,held=True,left=42,right=-10));time.sleep(.08)
                self.assertTrue(any(p[3]==2 and struct.unpack_from('<hh',p,6)==(42,-10) for p in device.writes))
                device.silent=True;end=time.monotonic()+.35
                while gateway.control.owner and time.monotonic()<end:
                    try:
                        seq+=1;gateway.control.request(dict(action='heartbeat',lease=lease,sequence=seq,held=False))
                    except ValueError:break
                    time.sleep(.02)
                time.sleep(.03);self.assertIsNone(gateway.control.owner);self.assertFalse(device.armed)
            finally:gateway.stop.set();thread.join(1)
    def test_pc_offline_and_stale_gateway_never_enable_motion(self):
        from dashboard import Session
        session=Session()
        try:
            self.assertFalse(session.state()['motor_outputs_enabled'])
            with self.assertRaises(ValueError):session.gateway.control(dict(action='claim'))
            session.mode='live';session.gateway.data=dict(control=dict(motor_fresh=True,motor=dict(armed=True)),imu={},lidar=dict(online=True,scan=dict(points=[])))
            session.gateway.received=time.monotonic()-1
            state=session.state();self.assertFalse(state['motor_outputs_enabled']);self.assertIsNone(state['lidar']['scan'])
        finally:session.close()
    def test_authentication_and_read_only_offline(self):
        gateway=Gateway();server=make_server(gateway,'127.0.0.1',0,'a'*24)
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        url='http://127.0.0.1:'+str(server.server_port)
        try:
            with self.assertRaises(urllib.error.HTTPError):urllib.request.urlopen(url+'/api/state')
            req=urllib.request.Request(url+'/api/state',headers={'X-Session-Token':'a'*24})
            with urllib.request.urlopen(req) as r:state=json.load(r)
            self.assertEqual(state['mode'],'offline');self.assertFalse(state['control']['connected'])
            req=urllib.request.Request(url+'/api/control',data=b'{"action":"claim"}',headers={'X-Session-Token':'a'*24})
            with self.assertRaises(urllib.error.HTTPError):urllib.request.urlopen(req)
        finally:server.shutdown();server.server_close();gateway.close();worker.join()
    def test_lidar_can_be_disabled_without_opening_its_serial(self):
        import tempfile
        with tempfile.TemporaryDirectory() as temp,patch.dict(sys.modules,{'serial':types.SimpleNamespace()}):
            gateway=Gateway(dict(fpga_serial='TEST_ONLY',lidar_enabled=False,record_dir=temp),live=True)
            with patch.object(gateway,'serial_loop') as fpga,patch.object(gateway,'lidar_loop') as lidar,patch.object(gateway,'record_loop'):
                gateway.start()
                for thread in gateway.threads:thread.join(1)
                self.assertTrue(fpga.called);self.assertFalse(lidar.called)
                self.assertFalse(gateway.state()['lidar']['enabled'])
                gateway.close()

    def test_nano_python36_syntax(self):
        for folder in ('nano','shared'):
            for path in (ROOT/folder).rglob('*.py'):ast.parse(path.read_text(encoding='utf-8'),feature_version=(3,6))

    def test_dynamic_acceptance_rejects_missing_nan_and_short_run(self):
        sys.path.insert(0,str(ROOT/'tools'));from evaluate_dynamic import evaluate
        self.assertFalse(evaluate({},{} )['passed'])
        self.assertFalse(evaluate(dict(video_fps=float('nan')),dict(duration_s=float('nan')))['passed'])

if __name__=='__main__':unittest.main()
