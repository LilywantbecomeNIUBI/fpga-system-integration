"""Unified local sensing console. Default offline; live IO is explicitly selected."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import secrets
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'vendor/vision'))
sys.path.insert(0, str(ROOT/'.deps'))
import cv2
import numpy as np
from vision_dashboard_common import Capture, jpeg
from vision_dashboard import NanoResults
from shared.perception import PerceptionEvents
from shared.live_capture import Capture
spec = importlib.util.spec_from_file_location('imu_protocol', ROOT/'vendor/imu/pc/imu_viewer.py')
imu_protocol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(imu_protocol)
PROFILE = 'fpga_robot_sensing_v1'

class ImuState:
    def __init__(self):
        self.parser = imu_protocol.Parser()
        self.stats = imu_protocol.Statistics()
        self.lock = threading.Lock()
        self.last = None
        self.received = None
        self.error = None
        self.pitch = self.roll = None
        self.previous = None

    def feed(self, payload, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            for row in self.parser.feed(payload):
                self.stats.add(row)
                self.last, self.received = row, now
                good = row['type']=='sample' and row['ready'] and not row['fault'] and not row['stale'] and row['who_am_i']==0x47
                if not good:
                    self.previous = None
                    self.pitch = self.roll = None
                    continue
                ax,ay,az = (row[k] for k in ('ax_g','ay_g','az_g'))
                norm = math.sqrt(ax*ax+ay*ay+az*az)
                if norm < .2 or norm > 2:
                    self.previous = None
                    self.pitch = self.roll = None
                    continue
                pitch = math.degrees(math.atan2(-ax, math.hypot(ay,az)))
                roll = math.degrees(math.atan2(ay,az))
                dt = ((row['timestamp_us']-self.previous)&0xffffffff)/1e6 if self.previous is not None else 0
                if 0<dt<.05 and self.pitch is not None:
                    self.pitch = .98*(self.pitch+row['gy_dps']*dt)+.02*pitch
                    # Circular roll innovation avoids the +/-180 degree discontinuity.
                    prediction = self.roll+row['gx_dps']*dt
                    self.roll = (prediction+.02*((roll-prediction+180)%360-180)+180)%360-180
                else:
                    self.pitch,self.roll = pitch,roll
                self.previous = row['timestamp_us']
            self.error = None

    def snapshot(self, now=None):
        now=time.monotonic() if now is None else now
        with self.lock:
            age = now-self.received if self.received is not None else None
            row=dict(self.last) if self.last else None
            healthy = bool(row and row['type']=='sample' and row['ready'] and not row['fault'] and not row['stale'] and row['who_am_i']==0x47)
            return dict(online=age is not None and age<.5 and not self.error, healthy=healthy,
                age_ms=age*1000 if age is not None else None,error=self.error,
                frame=row,statistics=self.stats.report(self.parser),
                attitude=dict(valid=bool(healthy and age is not None and age<.5 and not self.error and self.pitch is not None),
                    pitch_deg=self.pitch,roll_deg=self.roll,yaw_deg=None,
                    convention='sensor axes; roll about +X, pitch about +Y, upright +Z; yaw unavailable',
                    method='host complementary filter of decimated IMU; visualization only, no motor control'))

class Session:
    def __init__(self, mode='offline', config=None, out=None, replay=None):
        self.mode,self.config,self.out = mode,config or {},out
        self.stop = threading.Event()
        self.imu = ImuState()
        self.capture=self.nano=None
        self.threads=[]
        self.event=None;self.event_stamp=None;self.event_error=None;self.perception=PerceptionEvents()
        self.video_error=None
        self.frame_lock=threading.Lock()
        self.demo_image=None
        self.record_lock=threading.Lock()
        self.recording=False;self.record_file=None;self.raw_file=None;self.record_error=None
        if mode=='live':
            try:
                self.capture=Capture(self.config['bind'],port=self.config.get('video_port',6102),source=self.config.get('fpga','192.168.50.30'))
            except Exception as exc:self.video_error=str(exc)
            if self.config.get('nano_url'):
                self.nano=NanoResults(self.config['nano_url'],self.stop)
            self.spawn(self.serial_loop)
            self.spawn(self.event_loop)
        elif mode=='demo':
            self.spawn(self.demo_loop)
        elif mode=='replay':
            if replay is None:raise ValueError('Replay path required')
            self.spawn(lambda:self.replay_loop(replay))
        self.spawn(self.record_loop)

    def spawn(self, target):
        thread=threading.Thread(target=target,daemon=True)
        self.threads.append(thread);thread.start()

    def serial_loop(self):
        port=self.config.get('serial_port')
        if not port:
            self.imu.error='No serial_port configured';return
        while not self.stop.is_set():
            try:
                import serial
                with serial.Serial(port,115200,timeout=.1,write_timeout=.1) as stream:
                    # There is no UART write path in this sensing console.
                    while not self.stop.is_set():
                        payload=stream.read(4096)
                        if payload:
                            self.imu.feed(payload)
                            with self.record_lock:
                                if self.raw_file:self.raw_file.write(payload)
            except Exception as exc:
                with self.imu.lock:self.imu.error=str(exc)
                self.stop.wait(1)

    def event_loop(self):
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
                sock.bind((self.config['bind'],self.config.get('event_port',6202)));sock.settimeout(.2)
                while not self.stop.is_set():
                    try:data,peer=sock.recvfrom(2048)
                    except socket.timeout:continue
                    if peer!=(self.config.get('event_source',self.config.get('fpga','192.168.50.30')),self.config.get('event_source_port',6202)):continue
                    try:
                        with self.frame_lock:
                            kind,event=self.perception.ingest(data)
                            if kind=='tracking':self.event,self.event_stamp=event,time.monotonic()
                            self.event_error=None
                    except ValueError as exc:
                        self.event_error=str(exc);continue
        except Exception as exc:self.event_error=str(exc)

    def replay_loop(self, path):
        # Playback preserves original CRC/seq/timestamps; pacing is debug-record nominal 200Hz.
        with Path(path).open('rb') as stream:
            while not self.stop.is_set():
                packet=stream.read(48)
                if not packet:break
                self.imu.feed(packet);self.stop.wait(.005)

    def demo_loop(self):
        import struct
        n=0
        while not self.stop.is_set():
            n+=1
            body=imu_protocol.BODY.pack(b'\xa5\x5a',2,1,n*5,n*5000,0,0,8192,0,0,0,0,0x39,0x47,0,2,0,0,0,n,145,1000,5,150)
            self.imu.feed(body+struct.pack('!H',imu_protocol.crc16(body)))
            if n%10==0:
                image=np.zeros((320,320,3),dtype=np.uint8)
                cv2.putText(image,'SIMULATION',(28,155),cv2.FONT_HERSHEY_SIMPLEX,.9,(0,200,255),2)
                cv2.putText(image,'NO HARDWARE',(32,190),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
                with self.frame_lock:self.demo_image=(jpeg(image),time.monotonic())
            self.stop.wait(.005)

    def state(self):
        now=time.monotonic()
        pc=self.capture.status() if self.capture else dict(frames=0,fps=0,age_ms=None,error=self.video_error)
        pc['online']=pc.get('age_ms') is not None and pc['age_ms']<500 and not pc.get('error')
        with self.frame_lock:
            event=dict(self.event) if self.event else None
            event_age=(now-self.event_stamp)*1000 if self.event_stamp is not None else None
            flow=self.perception.flow_snapshot(now, self.mode if self.mode=='live' else 'idle')
        nano=self.nano.state() if self.nano else dict(online=False,error='Nano not connected',data=None)
        data=nano.get('data') or {}
        result=data.get('result') or {}
        # A responding HTTP process with an old inference is not a healthy detection chain.
        nano['result_fresh']=bool(nano.get('online') and not data.get('error') and not data.get('offline')
            and result and result.get('age_ms',float('inf'))<500)
        if event is None and data.get('event'):
            event=data['event'];event_age=data.get('event_age_ms')
            if not nano.get('online'):event_age=None
        if event_age is not None and self.event_stamp is None:
            event_age+=(nano.get('poll_age_ms') or 0)
        # A cached Nano snapshot must age locally even while HTTP is offline.
        if not flow['packets'] and data.get('flow'):
            flow=json.loads(json.dumps(data['flow']))
            c=flow['control'];age=c.get('age_ms')
            age=age+(nano.get('poll_age_ms') or 0) if age is not None else None
            c['age_ms']=age
            fresh=bool(nano.get('online') and age is not None and 0<=age<200 and c.get('fresh') and self.mode=='live')
            c['fresh']=fresh
            if not fresh:c.update(confident=False,state='unknown',stop_request=True,steer=0,speed_limit=0)
            flow.update(observe_only=True,automatic_control_enabled=False)
        tracking=dict(event=event,age_ms=event_age,error=self.event_error,
            fresh=event_age is not None and event_age<500)
        with self.record_lock:
            recording=dict(active=self.recording,error=self.record_error,path=str(self.out) if self.out else None,
                contents='10Hz PC status JSONL; raw serial streams are recorded on Nano; frames only on explicit save')
        return dict(profile=PROFILE,mode=self.mode,simulation=self.mode=='demo',replay=self.mode=='replay',
            motor_outputs_enabled=False,pc=pc,nano=nano,imu=self.imu.snapshot(),tracking=tracking,flow=flow,recording=recording)

    def image(self, inference=False, key=None):
        if inference:
            if not self.nano or not self.state()['nano']['result_fresh']:return None
            cached=self.nano.exact(key)
            return cached[0] if cached else None
        if self.capture:
            item=self.capture.frame()
            return item[3] if item and time.monotonic()-item[2]<.5 else None
        with self.frame_lock:
            return self.demo_image[0] if self.demo_image and time.monotonic()-self.demo_image[1]<.5 else None

    def stream_image(self, inference=False):
        """Newest encoded frame only; a slow viewer never builds a frame queue."""
        if inference:
            if not self.nano:return None
            status=self.nano.state();result=(status.get('data') or {}).get('result') or {}
            if not status['online'] or result.get('age_ms',float('inf'))+(status.get('poll_age_ms') or 0)>=500:return None
            key=result.get('key');cached=self.nano.exact(key)
            return (key,cached[0]) if cached else None
        if self.capture:
            item=self.capture.frame()
            return (item[0],item[3]) if item and time.monotonic()-item[2]<.5 and item[3] else None
        return None

    def record(self, enabled):
        with self.record_lock:
            if enabled==self.recording:return
            if enabled:
                if self.out is None:raise ValueError('No output directory')
                self.out.mkdir(parents=True,exist_ok=True)
                stamp=time.strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(3)
                self.record_file=(self.out/(stamp+'_status.jsonl')).open('x',encoding='utf-8')
                # Raw serial streams are recorded exclusively by the Nano gateway.
            else:
                for f in (self.record_file,self.raw_file):
                    if f:f.close()
                self.record_file=self.raw_file=None
            self.recording=enabled

    def record_loop(self):
        while not self.stop.wait(.1):
            data=self.state()
            with self.record_lock:
                if self.record_file:
                    try:
                        self.record_file.write(json.dumps(dict(host_unix=time.time(),state=data),allow_nan=False)+'\n')
                        self.record_file.flush()
                    except OSError as exc:self.record_error=str(exc)

    def close(self):
        self.stop.set()
        if self.capture:self.capture.close()
        for thread in self.threads:thread.join(3)
        self.record(False)

def make_server(session, port, token):
    html=(ROOT/'host/index.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,body,kind='application/json',status=200):
            self.send_response(status)
            self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def value(self,value,status=200):
            self.send(json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8'),status=status)
        def do_GET(self):
            url=urlsplit(self.path)
            if url.path=='/':self.send(html,'text/html; charset=utf-8')
            elif url.path in ('/api/state','/health'):self.value(session.state())
            elif url.path in ('/camera.jpg','/infer.jpg'):
                image=session.image(url.path=='/infer.jpg',parse_qs(url.query).get('key',[''])[0])
                if image:self.send(image,'image/jpeg')
                else:self.value({'error':'No fresh frame'},503)
            else:self.value({'error':'Not found'},404)
        def do_POST(self):
            host=self.headers.get('Host')
            origin=self.headers.get('Origin')
            browser=(host in ('127.0.0.1:%d'%self.server.server_port,'localhost:%d'%self.server.server_port)
                and origin=='http://'+host)
            if not browser and self.headers.get('X-Session-Token')!=token:
                self.value({'error':'Forbidden'},403);return
            try:
                if self.path=='/api/record/start':session.record(True)
                elif self.path=='/api/record/stop':session.record(False)
                elif self.path=='/api/save':
                    image=session.image()
                    if not image or session.out is None:self.value({'error':'No fresh image/output'},409);return
                    session.out.mkdir(parents=True,exist_ok=True)
                    path=session.out/('frame_'+str(time.time_ns())+'.jpg');path.write_bytes(image)
                elif self.path=='/api/shutdown':
                    self.value({'stopping':True});threading.Thread(target=self.server.shutdown,daemon=True).start();return
                else:self.value({'error':'Not found'},404);return
                self.value({'ok':True})
            except (ValueError,OSError) as exc:self.value({'error':str(exc)},400)
    return ThreadingHTTPServer(('127.0.0.1',port),Handler)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--mode',choices=('offline','demo','live','replay'),default='offline')
    ap.add_argument('--config',type=Path)
    ap.add_argument('--replay',type=Path)
    ap.add_argument('--port',type=int,default=8872)
    ap.add_argument('--out',type=Path,default=ROOT/'runs/console')
    args=ap.parse_args()
    config=json.loads(args.config.read_text()) if args.config else {}
    if args.mode=='live' and not config.get('bind'):ap.error('live requires --config with bind')
    # Bind HTTP before acquiring serial/UDP resources; port conflicts leave other owners alone.
    session=None
    token=secrets.token_hex(24)
    class Deferred:
        def state(self):return session.state()
        def image(self,*a):return session.image(*a)
        def record(self,*a):return session.record(*a)
        @property
        def out(self):return session.out
    server=make_server(Deferred(),args.port,token)
    state_path=ROOT/'runs/console_session.json'
    state_path.parent.mkdir(parents=True,exist_ok=True)
    try:
        session=Session(args.mode,config,args.out,args.replay)
        state_path.write_text(json.dumps(dict(profile=PROFILE,url='http://127.0.0.1:'+str(server.server_port),token=token)))
        print('Console http://127.0.0.1:%d mode=%s (no hardware writes)'%(server.server_port,args.mode),flush=True)
        server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        if session:session.close()
        server.server_close()
        if state_path.exists():
            saved=json.loads(state_path.read_text())
            if saved.get('token')==token:state_path.unlink()

if __name__=='__main__':main()
