"""Nano gateway, Python 3.6+. No IO until explicitly started in live mode."""
from pathlib import Path
import argparse,hmac,json,os,queue,signal,sys,threading,time
from http.server import BaseHTTPRequestHandler,HTTPServer
from socketserver import ThreadingMixIn
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'host'));sys.path.insert(0,str(ROOT/'nano'))
from control import Control
from shared.streams import MixedStream,LidarStream
from shared.imu_state import ImuState
from shared.calibration import Odometry

class Recorder:
    def __init__(self,path):
        self.path=Path(path);self.path.mkdir(parents=True,exist_ok=False);self.queue=queue.Queue(1024);self.drops=0;self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def put(self,kind,data):
        try:self.queue.put_nowait((kind,data,time.monotonic(),time.time()))
        except queue.Full:self.drops+=1
    def run(self):
        try:
            with (self.path/'events.jsonl').open('x',encoding='utf-8') as events,(self.path/'fpga_uart.bin').open('xb') as uart,(self.path/'lidar_uart.bin').open('xb') as lidar:
                while True:
                    item=self.queue.get()
                    if item is None:break
                    kind,data,received,wall=item
                    if kind in ('uart','lidar'):
                        stream=uart if kind=='uart' else lidar
                        offset=stream.tell();stream.write(data)
                        data=dict(offset=offset,length=len(data),file='fpga_uart.bin' if kind=='uart' else 'lidar_uart.bin')
                    events.write(json.dumps(dict(monotonic=received,wall_time=wall,kind=kind,data=data),allow_nan=False)+'\n');events.flush()
        except OSError as exc:self.error=str(exc)
    def close(self):
        try:self.queue.put(None,timeout=1)
        except queue.Full:return
        self.thread.join(2)

class Gateway:
    def __init__(self,config=None,live=False):
        self.config=config or {};self.live=live;self.stop=threading.Event();self.control=Control(self.config)
        self.imu=ImuState();self.mixed=MixedStream();self.lidar=LidarStream();self.odometry=Odometry(self.control.calibration)
        self.lock=threading.RLock();self.lidar_time=None;self.lidar_error=None;self.serial_error=None;self.threads=[];self.recorder=None
        self.serial_connections=0;self.lidar_connections=0;self.uart_crc_base=0
        self.lidar_frames_base=0;self.lidar_scans_base=0;self.lidar_crc_base=0
        self.encoder_diagnostics={}
        self.encoder_traces={}
    def start(self):
        if not self.live:return
        import serial # Fail before opening listeners' device workers if dependency is missing.
        lidar_enabled=self.config.get('lidar_enabled',True)
        if not isinstance(lidar_enabled,bool):raise ValueError('lidar_enabled must be boolean')
        ports=[self.config.get('fpga_serial'),self.config.get('lidar_serial')]
        if not ports[0] or (lidar_enabled and (not ports[1] or ports[0]==ports[1])):raise ValueError('Configure FPGA serial and, when enabled, a distinct lidar serial')
        self.recorder=Recorder(Path(self.config.get('record_dir','runs/gateway'))/(time.strftime('%Y%m%d_%H%M%S')+'_'+os.urandom(4).hex()))
        self.control.event_sink=lambda event:self.recorder.put('control_event',event)
        targets=[self.serial_loop,self.record_loop]
        if lidar_enabled:targets.append(self.lidar_loop)
        for target in targets:
            t=threading.Thread(target=target,daemon=True);self.threads.append(t);t.start()
    def ingest_uart(self,data):
        if self.recorder:self.recorder.put('uart',data)
        for kind,row in self.mixed.feed(data):
            if kind=='motor':
                previous_faults=self.control.motor.get('faults') if self.control.motor else None
                accepted=self.control.ingest(row)
                if accepted and self.recorder and row['faults']!=previous_faults:
                    self.recorder.put('motor_fault',dict(previous=previous_faults,current=row['faults'],packet_count=row.get('packet_count')))
                if accepted:
                    with self.lock:
                        if accepted=='restart':self.odometry.reset_reference()
                        self.odometry.update(row)
            elif kind=='encoder_diagnostics':
                with self.lock:
                    self.encoder_diagnostics[row['axis']]=dict(row,received_monotonic=time.monotonic())
            elif kind=='encoder_trace':
                with self.lock:
                    self.encoder_traces[row['axis']]=dict(row,received_monotonic=time.monotonic())
            else:self.imu.feed(b'',rows=[row])
    def serial_loop(self):
        import serial
        while not self.stop.is_set():
            try:
                with serial.Serial(self.config['fpga_serial'],115200,timeout=.005,write_timeout=.05,exclusive=True) as stream:
                    self.uart_crc_base+=self.mixed.bad_crc;self.mixed=MixedStream();self.serial_connections+=1;self.control.connect()
                    with self.lock:self.encoder_diagnostics={};self.encoder_traces={}
                    with self.lock:self.odometry.reset_reference()
                    while not self.stop.is_set():
                        # Check deadlines before any potentially blocking serial operation.
                        for packet in self.control.poll():
                            if stream.write(packet)!=len(packet):raise OSError('Short FPGA write')
                            if self.recorder:self.recorder.put('command',dict(sequence=packet[4],opcode=packet[3],flags=packet[5],left=int.from_bytes(packet[6:8],'little',signed=True),right=int.from_bytes(packet[8:10],'little',signed=True)))
                        data=stream.read(min(max(stream.in_waiting,1),4096))
                        if data:self.ingest_uart(data)
                        self.serial_error=None
                    self.control.revoke('gateway_shutdown')
                    for packet in self.control.poll():stream.write(packet)
            except Exception as exc:
                self.serial_error=str(exc);self.control.disconnect()
                if self.recorder:self.recorder.put('serial_error',str(exc))
                self.stop.wait(1)
    def lidar_loop(self):
        import serial
        while not self.stop.is_set():
            try:
                with serial.Serial(self.config['lidar_serial'],921600,timeout=.02,exclusive=True) as stream:
                    with self.lock:
                        self.lidar_frames_base+=self.lidar.frames;self.lidar_scans_base+=self.lidar.scans;self.lidar_crc_base+=self.lidar.bad_crc
                        self.lidar=LidarStream();self.lidar_time=None;self.lidar_connections+=1
                    while not self.stop.is_set():
                        data=stream.read(min(max(stream.in_waiting,1),8192))
                        if data:
                            if self.recorder:self.recorder.put('lidar',data)
                            with self.lock:
                                scans=self.lidar.feed(data)
                                if scans:self.lidar_time=time.monotonic()
                            self.lidar_error=None
            except Exception as exc:
                self.lidar_error=str(exc)
                with self.lock:self.lidar_time=None
                self.stop.wait(1)
    def state(self):
        with self.lock:
            age=time.monotonic()-self.lidar_time if self.lidar_time is not None else None
            lidar=dict(enabled=self.config.get('lidar_enabled',True),online=self.config.get('lidar_enabled',True) and age is not None and age<.5,age_ms=age*1000 if age is not None else None,error=self.lidar_error,
                frames=self.lidar_frames_base+self.lidar.frames,scans=self.lidar_scans_base+self.lidar.scans,bad_crc=self.lidar_crc_base+self.lidar.bad_crc,
                connections=self.lidar_connections,scan=self.lidar.last_scan if age is not None and age<.5 else None)
            odometry=self.odometry.snapshot()
        control=self.control.snapshot();odometry['valid']=odometry['valid'] and control['motor_fresh']
        with self.lock:
            encoder_diagnostics={str(axis):dict(row,age_ms=(time.monotonic()-row['received_monotonic'])*1000)
                                 for axis,row in self.encoder_diagnostics.items()}
            encoder_traces={str(axis):dict(row,age_ms=(time.monotonic()-row['received_monotonic'])*1000)
                            for axis,row in self.encoder_traces.items()}
        return dict(profile='system_integration_v1',mode='live' if self.live else 'offline',control=control,imu=self.imu.snapshot(),lidar=lidar,
            encoder_diagnostics=encoder_diagnostics,
            encoder_traces=encoder_traces,
            odometry=odometry,serial_error=self.serial_error,serial_connections=self.serial_connections,uart_bad_crc=self.uart_crc_base+self.mixed.bad_crc,recording=dict(active=bool(self.recorder),drops=self.recorder.drops if self.recorder else 0,error=self.recorder.error if self.recorder else None))
    def record_loop(self):
        while not self.stop.wait(.1):
            if self.recorder:self.recorder.put('state',self.state())
    def close(self):
        with self.control.lock:self.control.revoke('gateway_shutdown')
        self.stop.set()
        for thread in self.threads:thread.join(2)
        self.control.disconnect()
        if self.recorder:self.recorder.close()

class Server(ThreadingMixIn,HTTPServer):daemon_threads=True
def make_server(gateway,bind,port,token):
    if len(token)<24:raise ValueError('Session token must contain at least 24 characters')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,value,status=200):
            body=json.dumps(value,allow_nan=False).encode('utf-8');self.send_response(status)
            self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
        def authorized(self):return hmac.compare_digest(self.headers.get('X-Session-Token',''),token)
        def do_GET(self):
            if not self.authorized():self.reply(dict(error='Forbidden'),403)
            elif self.path=='/api/state':self.reply(gateway.state())
            else:self.reply(dict(error='Not found'),404)
        def do_POST(self):
            if not self.authorized():self.reply(dict(error='Forbidden'),403);return
            if self.path!='/api/control':self.reply(dict(error='Not found'),404);return
            try:
                self.connection.settimeout(.1);size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=2048:raise ValueError('Invalid body size')
                data=json.loads(self.rfile.read(size).decode('utf-8'))
                self.reply(gateway.control.request(data))
            except (ValueError,TypeError,OSError) as exc:self.reply(dict(error=str(exc)),409)
    return Server((bind,port),Handler)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--live',action='store_true');a=ap.parse_args()
    config=json.loads(a.config.read_text(encoding='utf-8'));token=os.environ.get('ROBOT_GATEWAY_TOKEN','')
    gateway=Gateway(config,a.live);server=make_server(gateway,config.get('bind','127.0.0.1'),config.get('port',8873),token)
    def shutdown(*_):threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,shutdown);signal.signal(signal.SIGINT,shutdown)
    try:gateway.start();print('Gateway mode='+('live' if a.live else 'offline'),flush=True);server.serve_forever(poll_interval=.05)
    finally:gateway.close();server.server_close()
if __name__=='__main__':main()
