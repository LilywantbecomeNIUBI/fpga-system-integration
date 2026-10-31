"""Independent Nano YOLO receiver + read-only result endpoint; no GUI required."""
import argparse
import base64
from collections import deque
from http.server import BaseHTTPRequestHandler,HTTPServer
import json
import ipaddress
from pathlib import Path
import signal
import socket
from socketserver import ThreadingMixIn
import threading
import time
import uuid
from urllib.parse import urlsplit,parse_qs
import cv2
from vision_dashboard_common import Capture,create_detector,PROFILE,select_feedback,jpeg
from vision_v23_protocol import model_detection
from shared.perception import PerceptionEvents
from shared.live_capture import Capture
from shared.capture_process import CaptureProcess
from shared.perception_process import PerceptionProcess
from vision_protocol import parse_status
from vision_models import MODEL_IDS,model_paths,resolve_model


class Server(ThreadingMixIn,HTTPServer):
    daemon_threads=True
    allow_reuse_address=True


class StaticCapture:
    """Explicit offline fixture. No socket and no synthetic camera frame stream."""
    def __init__(self,path):
        self.image=cv2.imread(str(path))
        if self.image is None:raise ValueError('Cannot read offline image: '+str(path))
        self.image=cv2.resize(self.image,(320,320));self.encoded=jpeg(self.image)

    def frame(self):return (0,self.image,time.monotonic(),self.encoded)
    def status(self):return dict(frames=1,packets=0,bad=0,missing_frames=0,fps=0,frame_id=0,age_ms=None,error=None,offline=True)
    def close(self):pass


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--bind',required=True);ap.add_argument('--port',type=int,default=8766)
    ap.add_argument('--fpga',default='192.168.50.30');ap.add_argument('--model-dir',required=True)
    ap.add_argument('--backend',choices=('tensorrt','opencv'),default='tensorrt')
    ap.add_argument('--model',choices=MODEL_IDS,default='yolov3-tiny')
    ap.add_argument('--engine',type=Path)
    ap.add_argument('--capture-buffer-kib',type=int,default=4096)
    ap.add_argument('--capture-process',action='store_true',help='Isolate UDP assembly from inference Python GIL')
    ap.add_argument('--events-process',action='store_true',help='Isolate event reception and parsing from inference Python GIL')
    ap.add_argument('--event-relay',help='Forward validated events independently of HTTP to this IPv4 host')
    ap.add_argument('--offline-image',type=Path,help='One static inference; never opens FPGA sockets')
    ap.add_argument('--token',required=True);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    if args.event_relay:
        if not args.events_process:ap.error('event-relay requires events-process')
        try:args.event_relay=str(ipaddress.IPv4Address(args.event_relay))
        except ValueError:ap.error('event-relay must be an IPv4 address')
    resolve_model(args.model,args.backend)
    if args.engine is None:args.engine=Path(model_paths(Path(__file__).resolve().parent,args.model_dir,args.model)['engine'])
    args.out.mkdir(parents=True,exist_ok=True)
    if not 128<=args.capture_buffer_kib<=8192:ap.error('capture-buffer-kib must be 128..8192')
    capture_class=CaptureProcess if args.capture_process else Capture
    capture=StaticCapture(args.offline_image) if args.offline_image else capture_class(args.bind,source=args.fpga,encode_preview=False,buffer_kib=args.capture_buffer_kib)
    stop=threading.Event();lock=threading.Lock()
    session=uuid.uuid4().hex;state=dict(result=None,event=None,event_stamp=None,error=None,inferences=0,model='正在加载模型')
    perception=PerceptionEvents()
    completed=deque(maxlen=120)
    semantics=events=event_process=None
    if not args.offline_image:
        semantics=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);semantics.bind((args.bind,0));semantics.connect((args.fpga,6201));semantics.settimeout(.2)
        if args.events_process:event_process=PerceptionProcess(args.bind,args.fpga,args.out/'raw_events_process.jsonl',
            (args.event_relay,6204) if args.event_relay else None)
        else:
            events=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);events.bind((args.bind,6202));events.settimeout(.1)

    def receive_events():
        while not stop.is_set():
            try:
                payload,peer=events.recvfrom(2048)
                if peer!=(args.fpga,6202):continue
                with lock:
                    kind,event=perception.ingest(payload)
                    if kind=='tracking':state.update(event=event,event_stamp=time.monotonic())
            except socket.timeout:continue
            except (ValueError,OSError):
                if stop.is_set():break

    def infer():
        detector=None
        try:
            detector=create_detector(args.model_dir,args.backend,args.engine,args.model);previous=None
            with lock:state['model']=detector.model
            while not stop.is_set():
                item=capture.frame()
                if item is None or item[0]==previous or time.monotonic()-item[2]>.5:
                    stop.wait(.01);continue
                fid,image,received,encoded=item;previous=fid;started=time.monotonic()
                detections=detector.detect(image);elapsed=(time.monotonic()-started)*1000
                selected=select_feedback(detections)
                payload=model_detection(fid,selected['bbox'] if selected else None,
                                        confidence=selected['confidence'] if selected else 0,
                                        inference_ms=min(65535,int(elapsed)))
                feedback=dict(kind='bottle' if selected else 'none',threshold=.5,reply=None,error=None)
                if semantics is not None:
                    semantics.send(payload)
                    try:feedback['reply']=parse_status(semantics.recv(2048))
                    except (socket.timeout,ValueError,OSError) as exc:feedback['error']=str(exc) or 'FPGA reply timeout'
                else:
                    feedback.update(kind='offline',error='静态图片验证：未连接 FPGA，未发送反馈')
                if encoded is None:encoded=jpeg(image)
                finished=time.monotonic()
                with lock:
                    completed.append(finished)
                    state['inferences']+=1
                    state['result']=dict(session=session,sequence=state['inferences'],frame_id=fid,
                        inference_ms=elapsed,receive_to_result_ms=(finished-received)*1000,
                        detections=detections,feedback=feedback,jpeg_b64=base64.b64encode(encoded).decode('ascii'),
                        finished=finished)
        except Exception as exc:
            with lock:state['error']=repr(exc)
        finally:
            if detector is not None:detector.close()

    def snapshot():
        now=time.monotonic()
        with lock:
            external=event_process.snapshot(now) if event_process else None
            result=dict(state['result']) if state['result'] else None
            event=external['event'] if external else dict(state['event']) if state['event'] else None
            if result:result['age_ms']=(now-result.pop('finished'))*1000
            return dict(profile=PROFILE,session=session,capture=capture.status(),result=result,event=event,
                        flow=external['flow'] if external else perception.flow_snapshot(now),
                        event_age_ms=external['event_age_ms'] if external else (now-state['event_stamp'])*1000 if state['event_stamp'] else None,
                        event_receiver=external['stats'] if external else None,
                        event_receiver_error=external['error'] if external else None,
                        error=state['error'],model=state['model'],backend=args.backend,
                        model_id=args.model,
                        inference_fps=(len(completed)-1)/(completed[-1]-completed[0]) if len(completed)>1 else 0,
                        offline=bool(args.offline_image),inferences=state['inferences'])

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            parsed=urlsplit(self.path)
            if parsed.path=='/infer.mjpg':
                self.send_response(200)
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control','no-store');self.end_headers()
                self.connection.settimeout(1);last=None
                try:
                    while not stop.is_set():
                        with lock:
                            result=state['result']
                            item=(result['sequence'],result['jpeg_b64']) if result and time.monotonic()-result['finished']<.5 else None
                        if item and item[0]!=last:
                            last=item[0];encoded=base64.b64decode(item[1])
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(encoded)).encode()+
                                b'\r\nX-Frame-Key: '+(session+'-'+str(last)).encode()+b'\r\n\r\n'+encoded+b'\r\n')
                            self.wfile.flush()
                        stop.wait(.005)
                except (OSError,ValueError):pass
                return
            if parsed.path not in ('/health','/api/result'):
                self.send_error(404);return
            data=snapshot();result=data.get('result')
            if result and parse_qs(parsed.query).get('after',[''])[0]==result['session']+'-'+str(result['sequence']):
                result.pop('jpeg_b64',None)
            body=json.dumps(data,allow_nan=False).encode('utf-8')
            self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_POST(self):
            if self.path!='/api/shutdown' or self.headers.get('X-Session-Token')!=args.token:
                self.send_error(403);return
            self.send_response(200);self.send_header('Content-Length','0');self.end_headers();stop.set()
            threading.Thread(target=server.shutdown,daemon=True).start()
    server=Server((args.bind,args.port),Handler)
    def end(*_):
        stop.set();threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,end);signal.signal(signal.SIGINT,end)
    workers=[]
    for target in ((receive_events,infer) if events is not None else (infer,)):
        worker=threading.Thread(target=target,daemon=True);worker.start();workers.append(worker)
    try:
        print('Nano YOLO endpoint ready',args.bind,args.port,flush=True);server.serve_forever(poll_interval=.2)
    finally:
        stop.set()
        for worker in workers:worker.join(3)
        item=capture.frame()
        if item and semantics is not None:
            try:
                stop.wait(.08);semantics.send(model_detection((item[0]-2)&65535))
            except OSError:pass
        record=snapshot()
        if record['result']:record['result'].pop('jpeg_b64',None)
        (args.out/'exit_status.json').write_text(json.dumps(record,indent=2))
        capture.close();server.server_close()
        if event_process:event_process.close()
        if semantics is not None:semantics.close()
        if events is not None:events.close()


if __name__=='__main__':main()
