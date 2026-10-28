"""Loopback web UI: direct FPGA video plus frame-matched Nano inference results."""
import argparse
import base64
from collections import OrderedDict
from datetime import datetime
from http.server import BaseHTTPRequestHandler,HTTPServer
import json
from pathlib import Path
from socketserver import ThreadingMixIn
import threading
import time
import uuid
import urllib.request
from urllib.parse import urlsplit,parse_qs
import cv2
import numpy as np
from vision_dashboard_common import Capture,PROFILE

DISPLAY_INTERVAL = 1 / 60


class Server(ThreadingMixIn,HTTPServer):
    daemon_threads=True
    allow_reuse_address=True


def overlay(image,detections,threshold=.25,only_bottle=False):
    image=image.copy()
    for d in detections:
        if d['confidence']<threshold or (only_bottle and d['class_id']!=39):continue
        x1,y1,x2,y2=[max(0,min(319,int(v))) for v in d['bbox']]
        color=(240,160,60) if d['class_id']==39 and d['confidence']>=.5 else (40,190,255)
        cv2.rectangle(image,(x1,y1),(x2,y2),color,1)
        label='%s %.0f%%'%(d['label'],d['confidence']*100)
        cv2.putText(image,label,(x1,max(12,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,.36,(0,0,0),3,cv2.LINE_AA)
        cv2.putText(image,label,(x1,max(12,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,.36,color,1,cv2.LINE_AA)
    return image


class NanoResults:
    def __init__(self,url,stop):
        self.url=url;self.stop=stop;self.lock=threading.Lock();self.latest=None;self.error='等待 Nano';self.error_detail=None;self.received=None
        self.history=OrderedDict();self.thread=threading.Thread(target=self.poll,daemon=True);self.thread.start()

    def poll(self):
        while not self.stop.is_set():
            started=time.monotonic()
            try:
                with self.lock:after=((self.latest or {}).get('result') or {}).get('key','')
                with urllib.request.urlopen(self.url+'/api/result?after='+after,timeout=2) as response:body=response.read(2*1024*1024+1)
                if len(body)>2*1024*1024:raise ValueError('Nano response too large')
                data=json.loads(body.decode('utf-8'))
                if data.get('profile')!=PROFILE:raise ValueError('unexpected Nano endpoint')
                result=data.get('result')
                if result:
                    if len(result['detections'])>100:raise ValueError('too many boxes')
                    for d in result['detections']:
                        if len(d['bbox'])!=4 or not all(np.isfinite(v) and 0<=v<=320 for v in d['bbox']):raise ValueError('bad bbox')
                        if not np.isfinite(d['confidence']) or not 0<=d['confidence']<=1:raise ValueError('bad score')
                    key=result['session']+'-'+str(result['sequence']);result['key']=key
                    encoded=result.pop('jpeg_b64',None)
                    if encoded is not None:
                        image=base64.b64decode(encoded,validate=True)
                        decoded=cv2.imdecode(np.frombuffer(image,np.uint8),cv2.IMREAD_COLOR)
                        if decoded is None or decoded.shape!=(320,320,3):raise ValueError('bad inference JPEG')
                    else:
                        with self.lock:cached=self.history.get(key)
                        if cached is None:raise ValueError('Nano omitted an uncached inference JPEG')
                        image=cached[0]
                    with self.lock:
                        self.history[key]=(image,result)
                        while len(self.history)>12:self.history.popitem(last=False)
                with self.lock:self.latest=data;self.received=time.monotonic();self.error=None;self.error_detail=None
            except Exception as exc:
                with self.lock:self.error='Nano 连接暂不可用，请检查设备后重新启动测试';self.error_detail=str(exc)
            self.stop.wait(max(0,DISPLAY_INTERVAL-(time.monotonic()-started)) if not self.error else .5)

    def state(self):
        with self.lock:
            return dict(online=self.received is not None and time.monotonic()-self.received<2 and not self.error,
                        poll_age_ms=(time.monotonic()-self.received)*1000 if self.received else None,
                        error=self.error,error_detail=self.error_detail,data=self.latest)

    def exact(self,key):
        with self.lock:return self.history.get(key)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--local-ip',required=True);ap.add_argument('--port',type=int,default=8765)
    ap.add_argument('--nano-host',required=True);ap.add_argument('--nano-port',type=int,default=8766)
    ap.add_argument('--token',required=True);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    stop=threading.Event();capture=Capture(args.local_ip);server_session=uuid.uuid4().hex
    nano_url='http://%s:%d'%(args.nano_host,args.nano_port);nano=NanoResults(nano_url,stop)
    html=(Path(__file__).resolve().parent/'vision_dashboard.html').read_bytes()

    def state():return dict(profile=PROFILE,server_session=server_session,pc=capture.status(),nano=nano.state(),save_root=str(args.out))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,body,kind='application/json',status=200):
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
        def json(self,value,status=200):self.send(json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8'),status=status)
        def do_GET(self):
            parsed=urlsplit(self.path)
            try:
                if parsed.path=='/':self.send(html,'text/html; charset=utf-8')
                elif parsed.path in ('/health','/api/state'):self.json(state())
                elif parsed.path=='/infer.jpg':
                    entry=nano.exact(parse_qs(parsed.query).get('key',[''])[0])
                    if entry:self.send(entry[0],'image/jpeg')
                    else:self.json(dict(error='该推理帧已过期，请等待下一帧'),404)
                elif parsed.path=='/raw.mjpg':
                    self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control','no-store');self.end_headers();last=None
                    while not stop.is_set():
                        started=time.monotonic()
                        item=capture.frame()
                        if item and item[0]!=last:
                            image=item[3];last=item[0]
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(image)).encode()+b'\r\n\r\n'+image+b'\r\n');self.wfile.flush()
                        stop.wait(max(0,DISPLAY_INTERVAL-(time.monotonic()-started)))
                else:self.send_error(404)
            except (BrokenPipeError,ConnectionResetError,TimeoutError):pass
        def do_POST(self):
            origin=self.headers.get('Origin');host=self.headers.get('Host')
            authorized=self.headers.get('X-Session-Token')==args.token or (host in ('127.0.0.1:%d'%args.port,'localhost:%d'%args.port) and origin=='http://'+host)
            if not authorized:self.send_error(403);return
            if self.path=='/api/shutdown':
                self.json(dict(stopped=True));stop.set()
                threading.Thread(target=server.shutdown,daemon=True).start();return
            if self.path!='/api/save':self.send_error(404);return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<4096:raise ValueError('invalid request length')
                request=json.loads(self.rfile.read(length));threshold=float(request.get('threshold',.25))
                if not .05<=threshold<=1:raise ValueError('invalid threshold')
                entry=nano.exact(request['key'])
                if not entry:self.json(dict(error='这一帧已过期，请重新保存'),409);return
                encoded,result=entry
                folder=args.out/('sample_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
                (folder/'inference_source.jpg').write_bytes(encoded)
                image=cv2.imdecode(np.frombuffer(encoded,np.uint8),cv2.IMREAD_COLOR)
                if not cv2.imwrite(str(folder/'detections.png'),overlay(image,result['detections'],threshold,bool(request.get('only_bottle')))):
                    raise RuntimeError('sample save failed')
                record=dict(result=result,display_threshold=threshold,only_bottle=bool(request.get('only_bottle')),state=state())
                (folder/'result.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
                self.json(dict(directory=str(folder),frame_id=result['frame_id']))
            except (ValueError,KeyError,RuntimeError) as exc:self.json(dict(error=str(exc)),400)
    server=Server(('127.0.0.1',args.port),Handler)
    try:
        print('YOLO dashboard: http://127.0.0.1:%d'%args.port,flush=True);server.serve_forever(poll_interval=.2)
    finally:
        stop.set();capture.close();nano.thread.join(3);server.server_close()
        request=urllib.request.Request(nano_url+'/api/shutdown',data=b'',headers={'X-Session-Token':args.token},method='POST')
        try:
            with urllib.request.urlopen(request,timeout=2):pass
        except Exception:pass
        (args.out/'exit_status.json').write_text(json.dumps(state(),ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':main()
