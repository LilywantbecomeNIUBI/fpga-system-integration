"""PC console: local UI + existing video services + authenticated Nano gateway client."""
from pathlib import Path
import argparse,json,os,secrets,sys,threading,time,urllib.request,urllib.error
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlsplit,parse_qs
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'host'))
import sensing_base

class GatewayClient:
    def __init__(self,url=None,token='',live=False):
        self.url=url.rstrip('/') if url else None;self.token=token;self.live=live
        self.lock=threading.Lock();self.transport_lock=threading.RLock();self.data=None;self.received=None;self.error=None;self.lease=None
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def request(self,path,payload=None):
        with self.transport_lock:return self._request(path,payload)
    def _request(self,path,payload=None):
        if not self.live or not self.url or len(self.token)<24:raise ValueError('Live gateway URL/token not configured')
        body=json.dumps(payload,allow_nan=False).encode('utf-8') if payload is not None else None
        req=urllib.request.Request(self.url+path,data=body,headers={'X-Session-Token':self.token,'Content-Type':'application/json'})
        try:
            with self.opener.open(req,timeout=.12) as response:
                raw=response.read(2*1024*1024+1)
                if len(raw)>2*1024*1024:raise ValueError('Gateway response too large')
                return json.loads(raw.decode('utf-8'))
        except urllib.error.HTTPError as exc:
            try:detail=json.loads(exc.read(2048).decode('utf-8')).get('error',str(exc))
            except (ValueError,UnicodeError):detail=str(exc)
            raise ValueError(detail) from exc
    def poll(self):
        with self.transport_lock:self._poll()
    def _poll(self):
        try:
            data=self.request('/api/state')
            if data.get('profile')!='system_integration_v1':raise ValueError('Unexpected gateway profile')
            with self.lock:self.data=data;self.received=time.monotonic();self.error=None
        except Exception as exc:
            self.release()
            with self.lock:self.error=str(exc);self.data=None;self.received=None
    def control(self,data):
        with self.transport_lock:return self._control(data)
    def _control(self,data):
        result=self.request('/api/control',data)
        if data.get('action')=='claim':self.lease=result['lease']
        elif data.get('action') in ('stop','release'):self.lease=None
        return result
    def snapshot(self):
        with self.lock:
            age=time.monotonic()-self.received if self.received is not None else None
            return dict(online=age is not None and age<.3 and self.error is None,age_ms=age*1000 if age is not None else None,error=self.error,data=self.data)
    def release(self):
        with self.transport_lock:
            if self.lease:
                try:self.control(dict(action='release',lease=self.lease))
                except Exception:pass
                self.lease=None

    def reconnect(self,url):
        with self.transport_lock:
            self.release()
            self.url=url;self.lease=None
            with self.lock:self.data=None;self.received=None;self.error='Reconnecting'

class Session(sensing_base.Session):
    def __init__(self,mode='offline',config=None,out=None,replay=None):
        cfg=config or {};self.gateway=GatewayClient(cfg.get('gateway_url'),os.environ.get('ROBOT_GATEWAY_TOKEN',''),mode=='live')
        super().__init__(mode,cfg,out,replay)
    def serial_loop(self):
        # PC never opens the FPGA serial port. Nano is its sole owner.
        while not self.stop.is_set():self.gateway.poll();self.stop.wait(.05)
    def state(self):
        state=super().state();gateway=self.gateway.snapshot();data=gateway.get('data') or {}
        if self.mode=='live' and gateway['online']:state['imu']=data['imu']
        elif self.mode=='live':state['imu']['online']=False;state['imu']['attitude']['valid']=False
        state.update(profile='system_integration_v1',gateway=gateway,control=data.get('control'),lidar=data.get('lidar'),odometry=data.get('odometry'))
        if not gateway['online']:
            state['control']=dict(phase='offline',motor_fresh=False,motor=None,physical_control_enabled=False)
            state['lidar']=dict(online=False,scan=None)
        state['motor_outputs_enabled']=bool(gateway['online'] and state['control'].get('motor_fresh') and (state['control'].get('motor') or {}).get('armed'))
        return state
    def close(self):self.gateway.release();super().close()

def make_server(session,port,token,page='index.html'):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,value,status=200,kind='application/json'):
            body=value if isinstance(value,bytes) else json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8')
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
        def do_GET(self):
            url=urlsplit(self.path)
            if url.path=='/':self.send((ROOT/'host'/page).read_bytes(),kind='text/html; charset=utf-8')
            elif url.path in ('/motor','/tuning'):self.send((ROOT/'host/motor_tuner.html').read_bytes(),kind='text/html; charset=utf-8')
            elif url.path=='/home':self.send((ROOT/'host/home.html').read_bytes(),kind='text/html; charset=utf-8')
            elif url.path=='/imu':self.send((ROOT/'host/imu.html').read_bytes(),kind='text/html; charset=utf-8')
            elif url.path in ('/api/state','/health'):self.send(session.state())
            elif url.path in ('/camera.mjpg','/infer.mjpg'):
                if url.path=='/infer.mjpg' and session.mode=='live' and session.config.get('nano_url'):
                    # The browser receives latest inference frames directly; metadata
                    # polling must not cap video at the NanoResults polling rate.
                    self.send_response(302)
                    self.send_header('Location',session.config['nano_url'].rstrip('/')+'/infer.mjpg')
                    self.send_header('Cache-Control','no-store');self.send_header('Content-Length','0')
                    self.end_headers();return
                if not hasattr(session,'stream_image'):
                    self.send(dict(error='No video source'),503);return
                self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control','no-store');self.end_headers();last=None
                self.connection.settimeout(1)
                try:
                    while not session.stop.is_set():
                        item=session.stream_image(url.path=='/infer.mjpg')
                        if item and item[0]!=last:
                            key,encoded=item;last=key
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(encoded)).encode()+
                                b'\r\nX-Frame-Key: '+str(key).encode()+b'\r\n\r\n'+encoded+b'\r\n');self.wfile.flush()
                        session.stop.wait(1/60.)
                except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError,TimeoutError,OSError):pass
            elif url.path in ('/camera.jpg','/infer.jpg'):
                image=session.image(url.path=='/infer.jpg',parse_qs(url.query).get('key',[''])[0])
                self.send(image if image else dict(error='No fresh frame'),200 if image else 503,'image/jpeg' if image else 'application/json')
            else:self.send(dict(error='Not found'),404)
        def do_POST(self):
            host=self.headers.get('Host');origin=self.headers.get('Origin')
            browser=host in ('127.0.0.1:%d'%self.server.server_port,'localhost:%d'%self.server.server_port) and origin=='http://'+str(host)
            if not browser and self.headers.get('X-Session-Token')!=token:self.send(dict(error='Forbidden'),403);return
            try:
                if self.path=='/api/connection' and hasattr(session,'configure_connection'):
                    self.connection.settimeout(.15);size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=2048:raise ValueError('Invalid body size')
                    self.send(session.configure_connection(json.loads(self.rfile.read(size).decode('utf-8'))));return
                if self.path=='/api/control':
                    self.connection.settimeout(.15);size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=2048:raise ValueError('Invalid body size')
                    self.send(session.gateway.control(json.loads(self.rfile.read(size).decode('utf-8'))));return
                if self.path=='/api/record/start':session.record(True)
                elif self.path=='/api/record/stop':session.record(False)
                elif self.path=='/api/save':
                    image=session.image()
                    if not image or session.out is None:raise ValueError('No fresh image/output')
                    session.out.mkdir(parents=True,exist_ok=True);(session.out/('frame_'+str(time.time_ns())+'.jpg')).write_bytes(image)
                elif self.path=='/api/shutdown':
                    session.gateway.release();self.send(dict(stopping=True));threading.Thread(target=self.server.shutdown,daemon=True).start();return
                else:self.send(dict(error='Not found'),404);return
                self.send(dict(ok=True))
            except Exception as exc:self.send(dict(error=str(exc)),409)
    return ThreadingHTTPServer(('127.0.0.1',port),Handler)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=('offline','demo','live','replay'),default='offline');ap.add_argument('--config',type=Path)
    ap.add_argument('--replay',type=Path);ap.add_argument('--port',type=int,default=8872);ap.add_argument('--out',type=Path,default=ROOT/'runs/console');a=ap.parse_args()
    cfg=json.loads(a.config.read_text(encoding='utf-8')) if a.config else {}
    if a.mode=='live' and not all(cfg.get(x) for x in ('bind','gateway_url')):ap.error('live requires bind and gateway_url')
    session=None
    class Deferred:
        def __getattr__(self,key):return getattr(session,key)
    server=make_server(Deferred(),a.port,secrets.token_hex(24))
    try:
        session=Session(a.mode,cfg,a.out,a.replay);print('Console http://127.0.0.1:%d mode=%s'%(server.server_port,a.mode),flush=True);server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        if session:session.close()
        server.server_close()
if __name__=='__main__':main()
