"""Task-4 local diagnostic console. Offline by default; sole UART owner in live mode."""
import argparse,csv,io,json,math,secrets,sys,threading,time
from collections import deque
from datetime import datetime,timezone
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tools/task4'))
from protocol import StatusStream,configuration_frames,crc16
from shared.streams import MixedStream
from shared.lidar_display import FpgaLidar
from motor_protocol import command
FAULTS={0:'启动配置/路径点/ARM条件未满足',1:'IMU或雷达健康条件失效（该位无法细分）',2:'编码器位移或航向增量越界',3:'保留位',4:'数学运算或规划执行超时',5:'导航心跳租约到期',6:'电机未ARM或已经失去ARM',7:'相对位置超出±16m'}
class Session:
 def __init__(self,mode='offline',serial_port=None,replay=None,out=None):
  self.mode=mode;self.serial_port=serial_port;self.replay=replay;self.out=Path(out) if out else None
  self.lock=threading.RLock();self.write_lock=threading.Lock();self.stop_event=threading.Event();self.connection=None
  self.nav=StatusStream();self.mixed=MixedStream();self.lidar=FpgaLidar();self.rows={};self.received={};self.events=deque(maxlen=100);self.trace=deque(maxlen=3000);self.last_trace=None
  self.history=deque(maxlen=100);self.bytes=0;self.error=None;self.last_signature=None;self.finished=False;self.recording_error=None
  self.worker=None;self.record=None;self.started=time.monotonic()
  if self.out:
   self.out.mkdir(parents=True,exist_ok=False);self.record=(self.out/'chunks.jsonl').open('w',encoding='utf-8')
 def event(self,text):self.events.append(dict(at_s=round(time.monotonic()-self.started,3),message=text))
 def ingest(self,data,now=None,record=True):
  now=time.monotonic() if now is None else now
  with self.lock:
   self.bytes+=len(data)
   if self.record and record:
    try:self.record.write(json.dumps(dict(elapsed_s=now-self.started,utc=datetime.now(timezone.utc).isoformat(),hex=data.hex()))+'\n');self.record.flush()
    except OSError as e:self.recording_error=str(e)
   for row in self.nav.feed(data):
    old=self.rows.get('nav')
    if old and any(((row[k]-old[k])&0xffffffff)>=0x80000000 for k in ('lidar_packets','plans')):
     self.history.clear();self.trace.clear();self.last_trace=None;self.event('计数回退：可能是FPGA复位或切换记录；已清空速率和轨迹参考')
    self.rows['nav']=row;self.received['nav']=now
   for kind,row in self.mixed.feed(data):
    self.rows[kind]=row;self.received[kind]=now
    self.lidar.ingest(kind,row,now)
   n=self.rows.get('nav')
   if n and self.received['nav']==now:
    point=(n['plans'],n['x_mm'],n['y_mm'])
    if point!=self.last_trace:self.trace.append([n['x_mm'],n['y_mm'],n['heading_degrees']]);self.last_trace=point
    self.history.append((now,n['lidar_packets'],n['plans']))
    signature=(n['running'],n['done'],n['permit'],n['faults'])
    if signature!=self.last_signature:self.event('导航状态变化：运行=%s 完成=%s 许可=%s 故障=0x%04x'%signature);self.last_signature=signature
 def snapshot(self,now=None):
  now=time.monotonic() if now is None else now
  with self.lock:
   ages={k:max(0,now-t) for k,t in self.received.items()};n=self.rows.get('nav');m=self.rows.get('motor');imu=self.rows.get('imu');checks=[]
   def check(name,state,detail):checks.append(dict(name=name,state=state,detail=detail))
   check('数据来源','demo' if self.mode=='demo' else 'ok' if self.mode in ('live','replay','follow') else 'unknown',self.mode+(' / 回放结束' if self.finished else ''))
   fresh=bool(n and ages.get('nav',1e9)<.3 and not self.error and not self.finished)
   check('导航遥测','ok' if fresh else 'bad','未收到任务四 C3 3C 04 01 帧；检查是否仍运行旧固件' if not n else '帧龄 %.0f ms'%(ages['nav']*1000))
   check('电机遥测','ok' if m and ages.get('motor',1e9)<.25 else 'unknown','ARM=%s，faults=0x%04x'%(m['armed'],m['faults']) if m else '未收到；回放文件可能只有导航帧')
   check('IMU原始遥测','ok' if imu and ages.get('imu',1e9)<.3 and imu.get('ready') and not imu.get('fault') and not imu.get('stale') else 'unknown','只报告已收到字段；不以主机姿态代替FPGA里程计')
   packet_rate=plan_rate=None
   if len(self.history)>1:
    a,b=self.history[0],self.history[-1];dt=b[0]-a[0]
    if dt>0:packet_rate=((b[1]-a[1])&0xffffffff)/dt;plan_rate=((b[2]-a[2])&0xffffffff)/dt
   check('雷达包推进','ok' if fresh and packet_rate is not None and packet_rate>0 else 'unknown','%.1f 包/s'%packet_rate if packet_rate is not None else '等待至少两帧；不是雷达扫描圈数')
   check('规划推进','ok' if fresh and plan_rate is not None and plan_rate>0 else 'unknown','%.1f 次/s'%plan_rate if plan_rate is not None else '任务未运行时不推进属于正常情况')
   reasons=[]
   if not fresh:reasons.append('遥测缺失或过期，无法确认当前许可')
   if n:
    reasons += [v for k,v in FAULTS.items() if n['faults']&(1<<k)]
    if not n['running']:reasons.append('任务未运行或已经结束')
    if fresh and not n['permit'] and not n['faults'] and n['running']:reasons.append('许可未开：当前协议无法区分近障锁存、覆盖不足或无可行方向')
   check('FPGA输出许可','ok' if fresh and n['permit'] else 'bad','；'.join(reasons) or 'FPGA已给许可；仍需检查电机核心/接线')
   lidar=self.lidar.snapshot(now)
   if self.finished or self.error:lidar.update(online=False,scan=None)
   check('雷达扫描上传','ok' if lidar['online'] else 'unknown','72个5°扇区最近回波；最多5Hz' if lidar['online'] else '等待六片完整扫描；旧固件未上传')
   check('导航候选方向','unknown','当前固件未上传，界面不会推测或伪造')
   return dict(mode=self.mode,online=fresh,lidar=lidar,rows=dict(self.rows),ages_ms={k:round(v*1000,1) for k,v in ages.items()},checks=checks,events=list(self.events),trace=list(self.trace),bytes=self.bytes,nav_bad_frames=self.nav.bad_crc,mixed_bad_crc=self.mixed.bad_crc,error=self.error,recording_error=self.recording_error,recording=str(self.out) if self.out else None,elapsed_s=round(now-self.started,1))
 def start(self):
  self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
 def run(self):
  try:
   if self.mode=='live':
    import serial
    with serial.Serial(self.serial_port,115200,timeout=.03,write_timeout=.1,exclusive=True) as stream:
     self.connection=stream
     while not self.stop_event.is_set():
      data=stream.read(4096)
      if data:self.ingest(data)
   elif self.mode=='follow':
    with Path(self.replay).open(encoding='utf-8') as stream:
     pending=''
     while not self.stop_event.is_set():
      line=stream.readline()
      if not line:self.stop_event.wait(.03);continue
      pending+=line
      if pending.endswith('\n'):
       self.ingest(bytes.fromhex(json.loads(pending)['hex']),record=False);pending=''
   elif self.mode=='replay':
    previous=0
    with Path(self.replay).open(encoding='utf-8') as stream:
     for line in stream:
      item=json.loads(line);elapsed=float(item['elapsed_s']);delay=max(0,elapsed-previous);previous=elapsed
      if self.stop_event.wait(delay):return
      self.ingest(bytes.fromhex(item['hex']),record=False)
    self.finished=True
   elif self.mode=='demo':
    import struct
    i=0
    while not self.stop_event.wait(.1):
     i+=1;phase=(i//70)%3;fault=32 if phase==2 else 0;permit=phase==0
     x=int(650*math.sin(i/100)*65536);y=int(300*(1-math.cos(i/100))*65536)
     body=b'\xc3\x3c\x04\x01'+struct.pack('<BBHiiihhhhHIIII',1|(4 if permit else 0),0,fault,x,y,int(i*10000),120 if permit else 0,100 if permit else 0,8 if permit else 0,9 if permit else 0,1500 if phase==0 else 350,i*10,i*200,0,0)
     self.ingest(body+struct.pack('<H',crc16(body)))
  except Exception as e:
   with self.lock:self.error=str(e);self.event('接收停止：'+str(e))
  finally:self.connection=None
 def send_stop(self):
  if self.mode!='live' or self.connection is None:raise ValueError('只有已连接的实机串口可发送STOP')
  with self.write_lock:self.connection.write(command(0,0));self.connection.flush()
  with self.lock:self.event('用户显式发送 STOP；未发送 ARM/DRIVE')
 def close(self):
  self.stop_event.set()
  if self.worker:self.worker.join(1)
  if self.record:self.record.close()
def make_server(session,port=8876):
 key=secrets.token_hex(24)
 class Handler(BaseHTTPRequestHandler):
  def log_message(self,*args):pass
  def respond(self,body,kind='application/json',status=200):
   data=body if isinstance(body,bytes) else json.dumps(body,ensure_ascii=False).encode('utf-8');self.send_response(status);self.send_header('Content-Type',kind+'; charset=utf-8');self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
  def do_GET(self):
   if self.path=='/':self.respond((ROOT/'host/task4_console.html').read_text(encoding='utf-8').replace('__KEY__',key).encode(),'text/html')
   elif self.path=='/api/state':self.respond(session.snapshot())
   elif self.path=='/api/export':self.respond(session.snapshot())
   else:self.respond(dict(error='not found'),status=404)
  def do_POST(self):
   if self.headers.get('X-Task4-Key')!=key:self.respond(dict(error='invalid local session'),status=403);return
   try:
    length=int(self.headers.get('Content-Length','0'))
    if length<0 or length>100000:raise ValueError('请求过大')
    data=json.loads(self.rfile.read(length) or b'{}')
    if self.path=='/api/stop':session.send_stop();self.respond(dict(ok=True))
    elif self.path=='/api/prepare':
     frames,seq=configuration_frames(data['calibration'],data['mission']);self.respond(dict(ok=True,packet_count=len(frames),hex=''.join(f.hex() for f in frames),next_sequence=seq,note='仅导出，未写入设备，不包含ARM/START'))
    else:self.respond(dict(error='not found'),status=404)
   except Exception as e:self.respond(dict(error=str(e)),status=400)
 return ThreadingHTTPServer(('127.0.0.1',port),Handler)
def main():
 ap=argparse.ArgumentParser(description=__doc__);g=ap.add_mutually_exclusive_group();g.add_argument('--serial');g.add_argument('--replay');g.add_argument('--follow');g.add_argument('--demo',action='store_true');ap.add_argument('--port',type=int,default=8876);ap.add_argument('--record',type=Path);args=ap.parse_args()
 mode='live' if args.serial else 'follow' if args.follow else 'replay' if args.replay else 'demo' if args.demo else 'offline'
 if args.record and mode!='live':ap.error('--record requires --serial')
 session=Session(mode,args.serial,args.follow or args.replay,args.record);session.start();server=make_server(session,args.port);print('Task4 diagnostics: http://127.0.0.1:%d / %s'%(server.server_port,mode),flush=True)
 try:server.serve_forever()
 except KeyboardInterrupt:pass
 finally:server.server_close();session.close()
if __name__=='__main__':main()
