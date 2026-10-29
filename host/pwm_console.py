"""Local pure-PWM console. One serial worker owns UART; browser never opens the serial port."""
import argparse,csv,io,json,secrets,struct,sys,threading,time,uuid
from datetime import datetime
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'.deps'),str(ROOT/'host')]
from motor_protocol import crc16,StatusStream


def encode(op,seq,left=0,right=0):
    if any(type(x) is not int for x in (op,seq,left,right)):raise ValueError('Integer command required')
    if op not in (0,1,2,3) or not 0<=left<=200 or not 0<=right<=200 or (left and right):raise ValueError('Single wheel, 0..200 permille only')
    if op!=2 and (left or right):raise ValueError('Only DRIVE accepts PWM')
    body=b'\xa5\x5a'+struct.pack('<BBBBhhHH',1,op,seq&255,34,left,right,0,0)
    return body+struct.pack('<H',crc16(body))


class Control:
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.lock=threading.RLock();self.connected=False;self.motor=None;self.received=None
        self.phase='offline';self.reason='尚未连接';self.owner=None;self.run_id=None;self.client_seq=-1
        self.sequence=0;self.last_send=-1;self.stage_sent=None;self.arm_sequence=None;self.stop_pending=False
        self.deadline=0;self.run_until=None;self.spec=None;self.baseline=None;self.last_result=None;self.history=[]
    def fresh(self):return self.connected and self.received is not None and self.clock()-self.received<.2
    def compatible(self):return self.fresh() and self.motor['flags']==34
    def connect(self):
        with self.lock:
            self.connected=True;self.motor=None;self.received=None;self.stop('串口已连接，保持停机')
    def disconnect(self):
        with self.lock:
            self.connected=False;self.motor=None;self.received=None;self.stop('串口断开，须重新手动测试');self.phase='offline'
    def stop(self,reason='操作员 STOP'):
        with self.lock:
            self.owner=None;self.run_id=None;self.run_until=None;self.phase='stopping' if self.connected else 'offline'
            self.reason=reason;self.stage_sent=None;self.stop_pending=True
    def finish(self):
        if self.spec is not None:
            result=dict(self.spec,ended_at=datetime.now().isoformat(timespec='seconds'),reason=self.reason)
            if self.baseline and self.motor:
                for side in ('left','right'):
                    for label,key in [('logic_edges','position'),('pad_edges','invalid'),('pad_high_clocks','velocity')]:
                        field=side+'_'+key;result[side+'_'+label]=(self.motor[field]-self.baseline[field])&0xffffffff
            self.last_result=result;self.history.append(result);self.history=self.history[-1000:];self.spec=None;self.baseline=None
        self.phase='stopped';self.stage_sent=None
    def ingest(self,motor):
        with self.lock:
            if self.motor:
                delta=(motor['packet_count']-self.motor['packet_count'])&65535
                if delta==0:return
                if delta>16:self.stop('FPGA计数重启或遥测缺口，保持停机')
            self.motor=dict(motor);self.received=self.clock()
            if self.owner and (motor['flags']!=34 or (self.phase=='running' and motor['faults'])):self.stop('固件模式改变或 FPGA 故障，已停车')
    def request(self,data):
        with self.lock:
            if not isinstance(data,dict):raise ValueError('请求格式错误')
            action=data.get('action')
            if action=='stop':self.stop();return dict(ok=True)
            now=self.clock()
            if action=='test':
                if not self.compatible():raise ValueError('未连接到纯PWM诊断固件，不能测试')
                if self.phase!='stopped' or self.owner or self.motor['armed']:raise ValueError('请先等待停机确认；不能重叠测试')
                if data.get('setup_confirmed') is not True:raise ValueError('请确认双轮悬空、人员远离')
                channel=data.get('channel');duty=data.get('duty_permille');duration=data.get('duration_ms')
                if channel not in ('M1','M2'):raise ValueError('请选择M1或M2')
                if type(duty) is not int or not 0<=duty<=200:raise ValueError('占空比范围0～20%')
                if type(duration) is not int or not 100<=duration<=5000:raise ValueError('时长范围100～5000ms，硬件最长5000ms')
                client=data.get('client_id');seq=data.get('sequence')
                if not isinstance(client,str) or not 16<=len(client)<=80 or type(seq) is not int or seq<0:raise ValueError('无效控制会话')
                self.owner=client;self.run_id=uuid.uuid4().hex;self.client_seq=seq;self.deadline=now+.2
                self.spec=dict(run_id=self.run_id,channel=channel,duty_permille=duty,duration_ms=duration,started_at=datetime.now().isoformat(timespec='seconds'))
                self.baseline=dict(self.motor);self.phase='stop_ack';self.reason='准备零输出确认';self.stage_sent=None;self.run_until=None
                return dict(run_id=self.run_id)
            if action=='heartbeat':
                if not self.owner or now>=self.deadline or not self.fresh():
                    if self.owner:self.stop('页面或遥测超时，已停车')
                    raise ValueError('测试已结束或控制会话过期')
                if data.get('client_id')!=self.owner or data.get('run_id')!=self.run_id:raise ValueError('控制会话不匹配')
                seq=data.get('sequence')
                if type(seq) is not int or seq<=self.client_seq:raise ValueError('重复或旧命令')
                self.client_seq=seq;self.deadline=now+.2;return dict(ok=True)
            raise ValueError('不支持的操作')
    def emit(self,op,now):
        self.sequence=(self.sequence+1)&255;self.last_send=now
        left=right=0
        if op==2 and self.spec:
            if self.spec['channel']=='M1':left=self.spec['duty_permille']
            else:right=self.spec['duty_permille']
        return encode(op,self.sequence,left,right)
    def poll(self):
        with self.lock:
            now=self.clock()
            if self.owner and (now>=self.deadline or not self.compatible()):self.stop('页面或遥测超时，已停车')
            if self.owner and self.phase=='running' and now>=self.run_until:self.stop('限时输出完成')
            if self.stop_pending:
                self.stop_pending=False;self.stage_sent=now
                return [self.emit(0,now)] if self.connected else []
            if self.phase=='stopping':
                if self.fresh() and self.received>self.stage_sent and not self.motor['armed'] and self.motor['left_applied']==self.motor['right_applied']==0:self.finish()
                elif self.connected and now-self.last_send>=.1:return [self.emit(0,now)]
                return []
            stages={'stop_ack':0,'clear_ack':3,'arm_ack':1}
            if self.owner and self.phase in stages:
                if self.stage_sent is None:
                    self.stage_sent=now;packet=self.emit(stages[self.phase],now)
                    if self.phase=='arm_ack':self.arm_sequence=self.sequence
                    return [packet]
                if now-self.stage_sent>=.2:self.stop('FPGA确认超时，已停车');return self.poll()
                if self.received<=self.stage_sent:return []
                m=self.motor
                if self.phase=='stop_ack' and not m['armed']:self.phase='clear_ack';self.stage_sent=None
                elif self.phase=='clear_ack' and not m['armed'] and m['faults']==0:self.phase='arm_ack';self.stage_sent=None
                elif self.phase=='arm_ack' and m['armed'] and not m['faults'] and m['last_sequence']==self.arm_sequence:
                    self.phase='running';self.reason='固定PWM输出中';self.run_until=now+self.spec['duration_ms']/1000.;return [self.emit(2,now)]
                return []
            if self.owner and self.phase=='running':
                if self.motor['faults'] or (not self.motor['armed'] and self.received>self.last_send):self.stop('FPGA已自动停车');return self.poll()
                if now-self.last_send>=.04:return [self.emit(2,now)]
            return []
    def snapshot(self):
        with self.lock:
            return dict(connected=self.connected,fresh=self.fresh(),compatible=self.compatible(),phase=self.phase,reason=self.reason,
                motor=dict(self.motor) if self.motor else None,run_id=self.run_id,spec=dict(self.spec) if self.spec else None,
                remaining_ms=max(0,(self.run_until-self.clock())*1000) if self.run_until is not None else None,
                last_result=self.last_result,history=self.history[-30:],frequency_hz=10000,max_duty_permille=200,max_duration_ms=5000)


class Session:
    def __init__(self,port='COM20',live=False,out=None,serial_factory=None):
        self.control=Control();self.port=port;self.live=live;self.out=Path(out or ROOT/'runs/pwm_console');self.stop_event=threading.Event()
        self.error=None;self.serial_factory=serial_factory;self.worker=None
        if live:
            self.out.mkdir(parents=True,exist_ok=True);self.worker=threading.Thread(target=self.serial_loop,daemon=True);self.worker.start()
    def serial_loop(self):
        if self.serial_factory is None:
            try:
                import serial
                self.serial_factory=serial.Serial
            except ImportError as exc:
                self.error=str(exc);return
        with (self.out/'uart.bin').open('ab') as raw,(self.out/'events.jsonl').open('a',encoding='utf-8') as log:
            def record(kind,data):log.write(json.dumps(dict(time=time.time(),kind=kind,data=data),ensure_ascii=False)+'\n');log.flush()
            while not self.stop_event.is_set():
                try:
                    with self.serial_factory(self.port,115200,timeout=.005,write_timeout=.05,exclusive=True) as stream:
                        parser=StatusStream();self.control.connect();record('connected',dict(port=self.port))
                        while not self.stop_event.is_set():
                            for packet in self.control.poll():
                                if stream.write(packet)!=len(packet):raise OSError('串口写入不完整')
                                record('command',dict(op=packet[3],sequence=packet[4],left=int.from_bytes(packet[6:8],'little',signed=True),right=int.from_bytes(packet[8:10],'little',signed=True)))
                            data=stream.read(min(max(stream.in_waiting,1),4096))
                            if data:
                                raw.write(data);raw.flush()
                                for motor in parser.feed(data):self.control.ingest(motor);record('status',motor)
                            self.error=None
                        stream.write(encode(0,0));record('shutdown_stop',{})
                except Exception as exc:
                    self.error=str(exc);self.control.disconnect();record('serial_error',str(exc));self.stop_event.wait(.5)
    def state(self):
        data=self.control.snapshot();data.update(serial_port=self.port,mode='live' if self.live else 'offline',serial_error=self.error,log_dir=str(self.out));return data
    def close(self):
        self.control.stop('控制台关闭');self.stop_event.set()
        if self.worker:self.worker.join(2)
    def csv(self):
        output=io.StringIO();names=['started_at','ended_at','channel','duty_permille','duration_ms','left_logic_edges','right_logic_edges','left_pad_edges','right_pad_edges','left_pad_high_clocks','right_pad_high_clocks','reason']
        writer=csv.DictWriter(output,fieldnames=names,extrasaction='ignore');writer.writeheader()
        with self.control.lock:writer.writerows(self.control.history)
        return ('\ufeff'+output.getvalue()).encode('utf-8')


def make_server(session,port=8874,token=None):
    token=token or secrets.token_hex(24)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,data,code=200,kind='application/json; charset=utf-8'):
            body=data if isinstance(data,bytes) else json.dumps(data,ensure_ascii=False,allow_nan=False).encode('utf-8')
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            path=urlsplit(self.path).path
            if path=='/':self.send((ROOT/'host/pwm_console.html').read_bytes(),kind='text/html; charset=utf-8')
            elif path=='/api/state':self.send(session.state())
            elif path=='/api/log.csv':self.send(session.csv(),kind='text/csv; charset=utf-8')
            else:self.send(dict(error='不存在的地址'),404)
        def do_POST(self):
            host=self.headers.get('Host');origin=self.headers.get('Origin')
            browser=host in ('127.0.0.1:%d'%self.server.server_port,'localhost:%d'%self.server.server_port) and origin=='http://'+str(host)
            if not browser and self.headers.get('X-Session-Token')!=token:self.send(dict(error='未授权'),403);return
            try:
                if self.path=='/api/shutdown':session.control.stop('控制台关闭');self.send(dict(ok=True));threading.Thread(target=self.server.shutdown,daemon=True).start();return
                if self.path!='/api/control':self.send(dict(error='不存在的地址'),404);return
                self.connection.settimeout(.15);size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=2048:raise ValueError('请求长度错误')
                self.send(session.control.request(json.loads(self.rfile.read(size).decode('utf-8'))))
            except Exception as exc:self.send(dict(error=str(exc)),409)
    return ThreadingHTTPServer(('127.0.0.1',port),Handler)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--live',action='store_true');ap.add_argument('--serial-port',default='COM20');ap.add_argument('--port',type=int,default=8874);ap.add_argument('--out',type=Path)
    args=ap.parse_args();out=args.out or ROOT/'runs/pwm_console'/datetime.now().strftime('%Y%m%d_%H%M%S');session=Session(args.serial_port,args.live,out);server=make_server(session,args.port)
    try:print('PWM console http://127.0.0.1:%d mode=%s'%(server.server_port,'live' if args.live else 'offline'),flush=True);server.serve_forever()
    except KeyboardInterrupt:pass
    finally:session.close();server.server_close()


if __name__=='__main__':main()
