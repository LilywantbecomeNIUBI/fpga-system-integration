"""Headless Mecanum command worker and bounded handshakes (Python 3.6+)."""
import math
import queue
import threading
import time
import json
from pathlib import Path
from datetime import datetime
from collections import deque
from console import Stream, packet, motor_profile, mm_to_control_counts, polar_displacement
from live_status import TelemetryJournal
from navigation import NavigationSession
from remote import RemoteSession

class CalibrationSession:
    """STOP, acknowledge stopped state, recalibrate, observe invalid then healthy."""
    def __init__(self,send,now,motor,options):
        if not motor or now-motor[0]>.3:raise ValueError('需要最新电机遥测才能校零')
        self.send=send;self.options=options;self.phase='stop';self.sent=now
        self.deadline=now+.3;self.sequence=send(0,(0,0,0),options);self.saw_invalid=False

    def step(self,now,motor,pose):
        if not motor or now-motor[0]>.3:raise RuntimeError('校零期间电机遥测中断')
        if self.phase=='stop' and motor[0]>self.sent and not motor[1]['armed']:
            self.phase='ack';self.sent=now;self.deadline=now+.3
            self.sequence=self.send(8,(0,0,0),self.options)
        elif self.phase=='ack' and motor[0]>self.sent and not motor[1]['armed'] and motor[1]['seq']==self.sequence:
            self.phase='calibrating';self.deadline=now+10
        if self.phase!='stop' and pose and pose[0]>self.sent:
            if not pose[1]['calibrated']:self.saw_invalid=True
        if self.phase=='calibrating':
            if motor[1]['armed']:raise RuntimeError('校零期间电机被使能，停止')
            if pose and pose[0]>self.sent and now-pose[0]<=.3 and self.saw_invalid and pose[1]['healthy'] and pose[1]['calibrated']:
                return 'IMU通信恢复、校零完成，当前朝向为0°，电机保持停止'
        if now>=self.deadline:raise RuntimeError('IMU校零超时：保持静止，并检查固件版本/传感器状态')
        return None


def motion_spec(kind, dx, dy, angle, limit, tolerance, duration, cfg, profile, *, distance_mm=200, bearing_deg=45):
    if not math.isfinite(duration) or not 0 < duration <= 3600:
        raise ValueError('执行时限应在0～3600秒之间')
    if kind=='任意角度':
        dx,dy=polar_displacement(distance_mm,bearing_deg)
        angle=0
    if kind in ('位置','任意角度'):
        op=6;values=(mm_to_control_counts(dx,profile['raw_counts_per_rev'],profile['wheel_diameter_mm'],profile['control_shift']),
                     mm_to_control_counts(dy,profile['raw_counts_per_rev'],profile['wheel_diameter_mm'],profile['control_shift']),round(angle*100))
    elif kind=='定角': op=6;values=(0,0,round(angle*100))
    else: raise ValueError('仅支持定角、相对位置和任意角度移动')
    if kind=='定角' or (abs(angle)>=180 and kind!='任意角度'):op=9;values=(values[0],values[1],round(angle*10))
    options=dict(cfg,limit=limit,tolerance=tolerance)
    packet(op,1,x=values[0],y=values[1],z=values[2],**options)
    return dict(op=op,values=values,options=options,duration=duration,outer=op in (6,9),kind=kind)


class MotionSession:
    """Non-blocking handshake; only fresh FPGA acknowledgements advance it."""
    def __init__(self,send,spec,now,motor,pose,shift):
        if not motor or now-motor[0]>.3 or motor[1]['control_shift']!=shift:
            raise ValueError('需要最新的V3电机遥测，且控制单位必须匹配')
        if spec['op']==9 and not (motor[1].get('capabilities',0)&2):raise ValueError('当前固件不支持整圈旋转，请下载扩展运动固件')
        if spec['outer'] and (not pose or now-pose[0]>.3 or not pose[1]['healthy']):
            raise ValueError('IMU尚未完成静止校准或反馈无效')
        self.send=send;self.spec=spec;self.phase='stop';self.sent=now;self.deadline=now+.2
        self.sequence=send(0,(0,0,0),spec['options']);self.next_send=now;self.end=0
        self.moved=False

    def step(self,now,motor,pose):
        if not motor or now-motor[0]>.3:raise RuntimeError('电机遥测中断，停止执行')
        m=motor[1]
        # Acknowledgements must have been received after the last command.
        fresh=motor[0]>self.sent
        if self.phase=='stop' and fresh and not m['armed']:
            self.advance('clear',3,now)
        elif self.phase=='clear' and fresh and not m['armed'] and m['faults']==0:
            self.sequence=self.advance('arm',1,now)
        elif self.phase=='arm' and fresh and m['armed'] and m['faults']==0 and m['seq']==self.sequence:
            self.sequence=self.advance('goal',self.spec['op'],now,self.spec['values']);self.moved=True
        elif self.phase=='goal' and fresh and m['armed'] and m['faults']==0 and m['seq']==self.sequence:
            self.phase='active';self.end=now+self.spec['duration'];self.next_send=now
        elif self.phase=='active':
            if m['faults'] or not m['armed']:raise RuntimeError('FPGA已停机，检查故障状态')
            if self.spec['outer']:
                if not pose or now-pose[0]>.3 or not pose[1]['healthy'] or pose[1]['faults']:
                    raise RuntimeError('姿态反馈中断或无效，停止执行')
                # M4/P4 are sent separately. A delayed completion from the
                # previous goal must not terminate this motion after its ACK.
                if (self.spec['op'] in (6,9) and pose[0]>self.sent
                    and pose[1].get('seq')==m['seq'] and pose[1]['mode']==2 and pose[1]['arrived']):
                    return '目标已到达'
            if now>=self.end:
                if self.spec['op'] in (6,9):raise RuntimeError('到达超时，已停止；未判定到达')
                return '执行时限结束'
            if now>=self.next_send:
                self.send(7 if self.spec['outer'] else self.spec['op'],(0,0,0) if self.spec['outer'] else self.spec['values'],self.spec['options'])
                self.next_send=now+.05
        if self.phase!='active' and now>=self.deadline:raise RuntimeError('FPGA命令确认超时，停止执行')
        return None

    def advance(self,phase,op,now,values=(0,0,0)):
        self.phase=phase;self.sent=now;self.deadline=now+.2
        return self.send(op,values,self.spec['options'])


TELEMETRY_OPTIONS = dict(bind='0.0.0.0',peer='192.168.50.30')
COMMAND_TRANSPORT='router'


class SerialLink(threading.Thread):
    def __init__(self,port,events,profile):
        super().__init__(daemon=True);self.port=port;self.events=events;self.profile=profile
        self.journal=None;self.requests=queue.Queue();self.close_requested=threading.Event();self.sequence=0
        self.session=None;self.motor=None;self.pose=None;self.navigation=None;self.options=dict(motor=0,encoder=0,kp=profile['initial_kp_q8'],ki=profile['initial_ki_q8'])
        self.recent_frames=deque(maxlen=100);self.previous_imu_fault=False;self.previous_imu_count=0;self.last_fault_record=-1e9
        self.remote_lease=None
        self.auto_configure_pc=False
        self.telemetry_options=None;self.command_transport=None;self.journal_factory=None
        self.link_factory=None;self.autonomy_factory=None
        self.stationary_since=None
        self.runs_dir=Path(__file__).resolve().parents[1]/'runs'

    def run(self):
        try:
            from transport import SplitLink, RouterLink
            transport=self.command_transport or COMMAND_TRANSPORT
            link_type=self.link_factory or (RouterLink if transport=='router' else SplitLink)
            link_options=dict(self.telemetry_options or TELEMETRY_OPTIONS)
            if self.auto_configure_pc:
                # Explicit Connect only. Resolve the current Wi-Fi/LAN address
                # in this worker; don't block the HTTP STOP/lease thread.
                from tools.mecanum.start_router_gui import lan_address
                from tools.task4.telemetry_endpoint import transact
                local=lan_address(link_options['peer'])
                state=transact(local,board=link_options['peer'],target=local,slot='pc')
                if state['target']!=local or not state['enabled']:
                    raise ValueError('FPGA未确认PC回传地址')
                link_options['bind']=local
                self.events.put(('log','PC回传已连接：'+local+'；Nano端点保持原值'))
            with link_type(self.port,timeout=.015,write_timeout=.1,**link_options) as self.device:
                self.journal=(self.journal_factory or TelemetryJournal)(self.runs_dir)
                self.journal.record('connected',transport=transport,**link_options)
                self.events.put(('connected',self.port));decoder=Stream()
                try:
                    while not self.close_requested.is_set():
                        stopping=False;pending=None;calibrating=False
                        while True:
                            try:request,value=self.requests.get_nowait()
                            except queue.Empty:break
                            if request=='stop':stopping=True
                            elif request=='calibrate':calibrating=True
                            else:pending=value
                        if stopping:
                            self.stop();self.events.put(('log','停止指令已发送'))
                        elif calibrating:
                            self.stop()
                            if self.motor:self.sequence=self.motor[1]['seq']
                            try:
                                self.session=CalibrationSession(self.send,time.monotonic(),self.motor,self.options)
                                self.events.put(('running',True));self.events.put(('log','IMU校零：保持车体静止约3秒，不会自动恢复运动'))
                            except ValueError as exc:self.events.put(('log',str(exc)))
                        elif pending:
                            try:
                                if self.session:raise ValueError('请先停止当前动作')
                                if self.motor:self.sequence=self.motor[1]['seq']
                                self.options=pending['options']
                                if pending['kind']=='find_red_bottle':
                                    if self.autonomy_factory is None:raise ValueError('找物执行器尚未配置')
                                    self.session=self.autonomy_factory(self,pending,time.monotonic())
                                elif pending['kind']=='navigation':
                                    self.session=NavigationSession(self.send,self.device.write,pending,time.monotonic(),self.motor,self.navigation,self.options)
                                else:self.session=MotionSession(self.send,pending,time.monotonic(),self.motor,self.pose,self.profile['control_shift'])
                                self.events.put(('running',True));self.events.put(('log','执行：'+pending['kind']))
                            except ValueError as exc:
                                self.stop();self.events.put(('fault',str(exc)));self.events.put(('log',str(exc)))
                        if self.remote_lease is not None:
                            intent=self.remote_lease.snapshot(time.monotonic())
                            if intent and (not isinstance(self.session,RemoteSession) or self.session.gesture!=intent['gesture']):
                                if self.session:self.stop(cancel_remote=False)
                                if self.motor:self.sequence=self.motor[1]['seq']
                                try:
                                    self.session=RemoteSession(intent,self.remote_lease,self.send,self.profile,time.monotonic(),
                                                               self.motor,self.pose,MotionSession,motion_spec)
                                    self.events.put(('running',True))
                                except ValueError as exc:
                                    self.stop();self.events.put(('fault',str(exc)));self.events.put(('log',str(exc)))
                        for row in decoder.feed(self.device.read(256)):
                            now=getattr(self.device,'frame_received',None) or time.monotonic()
                            if time.monotonic()-now>=.3:continue
                            self.journal.record('telemetry',row=row)
                            self.recent_frames.append(dict(received_monotonic=now,**row))
                            if row['kind']=='navigation':self.navigation=(now,row)
                            elif row['kind']=='motor':
                                if self.motor is None or now-self.motor[0]>=.3:self.stationary_since=None
                                if row['armed'] or any(row['velocity']) or any(row['pwm']):self.stationary_since=None
                                elif self.stationary_since is None:self.stationary_since=now
                                self.motor=(now,row)
                            else:
                                self.pose=(now,row)
                                count=row.get('imu_fault_count',0)
                                if (row.get('imu_fault') and not self.previous_imu_fault) or (count!=self.previous_imu_count and now-self.last_fault_record>=1):
                                    try:
                                        folder=self.runs_dir;folder.mkdir(parents=True,exist_ok=True)
                                        path=folder/('imu_fault_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
                                        path.write_text(json.dumps(dict(imu=row,motor=self.motor[1] if self.motor else None,frames=list(self.recent_frames)),ensure_ascii=False,indent=2),encoding='utf-8')
                                        self.events.put(('log','IMU故障已记录：'+str(path)))
                                        self.last_fault_record=now
                                    except OSError as exc:self.events.put(('log','无法保存IMU故障记录：'+str(exc)))
                                self.previous_imu_fault=bool(row.get('imu_fault'))
                                self.previous_imu_count=count
                            self.events.put(('frame',dict(row,_received_monotonic=now)))
                        if self.session:
                            try:
                                if isinstance(self.session,NavigationSession) or getattr(self.session,'uses_navigation',False):result=self.session.step(time.monotonic(),self.motor,self.pose,self.navigation)
                                else:result=self.session.step(time.monotonic(),self.motor,self.pose)
                                if result:self.stop();self.events.put(('log',result+'，已发送STOP'))
                            except (ValueError,RuntimeError) as exc:
                                self.stop();self.events.put(('fault',str(exc)));self.events.put(('log',str(exc)))
                finally:
                    # Only a deliberate STOP/session/close sends STOP. Merely
                    # opening the port starts read-only, with no automatic ARM.
                    if self.session:self.stop()
        except Exception as exc:self.events.put(('log','连接错误：'+str(exc)))
        finally:
            if self.journal:self.journal.close()
            self.events.put(('disconnected',None))

    def send(self,op,values,options):
        self.sequence=(self.sequence+1)&255
        self.device.write(packet(op,self.sequence,x=values[0],y=values[1],z=values[2],**options))
        if self.journal:self.journal.record('command',op=op,seq=self.sequence,values=values,options=options)
        return self.sequence

    def stop(self,cancel_remote=True):
        if cancel_remote and self.remote_lease is not None:self.remote_lease.cancel()
        self.send(0,(0,0,0),self.options);self.session=None;self.events.put(('running',False))
