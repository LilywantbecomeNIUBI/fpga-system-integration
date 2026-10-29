"""Deterministic control lease/ack state machine. Only the serial worker emits bytes."""
import math,threading,time,uuid
from shared.calibration import Calibration
from motor_protocol import command

class Control:
    def __init__(self,config=None,clock=time.monotonic):
        self.clock=clock;self.lock=threading.RLock();self.connected=False;self.motor=None;self.motor_time=None
        self.owner=None;self.deadline=0;self.client_seq=-1;self.phase='offline';self.reason='not_connected'
        self.sequence=0;self.last_send=-1;self.phase_sent=-1;self.arm_sequence=None;self.stop_pending=False
        self.targets=[0,0];self.flags=1;self.kp=256;self.ki=0
        self.test_until=None
        self.event_sink=None
        self.calibration=Calibration((config or {}).get('calibration'));self.events=[]
    def event(self,kind):
        self.events.append(dict(monotonic=self.clock(),kind=kind,phase=self.phase,reason=self.reason))
        self.events=self.events[-100:]
        if self.event_sink:self.event_sink(dict(self.events[-1]))
    def revoke(self,reason):
        with self.lock:
            self.test_until=None;self.owner=None;self.targets=[0,0];self.phase='stopped' if self.connected else 'offline'
            self.reason=reason;self.stop_pending=True;self.event('stop')
    def connect(self):
        with self.lock:
            self.connected=True;self.motor=None;self.motor_time=None;self.revoke('serial_connected_requires_new_arm')
    def disconnect(self):
        with self.lock:
            self.connected=False;self.motor=None;self.motor_time=None;self.revoke('serial_disconnected')
    def ingest(self,motor,now=None):
        with self.lock:
            restarted=False
            if self.motor and 'packet_count' in motor and 'packet_count' in self.motor:
                delta=(motor['packet_count']-self.motor['packet_count'])&65535
                if delta==0:return
                if delta>16:self.revoke('fpga_counter_restart_or_large_gap');restarted=True
            now=self.clock() if now is None else now;self.motor=dict(motor);self.motor_time=now
            if self.phase=='active' and (motor['faults'] or not motor['armed']):self.revoke('fpga_stopped')
            return 'restart' if restarted else 'sample'
    def fresh(self,now):return self.connected and self.motor_time is not None and now-self.motor_time<.2
    def expire(self,now):
        if self.test_until is not None and now>=self.test_until:self.revoke('timed_test_complete')
        if self.owner and (now>=self.deadline or not self.fresh(now)):self.revoke('control_or_motor_timeout')
    def request(self,data):
        with self.lock:
            now=self.clock();self.expire(now)
            if not isinstance(data,dict):raise ValueError('Object required')
            action=data.get('action')
            if action=='stop':self.revoke('operator_stop');return self.snapshot()
            if action=='claim':
                if self.owner:raise ValueError('Control already owned')
                if not self.fresh(now):raise ValueError('No fresh FPGA status')
                self.owner=uuid.uuid4().hex;self.deadline=now+.2;self.client_seq=-1;self.phase='claimed';self.reason=None
                return dict(lease=self.owner,phase=self.phase)
            if not self.owner or data.get('lease')!=self.owner:raise ValueError('Expired or invalid control lease')
            if action=='release':self.revoke('control_session_closed');return self.snapshot()
            seq=data.get('sequence')
            if isinstance(seq,bool) or not isinstance(seq,int) or seq<=self.client_seq:raise ValueError('Stale client command')
            if action=='arm':
                if self.phase!='claimed':raise ValueError('ARM requires a new claim')
                flags=data.get('flags',1);kp=data.get('kp',256);ki=data.get('ki',0)
                if any(isinstance(x,bool) or not isinstance(x,int) for x in (flags,kp,ki)) or not 0<=flags<=31:raise ValueError('Invalid configuration')
                command(1,0,flags,0,0,kp,ki)
                if not flags&1 or kp==0:raise ValueError('Closed loop Kp must be positive')
                self.flags,self.kp,self.ki=flags,kp,ki;self.phase='stop_ack';self.phase_sent=-1;self.targets=[0,0]
            elif action=='test':
                if self.phase!='active':raise ValueError('Test requires explicit ARM acknowledgement')
                duration=data.get('duration_ms')
                if isinstance(duration,bool) or not isinstance(duration,int) or not 100<=duration<=5000:raise ValueError('Test duration must be 100..5000 ms')
                if self.test_until is not None:raise ValueError('Test already running')
                targets=[data.get('left',0),data.get('right',0)]
                if any(isinstance(x,bool) or not isinstance(x,int) for x in targets):raise ValueError('Integer wheel targets required')
                command(2,0,self.flags,*targets,self.kp,self.ki)
                self.targets=targets;self.test_until=now+duration/1000;self.event('timed_test_started')
            elif action=='heartbeat':
                held=data.get('held',False)
                if not isinstance(held,bool):raise ValueError('held must be boolean')
                targets=[0,0]
                if held:
                    if self.phase!='active':raise ValueError('Not armed')
                    units=data.get('units','counts')
                    if units=='physical':
                        if not self.flags&1:raise ValueError('Physical control requires speed loop')
                        d=self.calibration.data
                        if not self.calibration.valid or (self.flags&30) != ((d['motor_reverse_mask']<<1)|(d['encoder_reverse_mask']<<3)):raise ValueError('Control polarities do not match calibration')
                        targets=self.calibration.targets(data.get('v'),data.get('w'))
                    elif units=='counts':
                        targets=[data.get('left',0),data.get('right',0)]
                        if any(isinstance(x,bool) or not isinstance(x,int) for x in targets):raise ValueError('Integer wheel targets required')
                        command(2,0,self.flags,targets[0],targets[1],self.kp,self.ki)
                    else:raise ValueError('Unsupported units')
                if self.test_until is None:self.targets=targets
            else:raise ValueError('Unsupported action')
            self.client_seq=seq;self.deadline=now+.2
            return dict(phase=self.phase,targets=self.targets)
    def emit(self,op,now):
        self.sequence=(self.sequence+1)&255;self.last_send=now
        return command(op,self.sequence,self.flags,*(self.targets if op==2 else (0,0)),kp=self.kp,ki=self.ki)
    def poll(self):
        with self.lock:
            now=self.clock();self.expire(now)
            if self.stop_pending:
                self.stop_pending=False
                return [self.emit(0,now)] if self.connected else []
            if not self.owner:return []
            stages={'stop_ack':0,'clear_ack':3,'arm_ack':1}
            if self.phase in stages:
                if self.phase_sent<0:
                    self.phase_sent=now;packet=self.emit(stages[self.phase],now)
                    if self.phase=='arm_ack':self.arm_sequence=self.sequence
                    return [packet]
                if now-self.phase_sent>=.2:self.revoke('arm_ack_timeout');return self.poll()
                if self.motor_time is None or self.motor_time<=self.phase_sent:return []
                m=self.motor
                if self.phase=='stop_ack' and not m['armed']:self.phase='clear_ack';self.phase_sent=-1
                elif self.phase=='clear_ack' and not m['armed'] and m['faults']==0:self.phase='arm_ack';self.phase_sent=-1
                elif self.phase=='arm_ack' and m['armed'] and not m['faults'] and m['last_sequence']==self.arm_sequence:
                    self.phase='active';self.event('armed')
                return []
            if self.phase=='active' and now-self.last_send>=.05:return [self.emit(2,now)]
            return []
    def snapshot(self):
        with self.lock:
            now=self.clock()
            return dict(timed_test_supported=True,connected=self.connected,motor_fresh=self.fresh(now),motor_age_ms=(now-self.motor_time)*1000 if self.motor_time is not None else None,
                motor=dict(self.motor) if self.motor else None,phase=self.phase,reason=self.reason,owned=bool(self.owner),targets=list(self.targets),
                test_remaining_ms=max(0,(self.test_until-now)*1000) if self.test_until is not None else None,
                flags=self.flags,kp=self.kp,ki=self.ki,physical_control_enabled=self.calibration.valid)
