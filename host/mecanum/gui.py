"""Native desktop console. Starts offline; --demo never opens a serial port.

Visual thesis: a quiet instrument panel, navy masthead, one blue action color.
Content: connection/status, local trajectory, wheel feedback, control inspector.
Interaction: live trace/pose, selected motion pad, explicit execution and STOP.
"""
import argparse
import math
import queue
import threading
import time
import json
from pathlib import Path
from datetime import datetime
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox
from console import Stream, packet, motor_profile, mm_to_control_counts, polar_displacement
from live_status import LiveStatusWriter, TelemetryJournal


def imu_readout(row):
    """MPU6050/6500 configured +/-250 dps: 131 raw LSB per deg/s."""
    who=row.get('who',0);raw=row.get('gz',0);bias=row.get('bias',0)
    values=dict(identity=f"0x{who:02X} / {row.get('imu_error',0)}",
                raw=f'{raw:+d}',bias=f'{bias:+d}',rate=f'{(raw-bias)/131:+.3f}',
                sample=f"{row.get('sample_us',0)/1000:.1f}",yaw=f"{row.get('yaw_deg',0):+.2f}°")
    return values


def imu_diagnostic(row):
    if row.get('imu_diag_version')!=1:return '通信诊断：需新版固件'
    reason={0:'无',1:'NACK',2:'超时',3:'总线占用',4:'长度非法'}.get(row.get('imu_bus_error',0),'未知')
    lines=row.get('imu_bus_lines',0)
    return f"通信故障累计{row.get('imu_fault_count',0)}次 · 上次{reason} · SCL/SDA={(lines>>1)&1}/{lines&1} · 故障时PWM={'有' if row.get('imu_fault_pwm') else '无'}"


def drive_status(row):
    if row['faults']==1 and not row['armed']:return '上电锁定 · 待使能'
    if row['faults']:return f"故障 0x{row['faults']:04X}"
    return '已使能' if row['armed'] else '未使能'


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
    options=dict(cfg,limit=limit,tolerance=tolerance)
    packet(op,1,x=values[0],y=values[1],z=values[2],**options)
    return dict(op=op,values=values,options=options,duration=duration,outer=op in (5,6),kind=kind)


class MotionSession:
    """Non-blocking handshake; only fresh FPGA acknowledgements advance it."""
    def __init__(self,send,spec,now,motor,pose,shift):
        if not motor or now-motor[0]>.3 or motor[1]['control_shift']!=shift:
            raise ValueError('需要最新的V3电机遥测，且控制单位必须匹配')
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
                if self.spec['op']==6 and pose[1]['mode']==2 and pose[1]['arrived']:
                    return '目标已到达'
            if now>=self.end:
                if self.spec['op']==6:raise RuntimeError('到达超时，已停止；未判定到达')
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
        self.session=None;self.motor=None;self.pose=None;self.options=dict(motor=0,encoder=0,kp=512,ki=32)
        self.recent_frames=deque(maxlen=100);self.previous_imu_fault=False;self.previous_imu_count=0;self.last_fault_record=-1e9

    def run(self):
        try:
            from transport import SplitLink, RouterLink
            link_type=RouterLink if COMMAND_TRANSPORT=='router' else SplitLink
            with link_type(self.port,timeout=.015,write_timeout=.1,**TELEMETRY_OPTIONS) as self.device:
                self.journal=TelemetryJournal(Path(__file__).resolve().parents[1]/'runs')
                self.journal.record('connected',transport=COMMAND_TRANSPORT,**TELEMETRY_OPTIONS)
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
                                self.session=MotionSession(self.send,pending,time.monotonic(),self.motor,self.pose,self.profile['control_shift'])
                                self.events.put(('running',True));self.events.put(('log','执行：'+pending['kind']))
                            except ValueError as exc:self.events.put(('log',str(exc)))
                        for row in decoder.feed(self.device.read(256)):
                            now=time.monotonic()
                            self.journal.record('telemetry',row=row)
                            self.recent_frames.append(dict(received_monotonic=now,**row))
                            if row['kind']=='motor':self.motor=(now,row)
                            else:
                                self.pose=(now,row)
                                count=row.get('imu_fault_count',0)
                                if (row.get('imu_fault') and not self.previous_imu_fault) or (count!=self.previous_imu_count and now-self.last_fault_record>=1):
                                    try:
                                        folder=Path(__file__).resolve().parents[1]/'runs';folder.mkdir(exist_ok=True)
                                        path=folder/('imu_fault_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
                                        path.write_text(json.dumps(dict(imu=row,motor=self.motor[1] if self.motor else None,frames=list(self.recent_frames)),ensure_ascii=False,indent=2),encoding='utf-8')
                                        self.events.put(('log','IMU故障已记录：'+str(path)))
                                        self.last_fault_record=now
                                    except OSError as exc:self.events.put(('log','无法保存IMU故障记录：'+str(exc)))
                                self.previous_imu_fault=bool(row.get('imu_fault'))
                                self.previous_imu_count=count
                            self.events.put(('frame',row))
                        if self.session:
                            try:
                                result=self.session.step(time.monotonic(),self.motor,self.pose)
                                if result:self.stop();self.events.put(('log',result+'，已发送STOP'))
                            except (ValueError,RuntimeError) as exc:
                                self.stop();self.events.put(('log',str(exc)))
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

    def stop(self):
        self.send(0,(0,0,0),self.options);self.session=None;self.events.put(('running',False))


class App:
    BG='#f2f5f9';WHITE='#ffffff';INK='#18263c';MUTED='#697b92';BLUE='#2864df';BORDER='#dde5ef'
    COLORS=['#2864df','#12a09d','#b184e0','#e6a342']

    def __init__(self,root,demo=False):
        self.root=root;self.profile=motor_profile();self.demo=demo;self.link=None;self.running=False;self.demo_paused=False
        self.connected=False;self.motor_received=None;self.pose_received=None
        self.live_status=LiveStatusWriter(Path(__file__).resolve().parents[1]/'runs/live_status.json')
        self.export_error_reported=False
        self.events=queue.Queue();self.start=time.monotonic();self.latest_motor=None;self.latest_pose=None
        self.history=deque(maxlen=160);self.path=deque(maxlen=400);self.demo_positions=[0]*4
        self.mm_per_count=math.pi*self.profile['wheel_diameter_mm']/(self.profile['raw_counts_per_rev']/(1<<self.profile['control_shift']))
        root.title('麦轮底盘调试台 · MG513P30 GMR');root.geometry('1340x840');root.minsize(1120,760);root.configure(bg=self.BG)
        style=ttk.Style();style.theme_use('clam')
        style.configure('.',font=('Microsoft YaHei UI',10),background=self.WHITE,foreground=self.INK)
        style.configure('TNotebook',background=self.WHITE,borderwidth=0)
        style.configure('TNotebook.Tab',padding=(14,10));style.map('TNotebook.Tab',background=[('selected','#e9f0ff')],foreground=[('selected',self.BLUE)])
        style.configure('TCombobox',padding=5);style.configure('TSpinbox',padding=5)
        style.configure('Treeview',rowheight=32,background=self.WHITE,fieldbackground=self.WHITE,borderwidth=0)
        style.configure('Treeview.Heading',font=('Microsoft YaHei UI',9),background='#eef3fa',padding=7)
        self.make_ui()
        root.protocol('WM_DELETE_WINDOW',self.close);root.bind('<Escape>',lambda e:self.stop())
        self.refresh_ports();self.log('演示模式：所有数值为模拟数据，未连接硬件' if demo else '未连接。连接后仅监测，点击执行才会ARM。')
        self.export_status(force=True)
        root.after(80,self.poll)

    def label(self,parent,text,size=10,color=None,**kw):
        return tk.Label(parent,text=text,font=('Microsoft YaHei UI',size),fg=color or self.INK,bg=parent.cget('bg'),**kw)

    def button(self,parent,text,command,primary=False,danger=False,**kw):
        return tk.Button(parent,text=text,command=command,font=('Microsoft YaHei UI',10,'bold'),relief='flat',bd=0,
                         bg='#dc454e' if danger else self.BLUE if primary else '#eaf0f8',fg='white' if primary or danger else self.INK,
                         activebackground='#c8d9f5',cursor='hand2',padx=15,pady=9,**kw)

    def make_ui(self):
        header=tk.Frame(self.root,bg=self.INK,height=82);header.pack(fill='x');header.pack_propagate(False)
        tk.Label(header,text='M / 04',fg='#82aaff',bg=self.INK,font=('Consolas',20,'bold')).pack(side='left',padx=(25,18))
        block=tk.Frame(header,bg=self.INK);block.pack(side='left')
        self.label(block,'麦轮底盘调试台',20,'white').pack(anchor='w')
        self.label(block,'MG513P30—12V  /  GMR  /  MPU6050 · MPU6500',9,'#a9b8cc').pack(anchor='w')
        self.badge=self.label(header,'演示模式 · 模拟数据' if self.demo else '离线 · 未连接',11,'#ffcf7b');self.badge.pack(side='right',padx=25)
        bar=tk.Frame(self.root,bg=self.WHITE);bar.pack(fill='x',pady=(0,1))
        self.label(bar,'控制连接',10,self.MUTED).pack(side='left',padx=(24,8),pady=13)
        self.port=tk.StringVar();self.ports=ttk.Combobox(bar,textvariable=self.port,width=32 if COMMAND_TRANSPORT=='router' else 16,state='readonly');self.ports.pack(side='left')
        self.button(bar,'刷新',self.refresh_ports).pack(side='left',padx=7,pady=8)
        self.connect_button=self.button(bar,'连接设备',self.connect,primary=True);self.connect_button.pack(side='left',padx=3)
        self.demo_button=self.button(bar,'退出演示' if self.demo else '界面演示',self.toggle_demo);self.demo_button.pack(side='left',padx=7)
        self.label(bar,'115200 · 8N1 · 协议 V3',9,self.MUTED).pack(side='right',padx=24)
        body=tk.Frame(self.root,bg=self.BG);body.pack(fill='both',expand=True,padx=20,pady=16)
        body.columnconfigure(0,weight=1);body.columnconfigure(1,weight=0);body.rowconfigure(0,weight=1)
        left=tk.Frame(body,bg=self.BG);left.grid(row=0,column=0,sticky='nsew',padx=(0,16));left.rowconfigure(1,weight=1);left.columnconfigure(0,weight=1)
        stats=tk.Frame(left,bg=self.WHITE);stats.grid(row=0,column=0,sticky='ew',pady=(0,12))
        summary=tk.Frame(stats,bg=self.WHITE);summary.pack(fill='x')
        self.stat_values={}
        for n,(key,title,value) in enumerate([('imu','姿态反馈','—'),('heading','IMU 航向角','—'),('position','相对位置','—'),('fault','驱动状态','未使能')]):
            f=tk.Frame(summary,bg=self.WHITE);f.pack(side='left',fill='both',expand=True,padx=18,pady=13)
            self.label(f,title,9,self.MUTED).pack(anchor='w');v=self.label(f,value,16);v.pack(anchor='w',pady=(4,0));self.stat_values[key]=v
        hint=tk.Frame(stats,bg=self.WHITE);hint.pack(fill='x',padx=18,pady=(0,6))
        self.imu_hint=self.label(hint,'俯视：逆时针角速度为正、航向增大；顺时针相反。',8,self.MUTED,justify='left',wraplength=560);self.imu_hint.pack(side='left')
        self.button(hint,'IMU 重连 / 校零',self.calibrate_imu).pack(side='right')
        work=tk.Frame(left,bg=self.WHITE);work.grid(row=1,column=0,sticky='nsew');work.rowconfigure(1,weight=1);work.columnconfigure(0,weight=1)
        title=tk.Frame(work,bg=self.WHITE);title.grid(row=0,column=0,sticky='ew')
        self.label(title,'局部轨迹',12).pack(side='left',padx=18,pady=10)
        self.label(title,'+X 前进  ·  +Y 左移  ·  相对原点',9,self.MUTED).pack(side='right',padx=18)
        self.map=tk.Canvas(work,bg='#fbfcfe',highlightthickness=0,height=250);self.map.grid(row=1,column=0,sticky='nsew',padx=14)
        self.chart=tk.Canvas(work,bg=self.WHITE,highlightthickness=0,height=72);self.chart.grid(row=2,column=0,sticky='ew',padx=14,pady=(5,5))
        foot=tk.Frame(left,bg=self.WHITE);foot.grid(row=2,column=0,sticky='ew',pady=(12,0))
        self.label(foot,'四轮反馈',12).pack(anchor='w',padx=18,pady=(10,6))
        self.table=ttk.Treeview(foot,columns=('wheel','rpm','pwm'),show='headings',height=4,selectmode='none')
        for name,title,width in [('wheel','轮位',120),('rpm','转速 RPM',120),('pwm','输出 PWM',120)]:
            self.table.heading(name,text=title);self.table.column(name,width=width,anchor='center')
        for n,wheel in enumerate(['FL 左前','FR 右前','RL 左后','RR 右后']):self.table.insert('', 'end',iid=str(n),values=(wheel,'—','—'))
        self.table.pack(fill='x',padx=12,pady=(0,10))
        right=tk.Frame(body,bg=self.WHITE,width=362);right.grid(row=0,column=1,sticky='ns');right.grid_propagate(False)
        right.columnconfigure(0,weight=1);right.rowconfigure(5,weight=1)
        self.label(right,'运动控制',14).grid(row=0,column=0,sticky='w',padx=18,pady=(16,8))
        self.tabs=ttk.Notebook(right);self.tabs.grid(row=1,column=0,sticky='ew',padx=14)
        heading=tk.Frame(self.tabs,bg=self.WHITE);position=tk.Frame(self.tabs,bg=self.WHITE)
        self.tabs.add(heading,text='定角');self.tabs.add(position,text='相对位置')
        vector=tk.Frame(self.tabs,bg=self.WHITE);self.tabs.add(vector,text='任意角度')
        self.bearing=tk.DoubleVar(value=45);self.distance=tk.DoubleVar(value=200)
        self.field(vector,'移动方向 ° · 相对本次起始车头',self.bearing,-180,180)
        self.field(vector,'移动距离 · mm',self.distance,1,1000)
        self.label(vector,'0° 向前　90° 向左\n−90° 向右　±180° 向后\n车身保持起始朝向，按距离闭环停车。',9,self.MUTED,justify='left').pack(anchor='w',padx=16,pady=12)
        self.button(vector,'位置参数',self.position_settings).pack(anchor='e',padx=16,pady=5)
        self.angle=tk.DoubleVar(value=90);self.field(heading,'相对角度 ° · 逆时针为正',self.angle,-180,180)
        self.label(heading,'原地旋转到目标角度，到位停车。\n正角度逆时针，负角度顺时针。',9,self.MUTED,justify='left').pack(anchor='w',padx=16,pady=12)
        self.dx=tk.DoubleVar(value=200);self.dy=tk.DoubleVar(value=0);self.position_angle=tk.DoubleVar(value=0)
        self.field(position,'前进 X · mm',self.dx,-1000,1000);self.field(position,'左移 Y · mm',self.dy,-1000,1000);self.field(position,'终点相对角度 °',self.position_angle,-180,180)
        positionbar=tk.Frame(position,bg=self.WHITE);positionbar.pack(fill='x',padx=15,pady=5)
        self.label(positionbar,'以本次起点为原点',9,self.MUTED).pack(side='left')
        self.button(positionbar,'位置参数',self.position_settings).pack(side='right')
        limits=tk.Frame(right,bg=self.WHITE);limits.grid(row=2,column=0,sticky='ew',padx=14,pady=(12,4))
        self.duration=tk.DoubleVar(value=15);self.limit=tk.IntVar(value=20)
        self.small_field(limits,'时限 / s',self.duration,0);self.small_field(limits,'外环速度上限',self.limit,1)
        config=tk.Frame(right,bg='#f5f8fc');config.grid(row=3,column=0,sticky='ew',padx=14,pady=6)
        self.label(config,'速度 PI',10,self.MUTED).grid(row=0,column=0,columnspan=2,sticky='w',padx=10,pady=8)
        self.kp=tk.IntVar(value=self.profile['initial_kp_q8']);self.ki=tk.IntVar(value=self.profile['initial_ki_q8'])
        for row,(caption,var) in enumerate([('Kp / Q8',self.kp),('Ki / Q8',self.ki)],1):
            part=tk.Frame(config,bg='#f5f8fc');part.grid(row=1+(row-1)//2,column=(row-1)%2,sticky='ew',padx=8,pady=5)
            self.label(part,caption,9).pack(side='left');ttk.Entry(part,textvariable=var,width=6).pack(side='right',padx=(4,0))
        config.columnconfigure((0,1),weight=1)
        actions=tk.Frame(right,bg=self.WHITE);actions.grid(row=4,column=0,sticky='ew',padx=14,pady=(0,8));actions.columnconfigure((0,1),weight=1)
        self.execute=self.button(actions,'演示执行' if self.demo else '执行动作',self.run_motion,primary=True);self.execute.grid(row=0,column=0,sticky='ew',padx=(0,6))
        self.button(actions,'停止 · ESC',self.stop,danger=True).grid(row=0,column=1,sticky='ew')
        logbox=tk.Frame(right,bg=self.WHITE);logbox.grid(row=5,column=0,sticky='nsew',padx=14,pady=(0,10))
        self.log_text=tk.Text(logbox,height=4,font=('Microsoft YaHei UI',9),bg='#f5f8fc',fg=self.MUTED,relief='flat',wrap='word',state='disabled');self.log_text.pack(fill='both',expand=True)
        self.status=tk.StringVar(value='演示数据 · 未连接硬件' if self.demo else '未连接设备')
        tk.Label(self.root,textvariable=self.status,anchor='w',font=('Microsoft YaHei UI',9),fg=self.MUTED,bg='#e8eef6',padx=22,pady=6).pack(side='bottom',fill='x')
        body.pack_forget();body.pack(fill='both',expand=True,padx=20,pady=16)

    def position_settings(self):
        if self.running:self.log('请先停止当前动作，再修改测试参数');return
        self.kp.set(self.profile['initial_kp_q8']);self.ki.set(self.profile['initial_ki_q8'])
        self.duration.set(5);self.limit.set(20)
        self.log('已填位置参数：Kp512、Ki32、时限5秒、上限20；目标距离保持原值，未执行')

    def calibrate_imu(self):
        if self.demo:self.log('演示模式不发送校零指令');return
        if not self.link:self.log('请先连接设备，再校零');return
        self.link.requests.put(('calibrate',None))

    def field(self,parent,title,var,low,high):
        f=tk.Frame(parent,bg=self.WHITE);f.pack(fill='x',padx=16,pady=7)
        self.label(f,title,9,self.MUTED).pack(anchor='w',pady=(0,4));ttk.Spinbox(f,textvariable=var,from_=low,to=high,width=18).pack(fill='x')

    def small_field(self,parent,title,var,column):
        f=tk.Frame(parent,bg=self.WHITE);f.grid(row=0,column=column,sticky='ew',padx=5);parent.columnconfigure(column,weight=1)
        self.label(f,title,9,self.MUTED).pack(anchor='w');ttk.Entry(f,textvariable=var,width=13).pack(fill='x',pady=5)

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            names=[p.device for p in list_ports.comports()]
        except ImportError:names=[]
        if COMMAND_TRANSPORT=='router':names=['路由器 UDP · '+TELEMETRY_OPTIONS['peer']]
        self.ports['values']=names
        if not self.port.get() and names:self.port.set(names[0])

    def connect(self):
        if self.link:
            self.link.close_requested.set();return
        if self.demo:self.log('请先退出演示，再选择控制连接');return
        if not self.port.get():self.log('请选择控制连接；刷新后再连接');return
        self.link=SerialLink(self.port.get(),self.events,self.profile);self.link.start()
        self.log('正在连接，仅监测，不自动ARM')

    def toggle_demo(self):
        if self.link:self.log('请先断开真实设备');return
        self.demo=not self.demo;self.demo_paused=False;self.running=False;self.latest_motor=None;self.latest_pose=None;self.history.clear();self.path.clear()
        self.badge.config(text='演示模式 · 模拟数据' if self.demo else '离线 · 未连接',fg='#ffcf7b')
        self.demo_button.config(text='退出演示' if self.demo else '界面演示');self.execute.config(text='演示执行' if self.demo else '执行动作')
        self.log('演示模式：不连接硬件、不发送命令' if self.demo else '已退出演示，可选择控制连接')

    def run_motion(self):
        try:
            tab=self.tabs.index(self.tabs.select());kind='定角' if tab==0 else '位置' if tab==1 else '任意角度'
            spec=motion_spec(kind,float(self.dx.get()),float(self.dy.get()),
                             float(self.position_angle.get() if tab==1 else 0 if tab==2 else self.angle.get()),int(self.limit.get()),4,float(self.duration.get()),
                             dict(motor=0,encoder=0,kp=int(self.kp.get()),ki=int(self.ki.get())),self.profile,
                             distance_mm=float(self.distance.get()),bearing_deg=float(self.bearing.get()))
            if self.demo:self.demo_paused=False;self.log('模拟执行：'+kind+' · 未发送硬件指令');return
            if not self.link:self.log('未连接设备，不能执行');return
            self.link.requests.put(('start',spec))
        except (ValueError,tk.TclError,KeyError) as exc:messagebox.showerror('参数错误',str(exc),parent=self.root)

    def stop(self):
        if self.demo:self.demo_paused=True;self.log('演示停止 · 没有硬件连接');return
        if self.link:self.link.requests.put(('stop',None))
        else:self.log('未连接设备')

    def log(self,text):
        self.log_text.config(state='normal');self.log_text.insert('end',time.strftime('%H:%M:%S')+'  '+text+'\n');self.log_text.see('end');self.log_text.config(state='disabled')
        self.status.set(text)

    def poll(self):
        while True:
            try:kind,value=self.events.get_nowait()
            except queue.Empty:break
            if kind=='frame':self.consume(value)
            elif kind=='log':self.log(value)
            elif kind=='running':self.running=value
            elif kind=='connected':self.connected=True;self.badge.config(text='已连接 · '+value,fg='#84dbbd');self.connect_button.config(text='断开设备');self.log('已连接，仅监测；实时数据自动导出到 runs/live_status.json')
            elif kind=='disconnected':self.connected=False;self.link=None;self.running=False;self.connect_button.config(text='连接设备');self.badge.config(text='离线 · 未连接',fg='#ffcf7b')
        if self.demo:
            t=time.monotonic()-self.start
            velocity=(0,0,0,0) if self.demo_paused else tuple(round(300+18*math.sin(t*1.7+i*.5)) for i in range(4))
            self.demo_positions=[p+v*10 for p,v in zip(self.demo_positions,velocity)]
            self.consume(dict(kind='motor',velocity=velocity,position=self.demo_positions,pwm=(0,0,0,0) if self.demo_paused else (155,160,157,159),armed=False,faults=0))
            if not self.demo_paused:self.consume(dict(kind='pose',healthy=True,calibrated=True,x_counts=(95+85*math.sin(t*.15))*1/self.mm_per_count,
                              y_counts=(45+35*math.sin(t*.3))*1/self.mm_per_count,heading_deg=5*math.sin(t*.3),arrived=False,
                              yaw_deg=5*math.sin(t*.3),gz=round(131*1.5*math.cos(t*.3)),bias=0,who=0x70,imu_error=0,sample_us=int(t*1e6)))
        self.export_status();self.draw();self.root.after(100,self.poll)

    def export_status(self,force=False):
        settings={}
        for name in ['port','bearing','distance','dx','dy','position_angle','angle','duration','limit','kp','ki']:
            try:settings[name]=getattr(self,name).get()
            except (ValueError,tk.TclError):settings[name]=None
        settings['tab']=self.tabs.tab(self.tabs.select(),'text')
        now=time.monotonic()
        status=dict(schema_version=1,connected=self.connected,demo=self.demo,running=self.running,
            motor=self.latest_motor,pose=self.latest_pose,settings=settings,message=self.status.get(),
            motor_age_ms=None if self.motor_received is None else round((now-self.motor_received)*1000),
            pose_age_ms=None if self.pose_received is None else round((now-self.pose_received)*1000))
        try:self.live_status.write(status,force=force)
        except OSError as exc:
            if not self.export_error_reported:self.export_error_reported=True;self.log('实时数据导出失败：'+str(exc))

    def consume(self,row):
        if row['kind']=='motor':
            self.motor_received=time.monotonic()
            self.latest_motor=row
            for i in range(4):
                self.table.set(str(i),'rpm',f"{row['velocity'][i]*6000/self.profile['raw_counts_per_rev']:.1f}");self.table.set(str(i),'pwm',f"{row['pwm'][i]/10:+.1f}%")
            self.history.append(tuple(v*100*math.pi*self.profile['wheel_diameter_mm']/self.profile['raw_counts_per_rev'] for v in row['velocity']))
            self.stat_values['fault'].config(text='模拟反馈' if self.demo else drive_status(row),fg='#b4424b' if row['faults'] & ~1 else self.INK)
        else:
            self.pose_received=time.monotonic()
            self.latest_pose=row;x=row['x_counts']*self.mm_per_count;y=row['y_counts']*self.mm_per_count
            self.path.append((x,y));self.stat_values['position'].config(text=f'{x:.0f}, {y:.0f} mm')
            readings=imu_readout(row);self.stat_values['heading'].config(text=readings['yaw'])
            self.imu_hint.config(text='俯视：逆时针角速度为正、航向增大。\n'+('演示数据' if self.demo else imu_diagnostic(row)))
            sensor={0x68:'6050',0x70:'6500'}.get(row.get('who'),'IMU')
            holding=row.get('imu_stale') or row.get('imu_fault') or row.get('imu_error',0) in (5,6)
            self.stat_values['imu'].config(text='模拟已就绪' if self.demo else sensor+' 数据保持' if row['healthy'] and holding else sensor+' 已就绪' if row['healthy'] else sensor+' 重连 / 校准中',fg=self.BLUE if row['healthy'] else self.MUTED)

    def draw(self):
        c=self.map;c.delete('all');w=max(c.winfo_width(),200);h=max(c.winfo_height(),20)
        xs=[0]+[p[0] for p in self.path];ys=[0]+[p[1] for p in self.path]
        scale=max(.02,min((w-130)/max(200,max(ys)-min(ys)),(h-100)/max(200,max(xs)-min(xs))))
        cx=w/2+(min(ys)+max(ys))/2*scale;cy=h/2+(min(xs)+max(xs))/2*scale
        for x in range(25,int(w),35):c.create_line(x,0,x,h,fill='#edf1f7')
        for y in range(15,int(h),35):c.create_line(0,y,w,y,fill='#edf1f7')
        c.create_line(cx,25,cx,h-20,fill='#c8d4e4',dash=(4,4));c.create_line(20,cy,w-20,cy,fill='#c8d4e4',dash=(4,4))
        c.create_text(cx+8,14,text='+X / mm',anchor='w',fill=self.MUTED,font=('Segoe UI',9));c.create_text(25,cy-10,text='+Y',fill=self.MUTED,font=('Segoe UI',9))
        points=[]
        for x,y in self.path:points.extend((cx-y*scale,cy-x*scale))
        if len(points)>=4:c.create_line(*points,fill=self.BLUE,width=2,smooth=True)
        c.create_oval(cx-4,cy-4,cx+4,cy+4,fill='#879ab3',outline='')
        if self.latest_pose:
            p=self.latest_pose;px=cx-p['y_counts']*self.mm_per_count*scale;py=cy-p['x_counts']*self.mm_per_count*scale;theta=-math.radians(p.get('yaw_deg',p['heading_deg']))
        else:px=cx;py=cy;theta=0
        glyph_scale=max(.15,min(1,(h-24)/110))
        def point(x,y):return (px+(x*math.cos(theta)-y*math.sin(theta))*glyph_scale,py+(x*math.sin(theta)+y*math.cos(theta))*glyph_scale)
        def shape(coords,fill,outline=''):
            values=[]
            for x,y in coords:values.extend(point(x,y))
            c.create_polygon(*values,fill=fill,outline=outline,width=2)
        shape([(-23,-37),(23,-37),(23,37),(-23,37)],'#dce8fc',self.BLUE)
        for x,y in [(-30,-25),(30,-25),(-30,25),(30,25)]:shape([(x-7,y-14),(x+7,y-14),(x+7,y+14),(x-7,y+14)],self.INK)
        x1,y1=point(0,0);x2,y2=point(0,-51);c.create_line(x1,y1,x2,y2,fill=self.BLUE,width=3,arrow='last')
        c.create_text(w-14,h-16,text='模拟轨迹' if self.demo else 'IMU + 轮式里程计',anchor='e',fill=self.MUTED,font=('Microsoft YaHei UI',9))
        c=self.chart;c.delete('all');w=max(c.winfo_width(),200);h=max(c.winfo_height(),50);baseline=h*.64;amplitude=h*.25
        c.create_text(3,10,text='四轮速度 / mm·s⁻¹',anchor='w',fill=self.MUTED,font=('Microsoft YaHei UI',9))
        for i,name in enumerate(['FL','FR','RL','RR']):c.create_text(w-155+i*42,10,text=name,fill=self.COLORS[i],font=('Consolas',10,'bold'))
        values=list(self.history);maximum=max(160,max((abs(v) for row in values for v in row),default=1)*1.2)
        for yy in [baseline-amplitude,baseline,baseline+amplitude]:c.create_line(0,yy,w,yy,fill='#d7e1ed' if yy==baseline else '#edf1f6')
        for i,color in enumerate(self.COLORS):
            points=[]
            for j,row in enumerate(values):points.extend((j*w/159,baseline-row[i]/maximum*amplitude))
            if len(points)>=4:c.create_line(*points,fill=color,width=2,smooth=True)

    def close(self):
        if self.link and self.link.is_alive():
            self.link.close_requested.set();self.status.set('正在关闭连接并停止本次动作…');self.root.after(50,self.close)
        else:self.connected=False;self.running=False;self.export_status(force=True);self.root.destroy()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--demo',action='store_true');parser.add_argument('--connect',action='store_true',help='Connect for read-only monitoring on startup');parser.add_argument('--transport',choices=['router','serial'],default='router')
    parser.add_argument('--telemetry-bind',default='0.0.0.0');parser.add_argument('--fpga-ip',default='192.168.50.30')
    args=parser.parse_args();COMMAND_TRANSPORT=args.transport;TELEMETRY_OPTIONS.update(bind=args.telemetry_bind,peer=args.fpga_ip)
    root=tk.Tk();app=App(root,demo=args.demo)
    if args.connect and not args.demo:root.after(100,app.connect)
    root.mainloop()
