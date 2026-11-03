"""Teaching frame, route persistence, and remote deadman safety regressions."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import struct
import unittest
from contextlib import ExitStack
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'host/mecanum'))
from teaching import Teaching
from remote import RemoteLease, RemoteSession
from console import motor_profile
from gui import MotionSession, motion_spec, SerialLink
from host.motor_protocol import crc16

spec = importlib.util.spec_from_file_location('teach_web', ROOT/'host/mecanum/web.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)


def calibration():
    return dict(confirmed=True, front_mm=150, back_mm=150, half_width_mm=120,
                radius_mm=200, pad_mm=100, forward_mm_per_raw_count=1/256,
                lateral_mm_per_raw_count=1/256, rotation_lever_mm=200,
                laser_x_mm=0, laser_y_mm=0, laser_yaw_deg=0, max_speed_mm_s=50,
                accel_mm_s2=100, measured_stop_mm=40, deceleration_mm_s2=200)


def motor(**values):
    return dict(dict(armed=False, velocity=[0]*4, pwm=[0]*4, position=[0]*4,
                     faults=0, seq=0, control_shift=6, capabilities=3), **values)


def pose(**values):
    return dict(dict(healthy=True, calibrated=True, imu_stale=False, yaw_deg=20,
                     x_counts=0, y_counts=0, heading_deg=0, faults=0, seq=0,
                     mode=0, arrived=False), **values)


class TeachingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.t = Teaching(motor_profile(), self.temp.name)
        self.t.configure(calibration())
        self.m, self.p, self.now = motor(), pose(), 10.
        self.t.observe(self.m, self.p, 9.6, 9.6)
        self.t.observe(self.m, self.p, 9.8, 9.8)
        self.t.observe(self.m, self.p, 10, 10)
        self.t.set_origin((10,self.m),(10,self.p),10)

    def move(self, delta, yaw=None):
        self.now += .05
        self.m = motor(position=[a+b for a,b in zip(self.m['position'],delta)])
        if yaw is not None:self.p=pose(yaw_deg=yaw)
        self.t.observe(self.m,self.p,self.now,self.now)

    def still(self):
        for _ in range(8):self.move([0]*4)

    def record(self):
        self.still()
        self.t.record((self.now,self.m),(self.now,self.p),self.now)

    def test_independent_coordinates_survive_manual_goal_position_resets(self):
        for _ in range(4):self.move([10000]*4)
        self.p=pose(x_counts=0,y_counts=0,heading_deg=0)  # P4 reset on new goal.
        for _ in range(4):self.move([10000]*4)
        self.assertAlmostEqual(self.t.x,312.5)
        self.assertAlmostEqual(self.t.y,0)
        self.record()
        self.assertEqual(self.t.points,[[312,0,30]])

    def test_lateral_and_yaw_rotation_match_start_frame(self):
        for _ in range(4):self.move([-10000,10000,10000,-10000])
        self.assertAlmostEqual(self.t.y,156.25)
        self.move([10000]*4,yaw=110)
        self.assertAlmostEqual(self.t.x,0,places=6)
        self.assertAlmostEqual(self.t.y,195.3125)

    def test_signed_counter_wrap_and_nonzero_initial_positions(self):
        self.m=motor(position=[2147483640]*4)
        self.t.set_origin((self.now,self.m),(self.now,self.p),self.now)
        self.now+=.05;self.m=motor(position=[-2147483640]*4)
        self.t.observe(self.m,self.p,self.now,self.now)
        self.assertAlmostEqual(self.t.x,16/256)

    def test_gap_reset_and_stale_imu_invalidate_without_guessing(self):
        self.now+=.4
        self.t.observe(motor(position=[1000]*4),self.p,self.now,self.now)
        self.assertFalse(self.t.valid)
        self.assertEqual(self.t.x,0)
        self.assertFalse(self.t.replay_ready)

    def test_encoder_jump_invalidates(self):
        self.move([100000]*4)
        self.assertFalse(self.t.valid)

    def test_record_requires_continuous_stopped_feedback_and_spacing(self):
        with self.assertRaises(ValueError):self.record()
        for _ in range(4):self.move([10000]*4)
        self.now+=.05;self.m=motor(position=self.m['position'],armed=True)
        self.t.observe(self.m,self.p,self.now,self.now)
        with self.assertRaises(ValueError):self.t.record((self.now,self.m),(self.now,self.p),self.now)
        self.m=motor(position=self.m['position'])
        self.record()
        self.assertEqual(len(self.t.points),1)

    def test_save_reload_export_roundtrip_and_path_traversal(self):
        for _ in range(4):self.move([10000]*4)
        self.record();route_id=self.t.save('L形通道')
        self.assertEqual(self.t.routes()[0]['name'],'L形通道')
        saved=self.t.route()
        other=Teaching(motor_profile(),self.temp.name);other.load_id(route_id)
        self.assertEqual(other.route(),saved)
        self.assertFalse(other.valid)
        self.assertFalse(other.replay_ready)
        for name in ('../secret','a'*33,'not-a-route'):
            with self.assertRaises(ValueError):other.load_id(name)

    def test_replay_origin_keeps_points_and_any_movement_revokes_ready(self):
        for _ in range(4):self.move([10000]*4)
        self.record();points=copy.deepcopy(self.t.points)
        self.t.set_origin((self.now,self.m),(self.now,self.p),self.now,replay=True)
        self.assertTrue(self.t.replay_ready)
        self.assertEqual(self.t.points,points)
        self.move([10000]*4)
        self.assertFalse(self.t.replay_ready)

    def test_cannot_reinterpret_nominal_or_old_calibrated_points(self):
        for _ in range(4):self.move([10000]*4)
        self.record()
        with self.assertRaises(ValueError):self.t.configure(dict(calibration(),lateral_mm_per_raw_count=1/128))
        self.assertEqual(self.t.scales,[1/256]*2)

    def test_malformed_import_is_transactional(self):
        for _ in range(4):self.move([10000]*4)
        self.record();before=self.t.route()
        for changes in ({'name':{}},{'points':[[float('nan'),0,30]]},{'points':[[0,0,100]]},{'calibration':{'confirmed':False}}):
            with self.assertRaises(ValueError):self.t.load(dict(before,**changes))
            self.assertEqual(self.t.route(),before)


class RemoteTests(unittest.TestCase):
    def test_late_press_cannot_overtake_release(self):
        lease=RemoteLease();lease.end(2)
        with self.assertRaises(ValueError):lease.begin(2,[1,0,0],2,10)
        self.assertIsNone(lease.snapshot(10))
        lease.begin(3,[1,0,0],2,10)
        lease.end(2)
        self.assertEqual(lease.snapshot(10)['gesture'],3)

    def test_expired_pulse_never_resurrects_same_gesture(self):
        lease=RemoteLease();lease.begin(1,[1,0,0],2,10)
        with self.assertRaises(ValueError):lease.pulse(1,10.31)
        with self.assertRaises(ValueError):lease.begin(1,[1,0,0],2,10.32)
        lease.begin(2,[1,0,0],2,10.32)
        self.assertIsNotNone(lease.snapshot(10.33))

    def test_worker_checks_hold_deadline_before_each_handshake_step(self):
        lease=RemoteLease();lease.begin(1,[1,0,0],2,10)
        calls=[];seq=0
        def send(op,values,options):
            nonlocal seq
            seq+=1;calls.append(op);return seq
        session=RemoteSession(lease.snapshot(10),lease,send,motor_profile(),10,
                              (10,motor()),(10,pose()),MotionSession,motion_spec)
        self.assertEqual(calls,[0])
        session.step(10.05,(10.05,motor()),(10.05,pose()))
        self.assertEqual(calls,[0,3])
        self.assertIsNotNone(session.step(10.31,(10.31,motor()),(10.31,pose())))
        self.assertNotIn(1,calls)

    def test_released_active_goal_cannot_send_another_heartbeat(self):
        lease=RemoteLease();lease.begin(1,[1,0,0],2,10)
        calls=[];seq=0
        def send(op,values,options):
            nonlocal seq
            seq+=1;calls.append(op);return seq
        session=RemoteSession(lease.snapshot(10),lease,send,motor_profile(),10,
                              (10,motor()),(10,pose()),MotionSession,motion_spec)
        for now,armed in [(10.05,False),(10.10,False),(10.15,True),(10.20,True)]:
            session.step(now,(now,motor(armed=armed,seq=seq)),(now,pose()))
        lease.pulse(1,10.25)
        session.step(10.25,(10.25,motor(armed=True,seq=seq)),(10.25,pose()))
        self.assertEqual(calls,[0,3,1,6,7])
        lease.end(1);before=list(calls)
        self.assertIsNotNone(session.step(10.27,(10.27,motor(armed=True,seq=seq)),(10.27,pose())))
        self.assertEqual(calls,before)

    def test_diagonal_remote_limit_and_no_unsupported_velocity_opcode(self):
        lease=RemoteLease();lease.begin(1,[1,1,0],2,10)
        calls=[]
        session=RemoteSession(lease.snapshot(10),lease,lambda *a:calls.append(a),motor_profile(),10,
                              (10,motor()),(10,pose()),MotionSession,motion_spec)
        self.assertEqual(session.inner.spec['op'],6)
        self.assertEqual(session.inner.spec['options']['limit'],1)
        self.assertEqual(len(calls),1)  # Constructor sends STOP only.

    def test_real_command_worker_stops_on_release_and_expiry_without_http_loop(self):
        class Device:
            def __init__(self):
                self.records=[];self.armed=False;self.seq=0;self.index=0
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def write(self,data):
                op=data[3];self.records.append((time.monotonic(),op))
                if op==0:self.armed=False
                if op==1:self.armed=True
                if op in (1,6,7,9):self.seq=data[4]
                return len(data)
            def read(self,size):
                time.sleep(.005);self.index+=1;b=bytearray(46)
                if self.index%2:
                    b[:3]=b'M4\x03';b[3]=self.armed;b[6]=self.seq;b[44]=6;b[45]=3
                else:
                    b[:3]=b'P4\x03';b[3]=3;b[6]=self.seq;b[32]=6
                    struct.pack_into('<i',b,20,round(math.radians(20)*65536))
                return bytes(b)+struct.pack('<H',crc16(b))
        device=Device();lease=RemoteLease();link=SerialLink(None,queue.Queue(),motor_profile());link.remote_lease=lease
        def wait(predicate,seconds=1):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                if predicate():return
                time.sleep(.005)
            self.fail('Simulated command worker did not reach expected state: '+str(device.records))
        with ExitStack() as stack:
            folder=stack.enter_context(tempfile.TemporaryDirectory())
            from live_status import TelemetryJournal
            stack.enter_context(patch('gui.TelemetryJournal',side_effect=lambda _:TelemetryJournal(Path(folder))))
            stack.enter_context(patch('transport.RouterLink',return_value=device))
            link.start()
            try:
                wait(lambda:link.motor is not None and link.pose is not None)
                lease.begin(1,[1,0,0],2,time.monotonic())
                wait(lambda:any(op==6 for _,op in device.records))
                released=time.monotonic();lease.end(1)
                wait(lambda:any(t>=released and op==0 for t,op in device.records))
                time.sleep(.04)
                self.assertTrue(all(op==0 for t,op in device.records if t>=released))
                lease.begin(2,[0,1,0],2,time.monotonic())
                index=len(device.records)
                wait(lambda:any(op==6 for _,op in device.records[index:]))
                # No Console.tick(), browser heartbeat, or release request.
                wait(lambda:device.records[-1][1]==0)
                self.assertIsNone(lease.snapshot(time.monotonic()))
                self.assertFalse(device.armed)
            finally:
                link.close_requested.set();link.join(1)
        self.assertFalse(link.is_alive())


class Link:
    def __init__(self):self.requests=queue.Queue();self.remote_lease=None
    def is_alive(self):return True


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.now=10.
        self.c=web.Console(clock=lambda:self.now,routes_dir=self.temp.name)
        self.c.link=Link();self.c.connected=True;self.client='a'*32
        self.c.frames={'motor':(self.now,motor()),'pose':(self.now,pose()),
                       'navigation':(self.now,dict(active=False,calibrated=False,mission_id=0,ack_error=0,status=0))}
        self.epoch=self.action('acquire')['control_epoch']

    def action(self,action,**values):
        return self.c.action(dict(action=action,client=self.client,**values))

    def test_release_keeps_owner_and_normal_completion_keeps_owner(self):
        self.action('jog_begin',gesture=1,control_epoch=self.epoch,direction=[1,0,0])
        self.action('jog_end',gesture=1,control_epoch=self.epoch)
        self.c.events.put(('running',False));self.c.tick()
        self.assertTrue(self.c.state(self.client)['operator'])
        self.assertTrue(self.c.link.requests.empty())

    def test_general_heartbeat_cannot_keep_remote_moving(self):
        self.action('jog_begin',gesture=1,control_epoch=self.epoch,direction=[1,0,0])
        self.now+=.31
        with self.assertRaises(ValueError):self.action('heartbeat')
        self.assertIsNone(self.c.owner)

    def test_stale_control_epoch_and_late_begin_after_release_rejected(self):
        self.action('stop');new=self.action('acquire')['control_epoch']
        self.assertNotEqual(new,self.epoch)
        with self.assertRaises(ValueError):self.action('jog_begin',gesture=1,control_epoch=self.epoch,direction=[1,0,0])
        self.action('jog_end',gesture=1,control_epoch=new)
        with self.assertRaises(ValueError):self.action('jog_begin',gesture=1,control_epoch=new,direction=[1,0,0])

    def test_remote_rejects_stale_feedback_active_navigation_and_high_speed(self):
        for changes in ({'limit':3},{'direction':[1,0,1]},{'direction':[True,0,0]}):
            values=dict(gesture=1,control_epoch=self.epoch,direction=[1,0,0]);values.update(changes)
            with self.assertRaises(ValueError):self.action('jog_begin',**values)
        self.c.frames['motor']=(9,motor())
        with self.assertRaises(ValueError):self.action('jog_begin',gesture=1,control_epoch=self.epoch,direction=[1,0,0])

    def test_draft_cannot_upload_and_begin_requires_explicit_origin_confirmation(self):
        self.c.teaching.points=[[200,0,30]]
        with self.assertRaises(ValueError):self.action('teach_origin',confirmed=False)
        with self.assertRaises(ValueError):self.action('teach_upload',mission_id=1)

    def test_teach_upload_and_start_require_matching_route_ack_and_fixed_origin(self):
        t=self.c.teaching;t.configure(calibration());t.points=[[200,0,30]];t.still_since=9.5
        self.action('teach_replay_origin',confirmed=True)
        with self.assertRaises(ValueError):self.action('teach_start',mission_id=7)
        self.action('teach_upload',mission_id=7)
        _,queued=self.c.link.requests.get_nowait()
        self.assertEqual((queued['action'],queued['points']),('upload',[[200,0,30]]))
        self.assertEqual(self.c.mode,'navigation')
        self.c.pending=False
        with self.assertRaises(ValueError):self.action('teach_start',mission_id=7)
        self.c.frames['navigation']=(10,dict(active=False,calibrated=True,status=2,mission_id=7,ack_error=0))
        self.action('teach_start',mission_id=7)
        self.assertEqual(self.c.link.requests.get_nowait()[1]['action'],'start')
        self.assertFalse(t.replay_ready)

    def test_nominal_route_still_cannot_upload_after_return_origin_confirmed(self):
        t=self.c.teaching;t.points=[[200,0,30]];t.still_since=9.5
        self.action('teach_replay_origin',confirmed=True)
        with self.assertRaises(ValueError):self.action('teach_upload',mission_id=1)
        self.assertTrue(self.c.link.requests.empty())

    def test_source_receive_time_is_used_and_stale_queued_frames_are_rejected(self):
        self.c.events.put(('frame',dict(motor(),kind='motor',_received_monotonic=9)))
        self.c.tick()
        self.assertEqual(self.c.frames['motor'][0],10)
        self.assertFalse(self.c.teaching.valid)

    def test_action_fault_revokes_owner_ordinary_completion_does_not(self):
        self.c.events.put(('running',False));self.c.tick()
        self.assertEqual(self.c.owner,self.client)
        self.c.events.put(('fault','IMU过期'));self.c.tick()
        self.assertIsNone(self.c.owner)


if __name__=='__main__':unittest.main()
