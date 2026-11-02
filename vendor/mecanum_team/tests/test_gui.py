"""Safety/state tests for the GUI's transport logic, without Tk or serial I/O."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from gui import MotionSession,motion_spec,imu_readout,CalibrationSession,drive_status,imu_diagnostic,App
from types import SimpleNamespace
from unittest.mock import Mock
from console import motor_profile


class GUITransportTests(unittest.TestCase):
    def test_calibration_stop_ack_invalid_then_ready_no_arm(self):
        sent=[]
        def send(op,values,options):sent.append(op);return len(sent)
        motor=dict(armed=True,seq=20,faults=1)
        pose=dict(healthy=True,calibrated=True,imu_fault=False)
        session=CalibrationSession(send,0,(0,motor),{})
        self.assertEqual(sent,[0])
        session.step(.01,(.01,motor),(.01,pose));self.assertEqual(sent,[0])
        motor['armed']=False
        session.step(.02,(.02,motor),(.02,pose));self.assertEqual(sent,[0,8])
        motor['seq']=2
        self.assertIsNone(session.step(.03,(.03,motor),(.03,pose)))
        self.assertIsNone(session.step(.04,(.04,motor),(.04,pose)))
        pose.update(healthy=False,calibrated=False)
        session.step(.05,(.05,motor),(.05,pose))
        pose.update(healthy=True,calibrated=True)
        self.assertIn('校零完成',session.step(.1,(.1,motor),(.1,pose)))
        self.assertNotIn(1,sent);self.assertNotIn(2,sent)
        self.assertEqual(drive_status(motor),'上电锁定 · 待使能')
        self.assertEqual(drive_status(dict(armed=False,faults=0x11)),'故障 0x0011')

    def test_calibration_old_firmware_times_out(self):
        m=dict(armed=False,seq=0)
        session=CalibrationSession(lambda *args:7,0,(0,m),{})
        session.step(.01,(.01,m),None)
        with self.assertRaisesRegex(RuntimeError,'超时'):session.step(.32,(.32,m),None)

    def test_reconnection_allows_old_fault_then_requires_fresh_calibration(self):
        m=dict(armed=False,seq=0);p=dict(healthy=False,calibrated=False,imu_fault=True)
        s=CalibrationSession(lambda *args:7,0,(0,m),{})
        s.step(.01,(.01,m),(.01,p));m['seq']=7
        self.assertIsNone(s.step(.02,(.02,m),(.02,p)))
        p.update(imu_fault=False)
        s.step(.5,(.5,m),(.5,p))
        p.update(healthy=True,calibrated=True)
        self.assertIn('通信恢复',s.step(3,(3,m),(3,p)))
        bad=imu_readout(dict(imu_fault=True,gz=100,yaw_deg=90))
        self.assertEqual(bad['raw'],'+100');self.assertEqual(bad['yaw'],'+90.00°')
        self.assertIn('NACK',imu_diagnostic(dict(imu_diag_version=1,imu_bus_error=1,imu_bus_lines=3,imu_fault_count=2,imu_fault_pwm=True)))

    def test_imu_actual_yaw_signed_rate_and_identity(self):
        values=imu_readout(dict(who=0x70,imu_error=0,gz=1292,bias=-18,sample_us=1234567,yaw_deg=90,heading_deg=0))
        self.assertEqual(values,dict(identity='0x70 / 0',raw='+1292',bias='-18',rate='+10.000',sample='1234.6',yaw='+90.00°'))
        self.assertEqual(imu_readout(dict(gz=-1328,bias=-18))['rate'],'-10.000')

    def spec(self,kind='位置'):
        return motion_spec(kind,200,0,90,20,4,15,dict(motor=0,encoder=0,kp=512,ki=32),motor_profile())

    def test_spec_matches_gmr_units(self):
        spec=self.spec()
        self.assertEqual(spec['op'],6);self.assertEqual(spec['values'],(3183,0,9000))
        self.assertEqual(self.spec('定角')['values'],(0,0,9000))

    def test_arbitrary_direction_targets(self):
        args=('任意角度',0,0,90,20,4,15,dict(motor=0,encoder=0,kp=512,ki=32),motor_profile())
        for bearing,expected in [(0,(3183,0,0)),(90,(0,3183,0)),(-90,(0,-3183,0)),(180,(-3183,0,0)),(45,(2251,2251,0)),(-135,(-2251,-2251,0))]:
            spec=motion_spec(*args,distance_mm=200,bearing_deg=bearing)
            self.assertEqual((spec['op'],spec['values']),(6,expected))
        for distance,bearing in [(0,0),(-1,45),(1001,0),(200,181),(float('nan'),0),(200,float('inf'))]:
            with self.assertRaises(ValueError):motion_spec(*args,distance_mm=distance,bearing_deg=bearing)

    def test_no_arm_without_fresh_calibrated_feedback(self):
        sends=[];send=lambda *a:sends.append(a)
        motor=(0,dict(control_shift=4,armed=False,faults=0,seq=0))
        for now,pose in [(.4,(.4,dict(healthy=True))),(.1,None),(.1,(.1,dict(healthy=False)))]:
            with self.assertRaises(ValueError):MotionSession(send,self.spec(),now,motor,pose,4)
        self.assertEqual(sends,[])

    def test_removed_manual_and_unknown_modes_rejected(self):
        for kind in ['遥控','单轮','保持朝向','unknown']:
            with self.assertRaises(ValueError):self.spec(kind)

    def test_handshake_heartbeat_arrival(self):
        sends=[]
        def send(op,values,options):sends.append((op,values));return len(sends)
        motor=dict(control_shift=4,armed=False,faults=1,seq=0)
        pose=dict(healthy=True,faults=0,mode=0,arrived=False)
        s=MotionSession(send,self.spec(),0,(0,motor),(0,pose),4)
        s.step(.01,(.01,motor),(.01,pose));self.assertEqual(sends[-1][0],3)
        motor['faults']=0;s.step(.02,(.02,motor),(.02,pose));self.assertEqual(sends[-1][0],1)
        motor.update(armed=True,seq=3);s.step(.03,(.03,motor),(.03,pose));self.assertEqual(sends[-1][0],6)
        motor['seq']=4;s.step(.04,(.04,motor),(.04,pose))
        pose['mode']=2;s.step(.05,(.05,motor),(.05,pose));self.assertEqual(sends[-1],(7,(0,0,0)))
        self.assertEqual(sum(op==6 for op,v in sends),1)
        pose['arrived']=True;self.assertEqual(s.step(.06,(.06,motor),(.06,pose)),'目标已到达')

    def test_pose_loss_and_unreached_timeout(self):
        motor=dict(control_shift=4,armed=True,faults=0,seq=0)
        pose=dict(healthy=True,faults=0,mode=2,arrived=False)
        s=MotionSession(lambda *a:1,self.spec(),0,(0,motor),(0,pose),4)
        s.phase='active';s.end=1;s.next_send=1
        with self.assertRaisesRegex(RuntimeError,'姿态'):s.step(.4,(.4,motor),(0,pose))
        with self.assertRaisesRegex(RuntimeError,'到达超时'):s.step(1,(1,motor),(1,pose))


if __name__=='__main__':unittest.main()
