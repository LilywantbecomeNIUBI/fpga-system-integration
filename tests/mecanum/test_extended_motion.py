import math,sys,struct,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'host/mecanum'))
from console import motor_profile,packet,mm_to_control_counts,polar_displacement,Stream
from gui import motion_spec,MotionSession

class ExtendedMotion(unittest.TestCase):
    def test_long_moves_fit_real_fixed_point_range(self):
        p=motor_profile()
        self.assertEqual(p['control_shift'],6)
        for distance in (3000,3500,4000):
            for bearing in (0,45,90,135,180,-45,-90,-135):
                x,y=polar_displacement(distance,bearing)
                a,b=[mm_to_control_counts(v,60000,75,6) for v in (x,y)]
                self.assertLessEqual(max(abs(a),abs(b)),16000)
                packet(6,1,x=a,y=b)
        for distance in (0,4001,float('nan')):
            with self.assertRaises(ValueError):polar_displacement(distance,0)

    def test_full_turns_use_explicit_new_opcode(self):
        p=motor_profile();cfg=dict(kp=2048,ki=128)
        for angle in (90,-90,180,-180,360,-360,720,-720):
            spec=motion_spec('定角',0,0,angle,6,4,60,cfg,p)
            self.assertEqual(spec['op'],9);self.assertTrue(spec['outer'])
            raw=packet(spec['op'],1,z=spec['values'][2],**spec['options'])
            self.assertEqual(raw[3],9);self.assertEqual(struct.unpack_from('<h',raw,12)[0],angle*10)
        with self.assertRaises(ValueError):packet(9,1,z=7201)
        with self.assertRaises(ValueError):packet(6,1,z=36000)

    def test_unsupported_firmware_never_sends_arm(self):
        calls=[];p=motor_profile();now=100
        spec=motion_spec('定角',0,0,360,6,4,60,dict(kp=2048,ki=128),p)
        motor=(now,dict(control_shift=6,capabilities=0));pose=(now,dict(healthy=True))
        with self.assertRaises(ValueError):MotionSession(lambda *a:calls.append(a),spec,now,motor,pose,6)
        self.assertEqual(calls,[])

    def test_default_speed_and_gain_conversion_preserve_physics(self):
        p=motor_profile()
        old=20*(1<<4);new=p['default_speed_limit']*(1<<p['control_shift'])
        self.assertAlmostEqual(new/old,1.2)
        self.assertEqual(p['initial_kp_q8'],512*4);self.assertEqual(p['initial_ki_q8'],32*4)

    def test_delayed_previous_goal_completion_does_not_stop_new_turn(self):
        p=motor_profile();now=100;sequence=250
        def send(*args):
            nonlocal sequence
            sequence=(sequence+1)&255
            return sequence
        spec=motion_spec('定角',0,0,360,6,4,60,dict(kp=2048,ki=128),p)
        m=dict(control_shift=6,capabilities=3,armed=False,faults=0,seq=250)
        pose=dict(healthy=True,faults=0,mode=2,arrived=True,seq=250)
        session=MotionSession(send,spec,now,(now,m),(now,pose),6)
        for armed in (False,False,True,True):
            now+=.01;m=dict(m,armed=armed,seq=sequence)
            session.step(now,(now,m),(now,pose))
        self.assertEqual(session.phase,'active')
        # Late P4 from the last goal arrives after the new M4 acknowledgment.
        now+=.01
        self.assertIsNone(session.step(now,(now,m),(now,pose)))
        # A matching fresh snapshot can finish, including sequence wrap.
        now+=.01;m=dict(m,seq=0);pose=dict(pose,seq=0)
        self.assertEqual(session.step(now,(now,m),(now,pose)),'目标已到达')

if __name__=='__main__':unittest.main()
