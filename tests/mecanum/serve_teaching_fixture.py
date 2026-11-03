"""Browser-only fixture. No network/serial board connection and no motor IO.

Run explicitly on 8876; never used by the production launcher.
"""
import argparse
import importlib.util
from pathlib import Path
import queue
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'tests/mecanum'))
from test_teaching_remote import calibration, motor, pose
spec=importlib.util.spec_from_file_location('fixture_web',ROOT/'host/mecanum/web.py')
web=importlib.util.module_from_spec(spec);spec.loader.exec_module(web)


class FixtureConsole(web.Console):
    def state(self,client=None):
        return dict(super().state(client),simulation=True)


class FakeLink:
    def __init__(self):self.requests=queue.Queue();self.remote_lease=None
    def is_alive(self):return True


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--port',type=int,default=8876);args=ap.parse_args()
    c=FixtureConsole(routes_dir=ROOT/'output/playwright/teaching-fixture-routes')
    c.connected=True;c.link=FakeLink();c.teaching.configure(calibration())
    c.nano_state=dict(online=False,telemetry_fresh=False)
    m,p=motor(),pose();n=dict(kind='navigation',active=False,running=False,permit=False,flow_degraded=False,
        map_fresh=True,mission_id=0,waypoint=0,status=0,ack_sequence=0,ack_error=0,reason=0,calibrated=True,
        x_mm=0,y_mm=0,heading_deg=0,vx_mm_s=0,vy_mm_s=0,map_epoch=1,planner_cycles=100,
        point_overflows=0,unknown_motion=False,search_limited=False,planner_timeout=False)
    def feed():
        while not c.closed.wait(.05):
            try:
                while True:
                    action,s=c.link.requests.get_nowait()
                    if action=='stop':c.events.put(('running',False));m.update(armed=False,pwm=[0]*4,velocity=[0]*4)
                    elif action=='start':
                        if s['kind']=='navigation':
                            if s['action']=='upload':n.update(mission_id=s['mission_id'],status=2)
                            elif s['action']=='mode':n.update(active=False,status=0)
                            elif s['action']=='start':n.update(active=True,running=True,status=3)
                        c.events.put(('running',False))
            except queue.Empty:pass
            intent=c.remote.snapshot(time.monotonic())
            if intent:
                x,y,z=intent['direction'];step=640*intent['limit']/2
                ds=[x-y-z,x+y+z,x+y-z,x-y+z]
                m.update(armed=True,position=[round(a+b*step) for a,b in zip(m['position'],ds)],
                         pwm=[100]*4,velocity=[round(b*step/5) for b in ds])
                p['yaw_deg']+=z*.5
            else:m.update(armed=False,pwm=[0]*4,velocity=[0]*4)
            now=time.monotonic()
            for row in [dict(p,kind='pose'),dict(m,kind='motor'),dict(n)]:
                c.events.put(('frame',dict(row,_received_monotonic=now)))
            c.tick()
    threading.Thread(target=feed,daemon=True).start()
    server=web.make_server(c,args.port)
    print('SIMULATION ONLY: no board IO; http://127.0.0.1:%s'%args.port,flush=True)
    server.serve_forever()


if __name__=='__main__':main()
