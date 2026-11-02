"""Router-only board acceptance; optional explicit raised-wheel 1.1 s motion."""
from pathlib import Path
import argparse,json,sys,time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from host.mecanum.console import packet,Stream
from host.mecanum.transport import RouterLink

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--bind',default='192.168.50.10')
    ap.add_argument('--motion',action='store_true');ap.add_argument('--wheels-raised',action='store_true')
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if args.motion and not args.wheels_raised:ap.error('Motion requires confirmed --wheels-raised')
    args.output.mkdir(parents=True,exist_ok=False)
    latest={};received={};events=[];checks={};seq=0;last_packet=None;decoder=Stream()
    with RouterLink(bind=args.bind,timeout=.015) as link:
        def receive():
            for row in decoder.feed(link.read(256)):
                now=time.monotonic();latest[row['kind']]=row;received[row['kind']]=now
                events.append(dict(time=now,kind='status',data=row))
        def wait(kind,predicate,seconds=1):
            after=time.monotonic();end=after+seconds
            while time.monotonic()<end:
                receive()
                if received.get(kind,0)>after and predicate(latest[kind]):return latest[kind]
            raise RuntimeError('Fresh '+kind+' acknowledgment timeout')
        def send(op,x=0):
            nonlocal seq,last_packet
            seq=(seq+1)&255;last_packet=packet(op,seq,x=x,kp=512,ki=32,limit=8,tolerance=4)
            if link.write(last_packet)!=24:raise RuntimeError('Short UDP write')
            events.append(dict(time=time.monotonic(),kind='command',op=op,seq=seq,x=x,limit=8))
            return seq
        def stopped(r):return not r['armed'] and r['pwm']==(0,0,0,0)
        try:
            m=wait('motor',lambda r:True);seq=m['seq']
            if not stopped(m):raise RuntimeError('Board unexpectedly armed before test')
            send(0);wait('motor',stopped)
            send(3);wait('motor',lambda r:stopped(r) and not r['faults']);checks['udp_clear_ack']=True
            arm=send(1);wait('motor',lambda r:r['armed'] and r['seq']==arm and not r['faults'],.3)
            checks['zero_arm_ack']=True
            send(0);wait('motor',stopped,.3);checks['stop_ack']=True
            arm=send(1);wait('motor',lambda r:r['armed'] and r['seq']==arm,.3)
            old_arm=last_packet
            started=time.monotonic()
            wait('motor',lambda r:stopped(r) and bool(r['faults']&2),.6)
            checks['watchdog_stop_observed_s_after_arm_ack']=time.monotonic()-started
            send(0);wait('motor',stopped);send(3);wait('motor',lambda r:stopped(r) and not r['faults'])
            link.write(old_arm)
            for _ in range(3):wait('motor',stopped,.2)
            checks['old_arm_rejected_after_clear']=True
            if args.motion:
                wait('pose',lambda r:r['healthy'] and r['calibrated'] and not r['imu_stale'] and not r['imu_fault'],5)
                baseline=latest['motor']['position'];initial_imu_errors=latest['pose']['imu_fault_count']
                arm=send(1);wait('motor',lambda r:r['armed'] and r['seq']==arm and not r['faults'],.3)
                goal=send(6,512);wait('motor',lambda r:r['armed'] and r['seq']==goal and not r['faults'],.3)
                start=time.monotonic();next_send=start
                while time.monotonic()-start<1.1:
                    now=time.monotonic()
                    if now>=next_send:send(7);next_send=now+.05
                    receive();now=time.monotonic();m=latest['motor'];p=latest['pose']
                    if any(now-received.get(k,0)>.2 for k in ('motor','pose')):raise RuntimeError('Stale motion feedback')
                    if not m['armed'] or m['faults'] or not p['healthy'] or p['imu_stale'] or p['imu_fault']:raise RuntimeError('Motion/IMU health failed')
                    if any(abs(v)>512 for v in m['velocity']):raise RuntimeError('Speed bound exceeded')
                    if any(v-b<-32 for v,b in zip(m['position'],baseline)):raise RuntimeError('Unexpected negative wheel direction')
                checks['motion_completed']=True
                checks['position_delta_raw']=[v-b for v,b in zip(latest['motor']['position'],baseline)]
                checks['four_feedback_positive']=all(v>0 for v in checks['position_delta_raw'])
                checks['imu_errors_during_motion']=latest['pose']['imu_fault_count']-initial_imu_errors
        except Exception as exc:checks['error']=str(exc)
        finally:
            for _ in range(3):
                try:send(0)
                except Exception as exc:events.append(dict(kind='stop_error',error=str(exc)))
                time.sleep(.01)
            try:wait('motor',stopped,.5);checks['final_stop_confirmed']=True
            except Exception as exc:checks['final_stop_confirmed']=False;checks['stop_error']=str(exc)
            # Observe the wheels settling; do not extend the control lease.
            try:wait('motor',lambda r:stopped(r) and r['velocity']==(0,0,0,0),.7);checks['final_velocity_zero']=True
            except Exception:checks['final_velocity_zero']=False
        motor_rows=[e['data'] for e in events if e['kind']=='status' and e['data']['kind']=='motor']
        checks['max_abs_pwm']=max((abs(v) for r in motor_rows for v in r['pwm']),default=0)
        checks['max_abs_raw_speed']=max((abs(v) for r in motor_rows for v in r['velocity']),default=0)
        result=dict(checks=checks,latest=latest,events=events,transport=link.decoder.snapshot(),
                    command_transport='udp6206',serial_opened=False,hardware_tested=True,
                    motion_requested=args.motion,passed=not checks.get('error') and checks.get('final_stop_confirmed') and (not args.motion or checks.get('four_feedback_positive')))
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='events'},indent=2))
    if not result['passed']:raise SystemExit(1)

if __name__=='__main__':main()
