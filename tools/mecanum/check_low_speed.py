"""Explicit raised-wheel 1.1 s forward check. Always STOP; never auto-retries."""
from pathlib import Path
import argparse,json,socket,sys,time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
import serial
from host.mecanum.console import packet
from shared.telemetry_udp import TelemetryUDP
from shared.streams import MixedStream
from tools.task4.telemetry_endpoint import transact


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--port',required=True)
    ap.add_argument('--wheels-raised',action='store_true',required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    udp=TelemetryUDP();mixed=MixedStream();latest={};received={};events=[];seq=0;result={}
    def record(kind,**values):events.append(dict(time=time.monotonic(),kind=kind,**values))
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream:
        stream.bind(('192.168.50.10',6205));stream.settimeout(.02)
        transact('192.168.50.10',target='192.168.50.10')
        def receive():
            try:data,peer=stream.recvfrom(2048)
            except socket.timeout:return
            frame=udp.feed(data,peer)
            if udp.restart_required:raise RuntimeError('Board reset during test')
            if frame is None:return
            for kind,row in mixed.feed(frame):
                latest[kind]=row;received[kind]=time.monotonic()
                if kind.startswith('mecanum_'):record('status',packet_kind=kind,data=row)
        def wait(kind,predicate,seconds=1):
            end=time.monotonic()+seconds;after=time.monotonic()
            while time.monotonic()<end:
                receive()
                if received.get(kind,0)>=after and predicate(latest[kind]):return latest[kind]
            raise RuntimeError('Fresh '+kind+' acknowledgment timeout')
        with serial.Serial(args.port,115200,timeout=.02,write_timeout=.2) as uart:
            def send(op,x=0):
                nonlocal seq
                seq=(seq+1)&255;raw=packet(op,seq,x=x,kp=512,ki=32,limit=8,tolerance=4)
                assert uart.write(raw)==len(raw)
                record('command',opcode=op,seq=seq,x=x,limit=8)
                return seq
            try:
                motor=wait('mecanum_motor',lambda r:True);seq=motor['seq']
                send(0);wait('mecanum_motor',lambda r:not r['armed'] and r['pwm']==(0,0,0,0))
                # Do not ARM or send a motion goal without a fresh calibrated IMU.
                wait('mecanum_pose',lambda r:r['healthy'] and r['calibrated'] and not r['imu_stale'] and not r['imu_fault'],5)
                send(3);wait('mecanum_motor',lambda r:not r['armed'] and r['faults']==0)
                baseline=latest['mecanum_motor']['position']
                arm=send(1);wait('mecanum_motor',lambda r:r['armed'] and r['seq']==arm and r['faults']==0,.3)
                goal=send(6,512);wait('mecanum_motor',lambda r:r['armed'] and r['seq']==goal and r['faults']==0,.3)
                start=time.monotonic();next_heartbeat=start
                while time.monotonic()-start<1.1:
                    now=time.monotonic()
                    if now>=next_heartbeat:send(7);next_heartbeat=now+.05
                    receive();motor=latest['mecanum_motor'];pose=latest['mecanum_pose']
                    if any(now-received.get(k,0)>.2 for k in ('mecanum_motor','mecanum_pose')):raise RuntimeError('Telemetry stale')
                    if motor['faults'] or not motor['armed'] or not pose['healthy'] or pose['imu_stale'] or pose['imu_fault']:raise RuntimeError('Motion/IMU health failed')
                    if any(abs(v)>512 for v in motor['velocity']):raise RuntimeError('Speed exceeds raised-wheel test bound')
                    if any(p-b<-32 for p,b in zip(motor['position'],baseline)):raise RuntimeError('Unexpected negative encoder direction')
                result['position_delta_raw']=[p-b for p,b in zip(latest['mecanum_motor']['position'],baseline)]
                result['four_feedback_positive']=all(d>0 for d in result['position_delta_raw'])
                result['motion_window_completed']=True
            except Exception as exc:result['error']=str(exc)
            finally:
                for _ in range(3):
                    try:send(0)
                    except Exception as exc:record('stop_error',error=str(exc))
                    time.sleep(.01)
                try:wait('mecanum_motor',lambda r:not r['armed'] and r['pwm']==(0,0,0,0),.5);result['stop_confirmed']=True
                except Exception:result['stop_confirmed']=False
                try:transact('192.168.50.10',target='192.168.50.20',video_port=6102)
                except Exception as exc:result['endpoint_restore_error']=str(exc)
    result.update(latest=latest,events=events,transport=udp.snapshot(),hardware_tested=True)
    result['passed']=bool(result.get('motion_window_completed') and result.get('four_feedback_positive') and result.get('stop_confirmed'))
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='events'},indent=2))


if __name__=='__main__':main()
