"""Explicit board control checks: STOP/CLEAR, zero ARM, lease and IMU restart.

Never sends nonzero motion targets. Each failure attempts STOP in finally.
"""
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
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--configure-pc',action='store_true');args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    decoder=TelemetryUDP();mixed=MixedStream();latest={};events=[];seq=0;checks={}
    def note(kind,**data):events.append(dict(time=time.monotonic(),kind=kind,**data))
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as udp:
        udp.bind(('192.168.50.10',6205));udp.settimeout(.03)
        if args.configure_pc:note('endpoint',data=transact('192.168.50.10',target='192.168.50.10'))
        def wait(predicate,seconds=1):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                try:data,peer=udp.recvfrom(2048)
                except socket.timeout:continue
                raw=decoder.feed(data,peer)
                if decoder.restart_required:raise RuntimeError('FPGA reset; reconnect before further commands')
                if raw is None:continue
                for kind,row in mixed.feed(raw):
                    latest[kind]=row
                    if kind.startswith('mecanum_'):note('status',packet_kind=kind,data=row)
                    if predicate(kind,row):return row
            raise RuntimeError('Fresh telemetry/state acknowledgment timeout')
        motor=wait(lambda k,r:k=='mecanum_motor')
        assert not motor['armed'] and motor['pwm']==(0,0,0,0)
        seq=motor['seq']
        with serial.Serial(args.port,115200,timeout=.02,write_timeout=.2) as uart:
            def send(op):
                nonlocal seq
                seq=(seq+1)&255
                data=packet(op,seq,kp=512,ki=32)
                if uart.write(data)!=len(data):raise RuntimeError('Short serial write')
                note('command',opcode=op,seq=seq,targets=[0,0,0])
                return seq
            try:
                send(0);wait(lambda k,r:k=='mecanum_motor' and not r['armed'])
                send(3);wait(lambda k,r:k=='mecanum_motor' and not r['armed'] and r['faults']==0)
                checks['serial_clear_ack']=True
                arm=send(1);wait(lambda k,r:k=='mecanum_motor' and r['armed'] and r['seq']==arm and r['faults']==0,.3)
                checks['zero_arm_ack']=True
                send(0);wait(lambda k,r:k=='mecanum_motor' and not r['armed'] and r['pwm']==(0,0,0,0),.3)
                checks['stop_ack']=True
                arm=send(1);wait(lambda k,r:k=='mecanum_motor' and r['armed'] and r['seq']==arm,.3)
                wait(lambda k,r:k=='mecanum_motor' and not r['armed'] and (r['faults']&2)!=0,.6)
                checks['watchdog_stops_zero_arm']=True
                send(0);send(3);wait(lambda k,r:k=='mecanum_motor' and not r['armed'] and r['faults']==0)
                recal=send(8)
                wait(lambda k,r:k=='mecanum_motor' and r['seq']==recal and not r['armed'])
                checks['imu_recalibration_command_ack']=True
                try:
                    wait(lambda k,r:k=='mecanum_pose' and r['healthy'] and r['calibrated'] and not r['imu_stale'],8)
                    checks['imu_recovered']=True
                except RuntimeError:checks['imu_recovered']=False
            except Exception as exc:
                checks['error']=str(exc)
            finally:
                for _ in range(3):
                    try:send(0)
                    except Exception as exc:note('stop_error',error=str(exc))
                    time.sleep(.01)
                try:wait(lambda k,r:k=='mecanum_motor' and not r['armed'] and r['pwm']==(0,0,0,0),.5);checks['final_stop_confirmed']=True
                except Exception as exc:checks['final_stop_confirmed']=False;note('stop_confirmation_error',error=str(exc))
                if args.configure_pc:
                    try:note('endpoint_restored',data=transact('192.168.50.10',target='192.168.50.20',video_port=6102))
                    except Exception as exc:note('endpoint_restore_error',error=str(exc))
    result=dict(checks=checks,latest=latest,port=args.port,nonzero_targets_sent=False,
                zero_pwm_throughout=all(e['data']['pwm']==(0,0,0,0) for e in events if e['kind']=='status' and e['packet_kind']=='mecanum_motor'),
                transport=decoder.snapshot(),events=events,hardware_tested=True)
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='events'},indent=2))


if __name__=='__main__':main()
