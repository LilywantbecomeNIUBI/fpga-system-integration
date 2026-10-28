"""Explicit serial motor console; default action only monitors telemetry."""
import argparse
import json
import time
from motor_protocol import command,StatusStream

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',required=True)
    parser.add_argument('--action',choices=('monitor','stop','run'),default='monitor')
    parser.add_argument('--left',type=int,default=0);parser.add_argument('--right',type=int,default=0)
    parser.add_argument('--closed-loop',action='store_true')
    parser.add_argument('--motor-reverse',type=int,choices=range(4),default=0,help='bit0 left, bit1 right')
    parser.add_argument('--encoder-reverse',type=int,choices=range(4),default=0,help='bit0 left, bit1 right')
    parser.add_argument('--kp',type=int,default=256);parser.add_argument('--ki',type=int,default=0)
    parser.add_argument('--duration',type=float,default=0.5,help='run: 0.1..10 seconds; monitor: seconds')
    args=parser.parse_args()
    flags=int(args.closed_loop)|(args.motor_reverse<<1)|(args.encoder_reverse<<3)
    # Validate before opening a serial device.
    command(2,0,flags,args.left,args.right,args.kp,args.ki)
    if args.action=='run' and not 0.1<=args.duration<=10:parser.error('run duration must be 0.1..10 seconds')
    if args.duration<=0:parser.error('duration must be positive')
    import serial
    stream=StatusStream();sequence=0
    with serial.Serial(args.port,115200,timeout=0.02,write_timeout=0.2) as port:
        def send(op,left=0,right=0):
            nonlocal sequence
            sequence=(sequence+1)&255
            port.write(command(op,sequence,flags,left,right,args.kp,args.ki));port.flush()
        def statuses():
            return stream.feed(port.read(max(1,port.in_waiting)))
        def await_status(predicate,timeout=0.2):
            until=time.monotonic()+timeout
            while time.monotonic()<until:
                for status in statuses():
                    print(json.dumps(status),flush=True)
                    if predicate(status):return status
            raise RuntimeError('Expected FPGA acknowledgement not received')
        if args.action=='stop':
            send(0);await_status(lambda s:not s['armed']);return
        if args.action=='monitor':
            end=time.monotonic()+args.duration
            while time.monotonic()<end:
                for status in statuses():print(json.dumps(status),flush=True)
            return
        # ARM only after the FPGA acknowledges STOP and CLEAR. No automatic retry/rearm.
        try:
            send(0);await_status(lambda s:not s['armed'])
            send(3);await_status(lambda s:not s['armed'] and s['faults']==0)
            send(1);arm_sequence=sequence
            await_status(lambda s:s['armed'] and s['last_sequence']==arm_sequence)
            end=time.monotonic()+args.duration;next_send=0;last_status=time.monotonic()
            while time.monotonic()<end:
                now=time.monotonic()
                if now>=next_send:send(2,args.left,args.right);next_send=now+0.05
                for status in statuses():
                    last_status=time.monotonic();print(json.dumps(status),flush=True)
                    if status['faults'] or not status['armed']:raise RuntimeError('FPGA stopped: '+str(status))
                if time.monotonic()-last_status>0.2:raise RuntimeError('Telemetry timeout')
        finally:
            send(0)
            await_status(lambda s:not s['armed'])

if __name__=='__main__':main()
