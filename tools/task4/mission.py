"""Prepare configuration offline, or explicitly run an already commissioned board.

No serial port is touched by `prepare`. `run` requires complete calibration,
operator-provided motor polarity/gains and exclusive ownership of the UART.
"""
from pathlib import Path
import argparse,json,sys,time
from contextlib import ExitStack
from protocol import configuration_frames,packet,StatusStream
ROOT=Path(__file__).resolve().parents[2]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=['prepare','run']);ap.add_argument('--calibration',required=True);ap.add_argument('--mission',required=True)
    ap.add_argument('--output',default='build/task4/configuration.bin');ap.add_argument('--port');ap.add_argument('--flags',type=int);ap.add_argument('--kp',type=int);ap.add_argument('--ki',type=int);ap.add_argument('--seconds',type=float,default=30);ap.add_argument('--record',type=Path)
    args=ap.parse_args();cal=json.loads(Path(args.calibration).read_text(encoding='utf-8-sig'));mission=json.loads(Path(args.mission).read_text(encoding='utf-8-sig'))
    frames,seq=configuration_frames(cal,mission)
    if args.action=='prepare':
        out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(b''.join(frames));print('Prepared',len(frames),'frames, next sequence',seq,'(no ARM/START)');return
    if not args.port or any(v is None for v in (args.flags,args.kp,args.ki)):ap.error('run requires --port --flags --kp --ki from the validated motor configuration')
    if not 0<args.seconds<=120:ap.error('--seconds must be in (0,120]')
    sys.path.insert(0,str(ROOT/'host'));from motor_protocol import command as motor_command,StatusStream as MotorStream
    import serial
    nav_parser=StatusStream();motor_parser=MotorStream();latest={};seen={};record=None;record_started=time.monotonic()
    def receive(stream):
        data=stream.read(4096);now=time.monotonic()
        if data and record:
            record.write(json.dumps(dict(elapsed_s=now-record_started,hex=data.hex()))+'\n');record.flush()
        for row in nav_parser.feed(data):latest['nav']=row;seen['nav']=now
        for row in motor_parser.feed(data):latest['motor']=row;seen['motor']=now
    def wait_for(stream,predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            receive(stream)
            if predicate():return
        raise RuntimeError('Timed out waiting for board state: '+json.dumps(latest))
    # A separate gateway must relinquish this port before run is invoked.
    with ExitStack() as stack, serial.Serial(args.port,115200,timeout=.02,write_timeout=.1,exclusive=True) as stream:
        def stop():stream.write(motor_command(0,0));stream.flush()
        stop()
        try:
            if args.record:
                args.record.mkdir(parents=True,exist_ok=False);record=stack.enter_context((args.record/'chunks.jsonl').open('w',encoding='utf-8'))
            wait_for(stream,lambda:'nav' in latest and 'motor' in latest and not latest['motor']['armed'],1)
            before=latest['nav']['rejected']
            for frame in frames:stream.write(frame);time.sleep(.005);receive(stream)
            wait_for(stream,lambda:seen.get('nav',0)>time.monotonic()-.15,.5)
            # Wait through one complete status interval after the last write.
            # Extrinsic changes must outlive the previous histogram entries.
            deadline=time.monotonic()+max(.15,cal.get('parameters',{}).get('lidar_timeout_ms',200)/1000+.025)
            while time.monotonic()<deadline:receive(stream)
            if latest['nav']['rejected']!=before:raise RuntimeError('FPGA rejected navigation configuration')
            stream.write(motor_command(3,1,flags=args.flags,kp=args.kp,ki=args.ki));time.sleep(.01)
            stream.write(motor_command(1,2,flags=args.flags,kp=args.kp,ki=args.ki))
            wait_for(stream,lambda:latest.get('motor',{}).get('armed'),.2)
            stream.write(packet(0x42,seq));seq=(seq+1)&255
            started=time.monotonic();next_heartbeat=started;printed=0
            while time.monotonic()-started<args.seconds:
                receive(stream);now=time.monotonic()
                if now>=next_heartbeat:
                    stream.write(packet(0x43,seq));seq=(seq+1)&255;next_heartbeat=now+.05
                if now-seen.get('motor',0)>.2 or now-seen.get('nav',0)>.25:raise RuntimeError('Telemetry lease expired')
                nav=latest.get('nav',{})
                if nav.get('faults'):raise RuntimeError('Navigation fault '+json.dumps(nav))
                if nav.get('done'):print(json.dumps(nav));break
                if now-started>.25 and not nav.get('running'):raise RuntimeError('Mission did not start or lost ownership')
                if now-printed>.5:print(json.dumps(nav),flush=True);printed=now
        finally:stop()
if __name__=='__main__':main()
