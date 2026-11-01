"""Read-only M2 reception check. Records UART, never sends device commands.

Live requires exclusive serial ownership. Replay consumes task4 chunks.jsonl.
"""
import argparse,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'.deps'))
from shared.streams import MixedStream
from shared.lidar_display import FpgaLidar


class Check:
    def __init__(self):
        self.parser=MixedStream();self.lidar=FpgaLidar()
        self.first=None;self.last=None;self.motor=None;self.motor_time=None;self.motor_safe=True

    def feed(self,data,now):
        for kind,row in self.parser.feed(data):
            self.lidar.ingest(kind,row,now)
            if kind=='lidar_diagnostic':
                if self.first is None:self.first=dict(row)
                self.last=dict(row)
            elif kind=='motor':
                self.motor,self.motor_time=row,now
                self.motor_safe &= not row['armed'] and row['left_applied']==row['right_applied']==0

    def result(self,now):
        lidar=self.lidar.snapshot(now)
        def advancing(key):
            return bool(self.first and self.last and 0<((self.last[key]-self.first[key])&0xffffffff)<0x80000000)
        reception=advancing('bytes') and advancing('packets')
        display=lidar['online'] and self.lidar.scans>=2
        stopped=bool(self.motor_safe and self.motor and now-self.motor_time<.25)
        clean=self.parser.bad_crc==0
        return dict(passed=bool(reception and display and stopped and clean),
            receive_advancing=reception,complete_scan_advancing=display,
            stopped_window=stopped,mixed_crc_errors=self.parser.bad_crc,lidar=lidar,
            first_diagnostic=self.first,last_diagnostic=self.last,commands_sent=0)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    source=ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--serial');source.add_argument('--replay',type=Path)
    ap.add_argument('--seconds',type=float,default=10);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    if not 1<=args.seconds<=120:ap.error('--seconds must be 1..120')
    args.out.mkdir(parents=True,exist_ok=False);check=Check();now=0
    with (args.out/'rx.bin').open('xb') as raw,(args.out/'chunks.jsonl').open('x',encoding='utf-8') as chunks:
        def ingest(data,elapsed):
            raw.write(data);chunks.write(json.dumps(dict(elapsed_s=elapsed,hex=data.hex()))+'\n');chunks.flush()
            check.feed(data,elapsed)
        if args.replay:
            for line in args.replay.read_text(encoding='utf-8').splitlines():
                item=json.loads(line);now=float(item['elapsed_s']);ingest(bytes.fromhex(item['hex']),now)
        else:
            import serial
            # Read operations only. No CLEAR, ARM, STOP, navigation or heartbeat.
            started=time.monotonic()
            with serial.Serial(args.serial,115200,timeout=.03,exclusive=True) as port:
                while time.monotonic()-started<args.seconds:
                    data=port.read(min(max(port.in_waiting,1),4096));now=time.monotonic()-started
                    if data:ingest(data,now)
    report=check.result(now);report.update(hardware_tested=bool(args.serial),programmed=False,
        source=str(args.serial or args.replay),measurement='host receive times; not device sample timestamps')
    (args.out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2,ensure_ascii=False))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
