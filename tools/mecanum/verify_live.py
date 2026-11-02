"""Capture raw/Mecanum board status on PC, without arming or motor commands."""
from pathlib import Path
import argparse,collections,json,socket,sys,time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from shared.streams import MixedStream
from shared.telemetry_udp import TelemetryUDP
from shared.lidar_display import FpgaLidar
from tools.task4.telemetry_endpoint import transact


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=float,default=20)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--configure-pc',action='store_true')
    ap.add_argument('--restore-nano',action='store_true');ap.add_argument('--skip-lidar',action='store_true')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    decoder=TelemetryUDP();mixed=MixedStream();lidar=FpgaLidar()
    counts=collections.Counter();latest={};first={};motors=[];densities=[]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream,(args.output/'datagrams.jsonl').open('w') as log:
        stream.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
        stream.bind(('192.168.50.10',6205));stream.settimeout(.1)
        if args.configure_pc:
            endpoint=transact('192.168.50.10',target='192.168.50.10')
            (args.output/'endpoint.json').write_text(json.dumps(endpoint,indent=2))
        start=time.monotonic()
        while time.monotonic()-start<args.seconds:
            try:data,peer=stream.recvfrom(2048)
            except socket.timeout:continue
            now=time.monotonic();frame=decoder.feed(data,peer,now)
            log.write(json.dumps(dict(elapsed=now-start,peer=peer,hex=data.hex()))+'\n')
            if frame is None:continue
            for kind,row in mixed.feed(frame):
                counts[kind]+=1;latest[kind]=row
                if kind not in first:first[kind]=row
                if kind=='mecanum_motor':motors.append(row)
                before=lidar.scans;lidar.ingest(kind,row,now=now-decoder.frame_age)
                if lidar.scans>before:densities.append(len(lidar.scan['points']))
        if args.restore_nano:transact('192.168.50.10',target='192.168.50.20',video_port=6102)
    pose=latest.get('mecanum_pose',{});diag=latest.get('lidar_diagnostic',{})
    result=dict(counts=dict(counts),latest=latest,first=first,transport=decoder.snapshot(),
        mixed_bad_crc=mixed.bad_crc,complete_scans=lidar.scans,scan_point_counts=sorted(set(densities)),
        raw_missing_packets=lidar.raw.missing_packets,raw_stale_packets=lidar.raw.stale_packets,
        motors_stopped=bool(motors) and all(not r['armed'] and r['pwm']==(0,0,0,0) for r in motors),
        imu_healthy=bool(pose.get('healthy') and pose.get('calibrated') and not pose.get('imu_stale') and pose.get('who')==0x70),
        lidar_receiving=diag.get('packets',0)>first.get('lidar_diagnostic',{}).get('packets',0),
        commands_sent=0,hardware_tested=True,lidar_deferred=args.skip_lidar,
        scope='static_mecanum_imu_transport' if args.skip_lidar else 'static_mecanum_imu_lidar_transport')
    lidar_ok=args.skip_lidar or (result['lidar_receiving'] and result['complete_scans'])
    result['passed']=bool(result['motors_stopped'] and result['imu_healthy'] and lidar_ok and
        decoder.rejected==decoder.missing==mixed.bad_crc==0 and
        lidar.raw.missing_packets==lidar.raw.stale_packets==0)
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
