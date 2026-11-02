"""Read-only UDP board capture. Does not configure the network or open a serial port."""
from pathlib import Path
import argparse
import collections
import json
import socket
import sys
import time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from shared.streams import MixedStream
from shared.lidar_display import FpgaLidar
from shared.telemetry_udp import TelemetryUDP
from tools.task4.protocol import StatusStream


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--bind',default='0.0.0.0')
    parser.add_argument('--board',default='192.168.50.30');parser.add_argument('--port',type=int,default=6205)
    parser.add_argument('--seconds',type=float,default=20);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    udp=TelemetryUDP(args.board,args.port);mixed=MixedStream();nav=StatusStream();lidar=FpgaLidar()
    counts=collections.Counter();motors=[];latest={};first_diag=None;scan_points=[]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream,(args.output/'datagrams.jsonl').open('w',encoding='utf-8') as raw:
        stream.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024);stream.bind((args.bind,args.port));stream.settimeout(.2)
        start=time.monotonic();wire_bytes=0
        print('UDP read-only capture started',flush=True)
        while time.monotonic()-start<args.seconds:
            try:data,peer=stream.recvfrom(2048)
            except socket.timeout:continue
            now=time.monotonic();frame=udp.feed(data,peer,now=now)
            raw.write(json.dumps(dict(elapsed_s=now-start,peer=list(peer),hex=data.hex()))+'\n')
            if frame is None:continue
            wire_bytes+=len(data)
            rows=mixed.feed(frame)+[('nav',r) for r in nav.feed(frame)]
            for kind,row in rows:
                counts[kind]+=1;latest[kind]=row;before=lidar.scans
                lidar.ingest(kind,row,now=now-udp.frame_age)
                if kind=='motor':motors.append(row)
                if kind=='lidar_diagnostic' and first_diag is None:first_diag=dict(row)
                if lidar.scans>before:scan_points.append(len(lidar.scan['points']))
        elapsed=time.monotonic()-start
    diagnostic=latest.get('lidar_diagnostic',{});first_diag=first_diag or {}
    errors={k:((diagnostic.get(k,0)-first_diag.get(k,0))&0xffffffff) for k in ('crc_errors','framing_errors','format_errors','timeouts')}
    raw_mode=diagnostic.get('representation')=='raw_points' or counts['lidar_raw']>0
    lidar_kind='lidar_raw' if raw_mode else 'lidar_scan'
    density_ok=bool(scan_points) and (min(scan_points)>=360 if raw_mode else set(scan_points)=={360})
    transport_passed=bool({'motor','encoder_diagnostics','encoder_trace','imu','nav',lidar_kind,'lidar_diagnostic'}<=set(counts)
        and density_ok and mixed.bad_crc==nav.bad_crc==udp.rejected==udp.missing==0
        and (not raw_mode or (lidar.raw.missing_packets==lidar.raw.stale_packets==0
             and diagnostic.get('dropped_packets',0)==first_diag.get('dropped_packets',0)))
        and all(v==0 for v in errors.values()))
    stopped=bool(motors) and all(not r['armed'] and r['left_applied']==r['right_applied']==0 for r in motors)
    imu=latest.get('imu',{})
    result=dict(passed=transport_passed and stopped,transport_passed=transport_passed,motor_stopped=stopped,
                hardware_tested=True,commands_sent=0,serial_ports_opened=0,elapsed_seconds=elapsed,
                counts=dict(counts),complete_scans=lidar.scans,upload_hz=lidar.scans/elapsed,
                lidar_mode='raw_points' if raw_mode else 'sector360',point_counts=sorted(set(scan_points)),udp_payload_bytes_per_second=wire_bytes/elapsed,
                ethernet_wire_estimate_mbit_per_second=(wire_bytes+udp.accepted*66)*8/elapsed/1e6,
                transport=udp.snapshot(),mixed_bad_crc=mixed.bad_crc,nav_bad_crc=nav.bad_crc,
                lidar_error_deltas=errors,latest_diagnostic=diagnostic,scan=lidar.scan,
                imu_healthy=bool(imu.get('type')=='sample' and imu.get('ready') and not imu.get('fault') and not imu.get('stale')))
    (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='scan'},ensure_ascii=True,indent=2),flush=True)
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()
