"""Read-only mixed-UART IMU acceptance. Never transmits or programs hardware."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'.deps')]
from shared.streams import MixedStream
from shared import imu_protocol

def evaluate(data, min_duration_s=0, require_motor_telemetry=True):
    parser=MixedStream();rows=parser.feed(data)
    frames=[row for kind,row in rows if kind=='imu']
    samples=[row for row in frames if row['type']=='sample']
    motors=[row for kind,row in rows if kind=='motor']
    stats=imu_protocol.Statistics()
    for row in frames:stats.add(row)
    identities=all(row['config_id'] in (4,6) and imu_protocol.identity_matches(row) and not row['fast_bus'] for row in frames)
    healthy=all(row['ready'] and not row['fault'] and not row['stale'] and row['temperature_valid'] and row['last_error']==0 for row in frames)
    counters={name:[row[name] for row in frames] for name in ('error_count','invalid_count','tx_drop_count')}
    clean=all(values and max(values)==0 for values in counters.values())
    stopped=bool(motors) and all(not row['armed'] and row['left_applied']==row['right_applied']==0 for row in motors)
    motor_check=stopped if motors else not require_motor_telemetry
    report=stats.report(imu_protocol.Parser())
    rate=report['raw_read_rate_hz']
    duration=((samples[-1]['timestamp_us']-samples[0]['timestamp_us'])&0xffffffff)/1e6 if samples else 0
    continuous=all(report[name]==0 for name in ('serial_sequence_gaps','sequence_resets','duplicate_sequences','raw_stride_anomalies'))
    return dict(passed=len(samples)>=5 and duration>=min_duration_s and identities and healthy and clean and motor_check and continuous and parser.bad_crc==0 and rate is not None and 900<=rate<=1100,
        imu_samples=len(samples),motor_packets=len(motors),identity_passed=bool(frames) and identities,
        sample_span_s=duration,required_sample_span_s=min_duration_s,sequence_continuity_passed=continuous,
        healthy_samples=bool(samples) and healthy,counters_zero=clean,motors_stopped=stopped,
        bad_crc=parser.bad_crc,imu_statistics=report,motor_telemetry_required=require_motor_telemetry,
        note='Read-only acquisition; no motion commands. Counts successful reads, not unique sensor conversions. Axis mounting and dynamic performance need physical checks.')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    source=ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--input',type=Path,help='Offline mixed UART capture')
    source.add_argument('--port',help='Exclusive PC USB UART port; stop its current owner first')
    ap.add_argument('--seconds',type=float,default=10)
    ap.add_argument('--imu-only',action='store_true',help='Only for the dedicated constant-low motor bringup bitstream; no motor UART packets')
    ap.add_argument('--out',type=Path)
    a=ap.parse_args()
    if not 1<=a.seconds<=600:ap.error('--seconds must be 1..600')
    out=a.out or ROOT/'results'/('mpu6050_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    if a.input:data=a.input.read_bytes()
    else:
        import serial
        data=bytearray()
        with serial.Serial(a.port,115200,timeout=.05) as port:
            end=time.monotonic()+a.seconds
            while time.monotonic()<end:data.extend(port.read(4096))
        data=bytes(data)
    (out/'rx.bin').write_bytes(data)
    report=evaluate(data,min_duration_s=.8*a.seconds if a.port else 0,require_motor_telemetry=not a.imu_only)
    report.update(hardware_tested=bool(a.port),serial_port=a.port,serial_bytes_written=0,programmed=False)
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':main()
