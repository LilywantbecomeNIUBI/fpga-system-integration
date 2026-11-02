"""PC command source only. Kinematics, speed PI and PWM execute in FPGA."""
import argparse
import struct
import time
import math
import json
from pathlib import Path

PROTOCOL_VERSION=3


def motor_profile():
    return json.loads((Path(__file__).resolve().parents[1]/'config/motor.json').read_text(encoding='utf-8'))


def mm_to_control_counts(distance_mm, raw_counts_per_rev, diameter_mm, shift):
    if not all(math.isfinite(v) for v in (distance_mm,raw_counts_per_rev,diameter_mm)) or raw_counts_per_rev<=0 or diameter_mm<=0 or not 0<=shift<=8:
        raise ValueError('Invalid encoder/wheel calibration')
    return round(distance_mm*raw_counts_per_rev/((1<<shift)*math.pi*diameter_mm))


def polar_displacement(distance_mm, bearing_deg):
    """Local displacement: 0deg forward, +90deg left; body heading unchanged."""
    if not math.isfinite(distance_mm) or not 0 < distance_mm <= 1000:
        raise ValueError('移动距离应大于0且不超过1000毫米')
    if not math.isfinite(bearing_deg) or not -180 <= bearing_deg <= 180:
        raise ValueError('移动方向应在-180～180度之间')
    bearing=math.radians(bearing_deg)
    return distance_mm*math.cos(bearing), distance_mm*math.sin(bearing)


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def packet(op, seq, motor=0, encoder=0, x=0, y=0, z=0, kp=256, ki=0, limit=40, tolerance=4):
    if op not in (0,1,3,6,7,8):
        raise ValueError('Unsupported opcode; manual, single-wheel and velocity-heading modes have been removed')
    if motor != 0 or encoder != 0:
        raise ValueError('Direction is fixed in FPGA; reserved mask fields must be zero')
    if op == 6:
        bound = 16000
        if not (-bound <= x <= bound and -bound <= y <= bound and -18000 <= z <= 18000):
            raise ValueError('Outer-loop targets out of range')
        if not 1 <= limit <= 128 or not 1 <= tolerance <= 1024:
            raise ValueError('limit=1..128, tolerance=1..1024 counts')
    elif not all(-1024 <= v <= 1024 for v in (x, y, z)):
        raise ValueError('Targets must be -1024..1024')
    if not 1 <= kp <= 4096 or not 0 <= ki <= 1024:
        raise ValueError('kp=1..4096, ki=0..1024 (Q8)')
    body = struct.pack('<2s6B3h4H', b'\xa5\x5a', PROTOCOL_VERSION, op, seq & 255,
                       motor, encoder, 0, x, y, z, kp, ki,
                       limit if op == 6 else 0, tolerance if op == 6 else 0)
    return body + struct.pack('<H', crc16(body))


class Stream:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        rows = []
        while len(self.buffer) >= 48:
            if self.buffer[:2] not in (b'M4', b'P4') or self.buffer[2] != PROTOCOL_VERSION:
                del self.buffer[0]
                continue
            frame = self.buffer[:48]
            if crc16(frame[:46]) != struct.unpack_from('<H', frame, 46)[0]:
                del self.buffer[0]
                continue
            if frame[:2] == b'P4':
                flags=frame[3]
                x,y,heading,yaw=struct.unpack_from('<4i',frame,8)
                rows.append(dict(kind='pose',healthy=bool(flags&1),calibrated=bool(flags&2),arrived=bool(flags&4),
                                 mode=2 if flags&16 else 1 if flags&8 else 0,imu_fault=bool(flags&32),
                                 faults=struct.unpack_from('<H',frame,4)[0],seq=frame[6],imu_error=frame[7],
                                 x_counts=x/65536,y_counts=y/65536,heading_deg=math.degrees(heading/65536),
                                 yaw_deg=math.degrees(yaw/65536),bias=struct.unpack_from('<h',frame,24)[0],
                                 gz=struct.unpack_from('<h',frame,26)[0],sample_us=struct.unpack_from('<I',frame,28)[0],who=frame[42],
                                 control_shift=frame[32],raw_counts_per_rev=struct.unpack_from('<I',frame,34)[0],
                                 imu_diag_version=frame[33],imu_fault_count=struct.unpack_from('<H',frame,38)[0],
                                 imu_bus_error=frame[40]&15,imu_bus_lines=(frame[40]>>4)&3,
                                 imu_fault_armed=bool(frame[41]&1),imu_fault_pwm=bool(frame[41]&2),imu_fault_outer=bool(frame[41]&4),imu_stale=bool(frame[41]&8)))
                del self.buffer[:48]
                continue
            rows.append(dict(kind='motor',armed=bool(frame[3]), faults=struct.unpack_from('<H', frame, 4)[0],
                             seq=frame[6], position=struct.unpack_from('<4i', frame, 8),
                             velocity=struct.unpack_from('<4h', frame, 24),
                             pwm=struct.unpack_from('<4h', frame, 32),
                             bad=struct.unpack_from('<H', frame, 40)[0],
                             rejected=struct.unpack_from('<H', frame, 42)[0],control_shift=frame[44]))
            del self.buffer[:48]
        return rows


def main():
    profile=motor_profile()
    shift=profile['control_shift']
    parser = argparse.ArgumentParser(description='ACG720 mecanum, FL FR RL RR order')
    parser.add_argument('--port', required=True)
    parser.add_argument('--action', choices=['monitor', 'stop', 'position', 'turn', 'vector'], default='monitor')
    parser.add_argument('--kp', type=int, default=profile['initial_kp_q8'])
    parser.add_argument('--ki', type=int, default=profile['initial_ki_q8'])
    parser.add_argument('--seconds', type=float, default=3)
    parser.add_argument('--distance',type=float,default=200,help='vector travel distance in mm, >0..1000')
    parser.add_argument('--bearing',type=float,default=45,help='vector direction: 0 forward, 90 left, -90 right, +/-180 backward')
    parser.add_argument('--angle',type=float,default=0,help='relative CCW target degrees, -180..180')
    parser.add_argument('--dx-counts',type=int,default=0,help='relative forward position in encoder counts')
    parser.add_argument('--dy-counts',type=int,default=0,help='relative left position in encoder counts')
    parser.add_argument('--dx',type=float,default=None,help='relative forward position in mm; requires counts-per-rev')
    parser.add_argument('--dy',type=float,default=None,help='relative left position in mm; requires counts-per-rev')
    parser.add_argument('--counts-per-rev',type=float,default=profile['raw_counts_per_rev'],help='raw output shaft x4-equivalent counts/revolution; MG513P30 GMR default=60000')
    parser.add_argument('--diameter-mm',type=float,default=profile['wheel_diameter_mm'])
    parser.add_argument('--limit',type=int,default=40,help='outer-loop axis speed cap, counts/10ms')
    parser.add_argument('--tolerance',type=int,default=4,help='position tolerance, counts')
    args = parser.parse_args()
    if not 0 < args.seconds <= 3600:
        parser.error('seconds must be >0 and <=3600')
    if not math.isfinite(args.counts_per_rev) or args.counts_per_rev<=0 or not math.isfinite(args.diameter_mm) or args.diameter_mm<=0:
        parser.error('counts-per-rev and diameter-mm must be positive and finite')
    x=y=z=0
    outer=args.action in ('position','turn','vector')
    motion_op=6 if args.action in ('position','turn','vector') else 0
    if args.action in ('position','turn','vector'):
        motion_op=6;x=args.dx_counts;y=args.dy_counts;z=round(args.angle*100)
        if args.dx is not None or args.dy is not None:
            if args.dx_counts or args.dy_counts:parser.error('choose mm or counts, not both')
            x=mm_to_control_counts(args.dx or 0,args.counts_per_rev,args.diameter_mm,shift)
            y=mm_to_control_counts(args.dy or 0,args.counts_per_rev,args.diameter_mm,shift)
        if args.action=='turn':x=0;y=0
    if args.action=='vector':
        if args.dx is not None or args.dy is not None or args.dx_counts or args.dy_counts or args.angle:
            parser.error('vector uses distance/bearing only; it keeps the current body heading')
        try:dx,dy=polar_displacement(args.distance,args.bearing)
        except ValueError as exc:parser.error(str(exc))
        x=mm_to_control_counts(dx,args.counts_per_rev,args.diameter_mm,shift)
        y=mm_to_control_counts(dy,args.counts_per_rev,args.diameter_mm,shift)
        z=0
    cfg = dict(motor=0, encoder=0, kp=args.kp, ki=args.ki)
    # Validate everything before opening a physical port.
    packet(motion_op, 0, x=x, y=y, z=z, limit=args.limit,tolerance=args.tolerance, **cfg)
    import serial
    stream = Stream()
    with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.2) as port:
        seq = 0

        def send(op, values=(0, 0, 0)):
            nonlocal seq
            seq = (seq+1) & 255
            port.write(packet(op, seq, x=values[0], y=values[1], z=values[2],limit=args.limit,tolerance=args.tolerance, **cfg))
            return seq

        def wait_state(predicate, seconds=1.0):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                for row in stream.feed(port.read(256)):
                    if predicate(row):
                        return row
            raise RuntimeError('FPGA status timeout / requested state not acknowledged')

        if args.action == 'stop':
            send(0)
            print(wait_state(lambda r: r['kind']=='motor' and not r['armed']))
            return
        if args.action == 'monitor':
            end = time.monotonic()+args.seconds
            while time.monotonic() < end:
                for row in stream.feed(port.read(256)):
                    print(row)
            return
        try:
            # Explicit motion action starts a fresh session; no automatic fault restart.
            send(0)
            stopped=wait_state(lambda r: r['kind']=='motor' and not r['armed'])
            if stopped['control_shift']!=shift:
                raise RuntimeError('Firmware/control-unit profile mismatch; load matching V3 firmware')
            send(3)
            wait_state(lambda r: r['kind']=='motor' and not r['armed'] and r['faults'] == 0)
            if outer:
                print('Waiting for MPU6050 calibration. Keep the chassis still.')
                wait_state(lambda r:r['kind']=='pose' and r['healthy'],seconds=8.0)
            arm_seq = send(1)
            wait_state(lambda r: r['kind']=='motor' and r['armed'] and r['faults'] == 0 and r['seq'] == arm_seq,seconds=0.2)
            if outer:
                goal_seq=send(motion_op,(x,y,z))
                wait_state(lambda r:r['kind']=='motor' and r['armed'] and r['faults']==0 and r['seq']==goal_seq,seconds=0.2)
            end = time.monotonic()+args.seconds
            next_send = 0
            last_status = time.monotonic()
            last_pose = last_status
            reached=False
            while time.monotonic() < end:
                now = time.monotonic()
                if now >= next_send:
                    send(7)
                    next_send = now+0.05
                for row in stream.feed(port.read(256)):
                    print(row)
                    if row['kind']=='pose':
                        last_pose=time.monotonic()
                        if outer and (not row['healthy'] or row['faults']):raise RuntimeError('Pose feedback lost / fault')
                        if args.action in ('position','turn','vector') and row['mode']==2 and row['arrived']:
                            reached=True
                        continue
                    last_status=time.monotonic()
                    if row['faults'] or not row['armed']:
                        raise RuntimeError('FPGA stopped: '+str(row))
                if time.monotonic()-last_status > 0.3:
                    raise RuntimeError('FPGA telemetry lost')
                if outer and time.monotonic()-last_pose>0.3:raise RuntimeError('Pose telemetry lost')
                if reached:
                    print('Target reached using IMU + wheel odometry.')
                    break
            if args.action in ('position','turn','vector') and not reached:
                raise RuntimeError('Target not reached before the time limit; stopping')
        finally:
            for _ in range(3):
                try:send(0)
                except (serial.SerialException,OSError):break
                time.sleep(0.01)


if __name__ == '__main__':
    main()
