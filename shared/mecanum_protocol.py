"""Mecanum V3 status codec, ported from the frozen team console. No IO."""
import math, struct
from host.motor_protocol import crc16
PROTOCOL_VERSION=3


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


def decode_mecanum(frame):
    if len(frame)!=48 or frame[:3] not in (b'M4\x03',b'P4\x03'):
        raise ValueError('Invalid Mecanum status identity')
    rows = Stream().feed(frame)
    if len(rows)!=1:
        raise ValueError('Invalid Mecanum status CRC')
    return rows[0]
