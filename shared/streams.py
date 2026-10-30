"""Mixed motor V1 / IMU v2 UART and STL-27L read-only stream decoders."""
from pathlib import Path
import struct,sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from motor_protocol import crc16,decode_status
ENCODER_COUNTER_NAMES=('sync_a_edges','sync_b_edges','filtered_a_edges','filtered_b_edges',
                       'a_positive','a_negative','b_positive','b_negative','diagonal')
ENCODER_MODES={0xa2:'a_edge_x2_scaled4',0xb2:'b_edge_x2_scaled4',0xd4:'quadrature_continuity_x4'}

def decode_encoder_diagnostics(frame):
    if len(frame)!=48 or frame[:3]!=b'\xc3\x3c\x02' or frame[3] not in (1,2) or frame[6] not in ENCODER_MODES:
        raise ValueError('Invalid encoder diagnostic identity')
    if crc16(frame[:46])!=struct.unpack_from('<H',frame,46)[0]:raise ValueError('Encoder diagnostic CRC mismatch')
    row=dict(zip(ENCODER_COUNTER_NAMES,struct.unpack_from('<9I',frame,8)))
    row.update(axis=frame[3],sequence=struct.unpack_from('<H',frame,4)[0],mode=ENCODER_MODES[frame[6]],
               synchronized_ab=(frame[7]>>2)&3,filtered_ab=frame[7]&3)
    return row

def decode_encoder_trace(frame):
    if len(frame)!=48 or frame[:3]!=b'\xc3\x3c\x03' or frame[3] not in (1,2) or frame[6] not in ENCODER_MODES:
        raise ValueError('Invalid encoder trace identity')
    if crc16(frame[:46])!=struct.unpack_from('<H',frame,46)[0]:raise ValueError('Encoder trace CRC mismatch')
    count=struct.unpack_from('<I',frame,40)[0]
    packed=int.from_bytes(frame[8:40],'little')
    transitions=[(packed>>(4*i))&15 for i in reversed(range(min(count,64)))]
    return dict(axis=frame[3],sequence=struct.unpack_from('<H',frame,4)[0],event_count=count,
                mode=ENCODER_MODES[frame[6]],
                transitions=transitions,synchronized_ab=(frame[7]>>2)&3,filtered_ab=frame[7]&3)
from shared import imu_protocol

class MixedStream:
    def __init__(self):self.buffer=bytearray();self.bad_crc=0;self.discarded=0
    def feed(self,data):
        self.buffer.extend(data);rows=[]
        while len(self.buffer)>=3:
            header=bytes(self.buffer[:3])
            if header not in (b'\xc3\x3c\x01',b'\xc3\x3c\x02',b'\xc3\x3c\x03',b'\xa5\x5a\x02'):
                del self.buffer[0];self.discarded+=1;continue
            if len(self.buffer)<48:break
            frame=bytes(self.buffer[:48]);motor=header[0]==0xc3
            expected=struct.unpack('<H' if motor else '>H',frame[46:])[0]
            if crc16(frame[:46])!=expected:
                self.bad_crc+=1;del self.buffer[0];continue
            del self.buffer[:48]
            if header==b'\xc3\x3c\x03':
                try:rows.append(('encoder_trace',decode_encoder_trace(frame)))
                except ValueError:self.discarded+=48
            elif header==b'\xc3\x3c\x02':
                try:rows.append(('encoder_diagnostics',decode_encoder_diagnostics(frame)))
                except ValueError:self.discarded+=48
            elif motor:rows.append(('motor',decode_status(frame)))
            else:
                decoded=imu_protocol.Parser().feed(frame)
                if decoded:rows.append(('imu',decoded[0]))
        return rows

def crc8(data):
    value=0
    for byte in data:
        value^=byte
        for _ in range(8):value=((value<<1)^0x4d)&255 if value&128 else(value<<1)&255
    return value

class LidarStream:
    """47-byte 54 2C packets; scan assembly only accepts continuous complete revolutions."""
    def __init__(self):
        self.buffer=bytearray();self.bad_crc=0;self.frames=0;self.scans=0
        self.points=[];self.previous=None;self.collecting=False;self.bad_scan=False
        self.last_scan=None;self.last_packet=None
    def reset_scan(self):self.points=[];self.previous=None;self.collecting=False;self.bad_scan=False
    def feed(self,data):
        self.buffer.extend(data);completed=[]
        while len(self.buffer)>=2:
            if self.buffer[:2]!=b'\x54\x2c':del self.buffer[0];continue
            if len(self.buffer)<47:break
            frame=bytes(self.buffer[:47])
            if crc8(frame[:46])!=frame[46]:
                self.bad_crc+=1;self.reset_scan();del self.buffer[0];continue
            del self.buffer[:47];self.frames+=1
            speed,start=struct.unpack_from('<HH',frame,2);end,stamp=struct.unpack_from('<HH',frame,42)
            if start>=36000 or end>=36000 or (end-start)%36000>3000:
                self.reset_scan();continue
            self.last_packet=dict(speed_hz=speed/360.0,timestamp_ms=stamp)
            span=(end-start)%36000
            for i in range(12):
                angle=((start+span*i/11.0)/100.0)%360
                distance,intensity=struct.unpack_from('<HB',frame,6+3*i)
                if self.previous is not None:
                    delta=(angle-self.previous)%360
                    if delta>10:self.bad_scan=True
                    if angle<self.previous and self.previous-angle>180:
                        if self.collecting and not self.bad_scan and len(self.points)>=12:
                            self.scans+=1;self.last_scan=dict(sequence=self.scans,points=self.points,**self.last_packet)
                            completed.append(self.last_scan)
                        self.points=[];self.collecting=True;self.bad_scan=False
                self.previous=angle
                if self.collecting and len(self.points)<10000:self.points.append([angle,distance,intensity])
        return completed
