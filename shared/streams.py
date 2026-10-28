"""Mixed motor V1 / IMU v2 UART and STL-27L read-only stream decoders."""
from pathlib import Path
import importlib.util,struct,sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from motor_protocol import crc16,decode_status
spec=importlib.util.spec_from_file_location('imu_protocol',str(ROOT/'vendor/imu/pc/imu_viewer.py'))
imu_protocol=importlib.util.module_from_spec(spec);spec.loader.exec_module(imu_protocol)

class MixedStream:
    def __init__(self):self.buffer=bytearray();self.bad_crc=0;self.discarded=0
    def feed(self,data):
        self.buffer.extend(data);rows=[]
        while len(self.buffer)>=3:
            header=bytes(self.buffer[:3])
            if header not in (b'\xc3\x3c\x01',b'\xa5\x5a\x02'):
                del self.buffer[0];self.discarded+=1;continue
            if len(self.buffer)<48:break
            frame=bytes(self.buffer[:48]);motor=header[0]==0xc3
            expected=struct.unpack('<H' if motor else '>H',frame[46:])[0]
            if crc16(frame[:46])!=expected:
                self.bad_crc+=1;del self.buffer[0];continue
            del self.buffer[:48]
            if motor:rows.append(('motor',decode_status(frame)))
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
