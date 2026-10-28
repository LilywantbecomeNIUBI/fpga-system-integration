"""Pure codec. No hardware access and no assumed encoder CPR/gearing."""
import struct

COMMAND_HEADER=b'\xa5\x5a'
STATUS_HEADER=b'\xc3\x3c'
FAULT_NAMES={0:'estop_or_power_on',1:'command_timeout',2:'encoder_invalid',3:'encoder_speed_limit',4:'no_encoder_motion'}

def crc16(data):
    crc=0xffff
    for value in data:
        crc^=value<<8
        for _ in range(8):
            crc=((crc<<1)^0x1021)&0xffff if crc&0x8000 else (crc<<1)&0xffff
    return crc

def command(opcode,sequence,flags=0,left=0,right=0,kp=256,ki=0):
    if opcode not in (0,1,2,3):raise ValueError('opcode must be STOP=0 ARM=1 DRIVE=2 CLEAR=3')
    if not 0<=flags<=31 or not 0<=kp<=4096 or not 0<=ki<=1024:raise ValueError('invalid flags/gains')
    bound=1024 if flags&1 else 1000
    if not -bound<=left<=bound or not -bound<=right<=bound:raise ValueError('target exceeds command range')
    if opcode in (1,3) and (left or right):raise ValueError('ARM/CLEAR require zero targets')
    payload=COMMAND_HEADER+struct.pack('<BBBBhhHH',1,opcode,sequence&255,flags,left,right,kp,ki)
    return payload+struct.pack('<H',crc16(payload))

def decode_status(frame):
    if len(frame)!=48 or frame[:2]!=STATUS_HEADER or frame[2]!=1:raise ValueError('invalid status header/length/version')
    if crc16(frame[:46])!=struct.unpack_from('<H',frame,46)[0]:raise ValueError('status CRC mismatch')
    values=struct.unpack_from('<iiii hh IIII H',frame,8)
    names=('left_position','right_position','left_velocity','right_velocity','left_applied','right_applied',
        'left_invalid','right_invalid','bad_frames','rejected_commands','packet_count')
    result=dict(zip(names,values))
    result.update(last_sequence=frame[3],armed=bool(frame[4]),flags=frame[5],faults=struct.unpack_from('<H',frame,6)[0])
    result['fault_names']=[name for bit,name in FAULT_NAMES.items() if result['faults']&(1<<bit)]
    return result

class StatusStream:
    def __init__(self):self.buffer=bytearray()
    def feed(self,data):
        self.buffer.extend(data);result=[]
        while len(self.buffer)>=2:
            start=self.buffer.find(STATUS_HEADER)
            if start<0:
                self.buffer[:]=self.buffer[-1:];break
            del self.buffer[:start]
            if len(self.buffer)<48:break
            try:status=decode_status(bytes(self.buffer[:48]))
            except ValueError:del self.buffer[0];continue
            result.append(status);del self.buffer[:48]
        return result
