"""Task-4 wire codec. Importing this module never opens a port."""
import math,struct
REGISTER_NAMES=('confirmed_mask','left_mm_per_count_q16','right_mm_per_count_q16','track_mm','robot_radius_mm','margin_mm',
 'stop_mm','slow_mm','v_max_mm_s','w_max_mrad_s','lookahead_min_mm','lookahead_max_mm','lookahead_ms','accept_mm',
 'block_low_mm','block_high_mm','vfh_weights','lidar_timeout_ms','left_counts_per_mm_s_q16','right_counts_per_mm_s_q16',
 'gyro_axis_sign','gyro_bias_raw','gyro_step_q16','gyro_blend_q8','lidar_angle_offset_q100','lidar_reverse','min_speed_mm_s',
 'waypoint_count','valley_width','max_counts_per_tick','imu_timeout_ms','reserved')
def crc16(data):
    c=0xffff
    for byte in data:
        c^=byte<<8
        for _ in range(8):c=((c<<1)^0x1021)&65535 if c&0x8000 else (c<<1)&65535
    return c
def packet(op,seq,a0=0,a1=0,a2=0,a3=0,flags=0):
    body=b'\xa5\x5a'+struct.pack('<BBBBHHHH',1,op,seq&255,flags,a0&65535,a1&65535,a2&65535,a3&65535)
    return body+struct.pack('<H',crc16(body))
def write_register(seq,index,value):return packet(0x40,seq,index,value,value>>16)
def waypoint(seq,index,x_mm,y_mm,speed):
    if not 0<=index<16 or not -16000<=x_mm<=16000 or not -16000<=y_mm<=16000 or not 1<=speed<=500:raise ValueError('Waypoint outside supported bounds')
    return packet(0x41,seq,index,x_mm,y_mm,speed)
def calibration_registers(calibration,mission):
    c=calibration
    needed=('wheel_diameter_mm','left_counts_per_rev','right_counts_per_rev','track_mm','robot_radius_mm','gyro_axis_sign',
            'gyro_bias_raw','lidar_angle_offset_q100','lidar_reverse')
    if c.get('confirmed') is not True or any(c.get(k) is None for k in needed):raise ValueError('Physical calibration is pending; configuration/start refused')
    for k in ('wheel_diameter_mm','left_counts_per_rev','right_counts_per_rev','track_mm','robot_radius_mm'):
        if not isinstance(c[k],(int,float)) or not math.isfinite(c[k]) or c[k]<=0:raise ValueError('Invalid calibration '+k)
    if not 1<=len(mission)<=16:raise ValueError('Use 1..16 waypoints')
    circumference=math.pi*c['wheel_diameter_mm']
    values={1:round(circumference/c['left_counts_per_rev']*65536),2:round(circumference/c['right_counts_per_rev']*65536),
      3:round(c['track_mm']),4:math.ceil(c['robot_radius_mm']),5:100,6:450,7:1000,8:120,9:400,10:300,11:800,12:1000,13:120,
      14:1200,15:1400,16:0x030105,17:200,18:round(c['left_counts_per_rev']/(100*circumference)*65536),
      19:round(c['right_counts_per_rev']/(100*circumference)*65536),20:c['gyro_axis_sign'],21:c['gyro_bias_raw'],
      22:c.get('gyro_step_q16',2000),23:c.get('gyro_blend_q8',192),24:c['lidar_angle_offset_q100'],25:int(c['lidar_reverse']),26:30,27:len(mission),28:8,29:32,30:20}
    # Overrides are explicit named run parameters; geometry still comes from the
    # separately confirmed measurement fields above.
    for name,value in c.get('parameters',{}).items():
        if name not in REGISTER_NAMES or REGISTER_NAMES.index(name) in (0,1,2,3,4,18,19,27,31):raise ValueError('Invalid parameter override '+name)
        values[REGISTER_NAMES.index(name)]=int(value)
    if not 1<=values[1]<=65536 or not 1<=values[2]<=65536 or not 1<=values[18]<=65536 or not 1<=values[19]<=65536:raise ValueError('Encoder scale outside RTL bounds')
    if not 50<=values[3]<=2000 or not 20<=values[4]<=1500 or not 0<=values[5]<=500:raise ValueError('Geometry outside RTL bounds')
    if not values[4]+values[5]<values[6]<values[7]<=values[14]<values[15]<=8000:raise ValueError('Inconsistent safety distances')
    if not 0<values[8]<=500 or not 0<values[9]<=1500 or values[8]>values[9]*8 or not 100<=values[10]<=values[11]<=2000:raise ValueError('Invalid speed/lookahead')
    if not 0<=values[12]<=3000 or not 20<=values[13]<=1000 or not 50<=values[17]<=1000:raise ValueError('Invalid time/distance parameter')
    if values[20] not in (0,1,2,4,5,6) or not -32767<=values[21]<=32767 or not 0<values[22]<=10000 or not 0<=values[23]<=256:raise ValueError('Invalid IMU calibration')
    if not -18000<=values[24]<=18000 or values[25] not in (0,1) or not 0<=values[26]<=values[8] or not 2<=values[28]<=16 or not 1<=values[29]<=128 or not 2<=values[30]<=100:raise ValueError('Invalid navigation parameter')
    wt,wc,wp=values[16]&255,(values[16]>>8)&255,(values[16]>>16)&255
    if not wt>wc+wp:raise ValueError('Target weight must exceed current+previous weights')
    return values
def configuration_frames(calibration,mission,start_sequence=1):
    values=calibration_registers(calibration,mission);seq=start_sequence;frames=[packet(0x44,seq)];seq+=1
    # Invalidate confirmation first, and write confirmation last.
    frames.append(write_register(seq,0,0));seq+=1
    for index,value in sorted(values.items()):frames.append(write_register(seq,index,value));seq+=1
    for index,p in enumerate(mission):frames.append(waypoint(seq,index,p['x_mm'],p['y_mm'],p.get('speed_mm_s',120)));seq+=1
    frames.append(write_register(seq,0,15));seq+=1
    return frames,seq&255
def decode_status(frame):
    if len(frame)!=48 or frame[:4]!=b'\xc3\x3c\x04\x01' or crc16(frame[:46])!=struct.unpack_from('<H',frame,46)[0]:raise ValueError('Invalid navigation status or CRC')
    flags,index=frame[4:6];faults=struct.unpack_from('<H',frame,6)[0]
    x,y,heading,v,w,l,r,front,plans,packets,errors,rejected=struct.unpack_from('<iiihhhhHIIII',frame,8)
    return dict(running=bool(flags&1),done=bool(flags&2),permit=bool(flags&4),waypoint_index=index,faults=faults,
      x_mm=x/65536,y_mm=y/65536,heading_degrees=heading/6553600,v_mm_s=v,w_mrad_s=w,left_target=l,right_target=r,
      front_mm=front,plans=plans,lidar_packets=packets,lidar_errors=errors,rejected=rejected)
class StatusStream:
    def __init__(self):self.buffer=bytearray();self.bad_crc=0
    def feed(self,data):
        self.buffer.extend(data);out=[]
        while True:
            at=self.buffer.find(b'\xc3\x3c\x04\x01')
            if at<0:self.buffer[:]=self.buffer[-3:];break
            del self.buffer[:at]
            if len(self.buffer)<48:break
            try:row=decode_status(bytes(self.buffer[:48]))
            except ValueError:self.bad_crc+=1;del self.buffer[0];continue
            del self.buffer[:48];out.append(row)
        return out
