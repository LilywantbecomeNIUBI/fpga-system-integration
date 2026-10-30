"""Mandatory calibrated geometry before accepting physical-unit control."""
import math

class Calibration:
    def __init__(self,data=None):
        self.data=data or {};self.valid=False
        keys=('left_cpr','right_cpr','left_diameter_m','right_diameter_m','track_m')
        try:
            if self.data.get('confirmed') is not True:return
            for key in keys:
                value=self.data[key]
                if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:return
            for key in ('motor_reverse_mask','encoder_reverse_mask'):
                if type(self.data.get(key)) is not int or not 0<=self.data[key]<=3:return
            self.valid=True
        except (KeyError,TypeError):return
    def targets(self,v,w):
        if not self.valid:raise ValueError('Physical control requires measured calibration')
        if not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in (v,w)):raise ValueError('Non-finite velocity')
        if abs(v)>.15 or abs(w)>.5:raise ValueError('Commissioning velocity limit')
        d=self.data;l=v-w*d['track_m']/2;r=v+w*d['track_m']/2
        values=[round(speed*d[side+'_cpr']/(math.pi*d[side+'_diameter_m'])*.01) for side,speed in (('left',l),('right',r))]
        if any(abs(x)>1024 for x in values):raise ValueError('Wheel target exceeds FPGA limit')
        return values

class Odometry:
    def __init__(self,calibration):self.cal=calibration;self.previous=None;self.x=self.y=self.yaw=0.0
    def reset_reference(self):self.previous=None
    def update(self,motor):
        current=(motor['left_position'],motor['right_position'])
        if self.cal.valid and self.previous is not None:
            delta=[((a-b+2**31)%2**32)-2**31 for a,b in zip(current,self.previous)]
            d=self.cal.data
            dl=delta[0]*math.pi*d['left_diameter_m']/d['left_cpr'];dr=delta[1]*math.pi*d['right_diameter_m']/d['right_cpr']
            dyaw=(dr-dl)/d['track_m'];distance=(dl+dr)/2
            self.x+=distance*math.cos(self.yaw+dyaw/2);self.y+=distance*math.sin(self.yaw+dyaw/2)
            self.yaw=(self.yaw+dyaw+math.pi)%(2*math.pi)-math.pi
        self.previous=current
    def snapshot(self):return dict(valid=self.cal.valid and self.previous is not None,x_m=self.x,y_m=self.y,yaw_rad=self.yaw,method='encoder dead reckoning; no SLAM or IMU fusion')
