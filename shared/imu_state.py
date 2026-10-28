import math,threading,time
from shared.streams import imu_protocol

class ImuState:
    def __init__(self):
        self.parser = imu_protocol.Parser()
        self.stats = imu_protocol.Statistics()
        self.lock = threading.Lock()
        self.last = None
        self.received = None
        self.error = None
        self.pitch = self.roll = None
        self.previous = None

    def feed(self, payload, now=None, rows=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            for row in (self.parser.feed(payload) if rows is None else rows):
                self.stats.add(row)
                self.last, self.received = row, now
                good = row['type']=='sample' and row['ready'] and not row['fault'] and not row['stale'] and row['who_am_i']==0x47
                if not good:
                    self.previous = None
                    self.pitch = self.roll = None
                    continue
                ax,ay,az = (row[k] for k in ('ax_g','ay_g','az_g'))
                norm = math.sqrt(ax*ax+ay*ay+az*az)
                if norm < .2 or norm > 2:
                    self.previous = None
                    self.pitch = self.roll = None
                    continue
                pitch = math.degrees(math.atan2(-ax, math.hypot(ay,az)))
                roll = math.degrees(math.atan2(ay,az))
                dt = ((row['timestamp_us']-self.previous)&0xffffffff)/1e6 if self.previous is not None else 0
                if 0<dt<.05 and self.pitch is not None:
                    self.pitch = .98*(self.pitch+row['gy_dps']*dt)+.02*pitch
                    # Circular roll innovation avoids the +/-180 degree discontinuity.
                    prediction = self.roll+row['gx_dps']*dt
                    self.roll = (prediction+.02*((roll-prediction+180)%360-180)+180)%360-180
                else:
                    self.pitch,self.roll = pitch,roll
                self.previous = row['timestamp_us']
            self.error = None

    def snapshot(self, now=None):
        now=time.monotonic() if now is None else now
        with self.lock:
            age = now-self.received if self.received is not None else None
            row=dict(self.last) if self.last else None
            healthy = bool(row and row['type']=='sample' and row['ready'] and not row['fault'] and not row['stale'] and row['who_am_i']==0x47)
            return dict(online=age is not None and age<.5 and not self.error, healthy=healthy,
                age_ms=age*1000 if age is not None else None,error=self.error,
                frame=row,statistics=self.stats.report(self.parser),
                attitude=dict(valid=bool(healthy and age is not None and age<.5 and not self.error and self.pitch is not None),
                    pitch_deg=self.pitch,roll_deg=self.roll,yaw_deg=None,
                    convention='sensor axes; roll about +X, pitch about +Y, upright +Z; yaw unavailable',
                    method='host complementary filter of decimated IMU; visualization only, no motor control'))

