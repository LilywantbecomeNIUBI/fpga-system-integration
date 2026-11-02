"""Assemble complete revolutions from every accepted FPGA parser point.

There is no sector reduction or return filtering in this state. Python 3.6+.
"""


class RawLidar:
    def __init__(self):
        self.pending = None
        self.last_sequence = None
        self.last_capture_ms = None
        self.last_received = None
        self.previous_angle = None
        self.collecting = False
        self.bad = False
        self.points = []
        self.scan = self.scan_time = None
        self.scans = self.incomplete = self.packets = self.missing_packets = 0
        self.clock_origin = self.clock_updated = None
        self.elapsed_capture_ms = 0
        self.stale_packets = 0

    def invalidate(self):
        if self.pending or self.collecting:
            self.incomplete += 1
        self.pending = None
        self.points = []
        self.collecting = False
        self.previous_angle = None
        self.bad = False

    def ingest(self, row, now):
        seq, part = row['packet_sequence'], row['half']
        if self.last_sequence is not None and not 0 < ((seq-self.last_sequence)&0xffffffff) < 0x80000000:
            return
        if self.last_received is not None and now-self.last_received >= .25:
            self.invalidate()
        if part == 0:
            if self.pending:
                if self.pending['packet_sequence'] == seq:
                    return  # Duplicates cannot renew pending age.
                if not 0 < ((seq-self.pending['packet_sequence'])&0xffffffff) < 0x80000000:
                    return
                self.invalidate()
            self.pending = dict(row, received=now, packet_time=now-row['source_age_ms']/1000.)
            return
        first = self.pending
        if (first is None or first['packet_sequence'] != seq or first['captured_ms'] != row['captured_ms']
                or first['discontinuity'] != row['discontinuity'] or now-first['received'] >= .25):
            self.invalidate()
            return
        points = first['points']+row['points']
        packet_time = min(first['packet_time'],now-row['source_age_ms']/1000.)
        self.pending = None
        if self.last_sequence is not None:
            delta = (seq-self.last_sequence)&0xffffffff
            if delta != 1:
                self.missing_packets += delta-1
                self.invalidate()
        capture_delta = 0 if self.last_capture_ms is None else (row['captured_ms']-self.last_capture_ms)&0xffffffff
        if self.last_capture_ms is not None and capture_delta >= 250:
            self.invalidate()
        if row['discontinuity']:
            self.invalidate()
        if self.last_capture_ms is None or capture_delta >= 0x80000000:
            self.elapsed_capture_ms=row['captured_ms'];self.clock_origin=None
        else:self.elapsed_capture_ms+=capture_delta
        candidate=packet_time-self.elapsed_capture_ms/1000.
        drift=0 if self.clock_updated is None else max(0,min(1,now-self.clock_updated))*.0002
        self.clock_origin=candidate if self.clock_origin is None else min(self.clock_origin+drift,candidate)
        self.clock_updated=now
        packet_time=min(packet_time,self.clock_origin+self.elapsed_capture_ms/1000.)
        self.last_sequence,self.last_capture_ms,self.last_received = seq,row['captured_ms'],now
        self.packets += 1
        if now-packet_time >= .4:
            self.stale_packets+=1;self.invalidate();return
        for point in points:
            angle = point[0]
            if self.previous_angle is not None:
                delta = (angle-self.previous_angle)%360
                if delta > 10:
                    self.bad = True
                if angle < self.previous_angle and self.previous_angle-angle > 180:
                    if self.collecting and not self.bad and len(self.points) >= 360:
                        self.scans += 1
                        self.scan = dict(sequence=self.scans,points=self.points,
                            representation='raw_points',angle_mode='measured',resolution_degrees=None,
                            average_angle_step_degrees=360./len(self.points),captured_ms=row['captured_ms'])
                        self.scan_time = packet_time
                    elif self.collecting:
                        self.incomplete += 1
                    self.points = []
                    self.collecting,self.bad = True,False
            self.previous_angle = angle
            if self.collecting:
                if len(self.points) < 8192:
                    self.points.append(point)
                else:
                    self.bad = True
