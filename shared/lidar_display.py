"""M2 full-point stream and compatible 72/360-sector display protocols.

Compatible with Nano Python 3.6. Display scans never feed motor control.
"""
import struct
import time
from shared.lidar_raw import RawLidar

RAW_POINTS = struct.Struct('<' + 'HHB'*6)


def decode_lidar(frame):
    # The mixed UART decoder checks the CRC before dispatch.
    if frame[:4] == b'\xc3\x3c\x07\x01':
        seq, age, captured_ms, half, flags = struct.unpack_from('<I H I BB',frame,4)
        values = RAW_POINTS.unpack_from(frame, 16)
        if half > 1 or flags & ~1 or age >= 400 or max(values[0::3]) >= 36000:
            raise ValueError('Invalid full-point stream packet')
        return 'lidar_raw',dict(packet_sequence=seq,half=half,captured_ms=captured_ms,
            source_age_ms=age,discontinuity=bool(flags&1),
            points=[[values[i]/100.,values[i+1],values[i+2]] for i in (0,3,6,9,12,15)])
    if frame[:4] == b'\xc3\x3c\x06\x03':
        names = ('device_ms','bytes','framing_errors','packets','crc_errors','format_errors',
                 'timeouts','forwarded_packets','dropped_packets')
        row = dict(zip(names,struct.unpack_from('<9I',frame,8)))
        row.update(sequence=struct.unpack_from('<H',frame,4)[0],flags=frame[6],
                   queued_packets=struct.unpack_from('<H',frame,44)[0],representation='raw_points',resolution_degrees=None)
        if frame[7] or row['flags'] != 1 or row['queued_packets'] > 8:
            raise ValueError('Invalid full-point stream diagnostic')
        return 'lidar_diagnostic',row
    if frame[:4] == b'\xc3\x3c\x05\x02':
        sequence, chunk, chunks, age, captured_ms, count, resolution = struct.unpack_from('<HBBHIBB', frame, 4)
        if chunks != 60 or chunk >= chunks or age >= 400 or count != 6 or resolution != 1:
            raise ValueError('Invalid 360-point scan chunk')
        points = [list(struct.unpack_from('<HHB', frame, 16+i*5)) for i in range(6)]
        if any(a >= 36000 or a//100 != chunk*6+i for i,(a,d,q) in enumerate(points) if d and q):
            raise ValueError('Measured angle outside its one-degree sector')
        return 'lidar_scan', dict(sequence=sequence, chunk=chunk, chunks=chunks,
            source_age_ms=age, captured_ms=captured_ms, resolution_degrees=1,
            points=[[a/100.0, d, q] for a,d,q in points])
    if frame[:4] == b'\xc3\x3c\x05\x01':
        sequence, chunk, chunks, age = struct.unpack_from('<HBBH', frame, 4)
        if chunks != 6 or chunk >= chunks or age >= 400:
            raise ValueError('Invalid M2 scan chunk')
        return 'lidar_scan', dict(sequence=sequence, chunk=chunk, chunks=chunks,
            source_age_ms=age, points=[list(struct.unpack_from('<HB', frame, 10+i*3)) for i in range(12)])
    if frame[:4] in (b'\xc3\x3c\x06\x01', b'\xc3\x3c\x06\x02'):
        names = ('device_ms', 'bytes', 'framing_errors', 'packets', 'crc_errors',
                 'format_errors', 'timeouts', 'completed_scans', 'skipped_scans')
        row = dict(zip(names, struct.unpack_from('<9I', frame, 8)))
        row.update(sequence=struct.unpack_from('<H', frame, 4)[0], flags=frame[6],
                   coverage=struct.unpack_from('<H', frame, 44)[0])
        row['resolution_degrees'] = 1 if frame[3] == 2 else 5
        if frame[7] or row['coverage'] > (360 if frame[3] == 2 else 72) or row['flags'] & ~31:
            raise ValueError('Invalid M2 receive diagnostic')
        return 'lidar_diagnostic', row
    raise ValueError('Unknown M2 LiDAR display version')


class FpgaLidar:
    def __init__(self):
        self.reset()

    def reset(self):
        self.raw = RawLidar()
        self.raw_mode = False
        self.sequence = None
        self.pending = None
        self.scan = None
        self.scan_time = None
        self.diagnostic = None
        self.diagnostic_time = None
        self.scans = 0
        self.incomplete = 0

    def ingest(self, kind, row, now=None):
        now = time.monotonic() if now is None else now
        if kind == 'lidar_diagnostic':
            if self.diagnostic and ((row['device_ms']-self.diagnostic['device_ms']) & 0xffffffff) >= 0x80000000:
                self.reset()
            self.diagnostic, self.diagnostic_time = dict(row), now
            if row.get('representation') == 'raw_points':self.raw_mode = True
            return
        if kind == 'lidar_raw':
            if not self.raw_mode:
                self.reset();self.raw_mode = True
            self.raw.ingest(row,now)
            self.scan,self.scan_time = self.raw.scan,self.raw.scan_time
            self.scans,self.incomplete = self.raw.scans,self.raw.incomplete
            return
        if kind != 'lidar_scan':
            return
        seq = row['sequence']
        if self.sequence is not None and not 0 < ((seq-self.sequence) & 65535) < 32768:
            return  # Repeated/old complete scans cannot refresh freshness.
        if self.pending and now-self.pending['started'] >= .5:
            self.incomplete += 1
            self.pending = None
        if self.pending and seq != self.pending['sequence']:
            if not 0 < ((seq-self.pending['sequence']) & 65535) < 32768:
                return
            self.incomplete += 1
            self.pending = None
        if self.pending is None:
            self.pending = dict(sequence=seq, started=now, chunks={}, captured=now,
                total=row['chunks'], captured_ms=row.get('captured_ms'),
                resolution=row.get('resolution_degrees', 5))
        pending = self.pending
        if (pending['total'] != row['chunks'] or pending['captured_ms'] != row.get('captured_ms')
                or pending['resolution'] != row.get('resolution_degrees', 5)):
            return
        index = row['chunk']
        if index in pending['chunks']:
            return
        pending['chunks'][index] = row['points']
        pending['captured'] = min(pending['captured'], now-row['source_age_ms']/1000.0)
        if len(pending['chunks']) != pending['total']:
            return
        self.sequence = seq
        points = [point for i in range(pending['total']) for point in pending['chunks'][i]]
        measured = pending['resolution'] == 1
        self.scan = dict(sequence=seq, points=points if measured else [[i*5+2.5,p[0],p[1]] for i,p in enumerate(points)],
                         resolution_degrees=pending['resolution'], representation='nearest_return_per_sector',
                         angle_mode='measured' if measured else 'sector_center', captured_ms=pending['captured_ms'])
        self.scan_time = pending['captured']
        self.scans += 1
        self.pending = None

    def snapshot(self, now=None, enabled=True):
        now = time.monotonic() if now is None else now
        age = max(0, now-self.scan_time) if self.scan_time is not None else None
        diag_age = max(0, now-self.diagnostic_time) if self.diagnostic_time is not None else None
        online = bool(enabled and age is not None and age < .5)
        diag = dict(self.diagnostic, age_ms=diag_age*1000) if self.diagnostic else None
        return dict(source='fpga_m2', enabled=enabled, online=online,
            age_ms=age*1000 if age is not None else None, scan=self.scan if online else None,
            scans=self.scans, incomplete_scans=self.incomplete, diagnostic=diag,
            frames=(diag or {}).get('packets', 0), bad_crc=(diag or {}).get('crc_errors', 0),
            error=None, resolution_degrees=None if self.raw_mode else (self.scan or self.diagnostic or {}).get('resolution_degrees', 5),
            representation='raw_points' if self.raw_mode else 'nearest_return_per_sector',
            missing_raw_packets=self.raw.missing_packets if self.raw_mode else 0,
            stale_raw_packets=self.raw.stale_packets if self.raw_mode else 0)

    def invalidate_raw(self):
        self.raw.invalidate()
        if self.raw_mode:self.incomplete=self.raw.incomplete
