"""Read-only M2 display protocol; six chunks form one complete 72-bin scan.

Compatible with Nano Python 3.6. Display scans never feed motor control.
"""
import struct
import time


def decode_lidar(frame):
    # The mixed UART decoder checks the CRC before dispatch.
    if frame[:4] == b'\xc3\x3c\x05\x01':
        sequence, chunk, chunks, age = struct.unpack_from('<HBBH', frame, 4)
        if chunks != 6 or chunk >= chunks or age >= 400:
            raise ValueError('Invalid M2 scan chunk')
        return 'lidar_scan', dict(sequence=sequence, chunk=chunk, chunks=chunks,
            source_age_ms=age, points=[list(struct.unpack_from('<HB', frame, 10+i*3)) for i in range(12)])
    if frame[:4] == b'\xc3\x3c\x06\x01':
        names = ('device_ms', 'bytes', 'framing_errors', 'packets', 'crc_errors',
                 'format_errors', 'timeouts', 'completed_scans', 'skipped_scans')
        row = dict(zip(names, struct.unpack_from('<9I', frame, 8)))
        row.update(sequence=struct.unpack_from('<H', frame, 4)[0], flags=frame[6],
                   coverage=struct.unpack_from('<H', frame, 44)[0])
        if frame[7] or row['coverage'] > 72 or row['flags'] & ~31:
            raise ValueError('Invalid M2 receive diagnostic')
        return 'lidar_diagnostic', row
    raise ValueError('Unknown M2 LiDAR display version')


class FpgaLidar:
    def __init__(self):
        self.reset()

    def reset(self):
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
            self.pending = dict(sequence=seq, started=now, chunks={}, captured=now)
        pending = self.pending
        index = row['chunk']
        if index in pending['chunks']:
            return
        pending['chunks'][index] = row['points']
        pending['captured'] = min(pending['captured'], now-row['source_age_ms']/1000.0)
        if len(pending['chunks']) != 6:
            return
        self.sequence = seq
        points = [point for i in range(6) for point in pending['chunks'][i]]
        self.scan = dict(sequence=seq, points=[[i*5+2.5, p[0], p[1]] for i,p in enumerate(points)],
                         resolution_degrees=5, representation='nearest_return_per_sector')
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
            error=None, resolution_degrees=5)
