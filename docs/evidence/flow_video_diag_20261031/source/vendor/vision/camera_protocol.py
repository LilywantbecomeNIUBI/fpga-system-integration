"""Camera v2 packet assembly. Frame IDs prevent blending different exposures.

Header: <2sBBHHHHHH (CV, 2, RGB565=1, frame, row, x, count, width, height).
720 pixels per segment; total IPv4 packet <= 1484 bytes, no IP fragmentation.
"""
import struct
import time
import numpy as np

HEADER = struct.Struct('<2sBBHHHHHH')
SEGMENT_PIXELS = 720


def newer(a, b):
    return 0 < ((a-b) & 65535) < 32768


class SegmentAssembler:
    def __init__(self, raw=False):
        self.raw = raw
        self.width = self.height = 0
        self.packets = self.frames = self.bad = self.duplicates = self.incomplete = 0
        self.stale = self.missing_frames = 0
        self.last_completed = None
        self.newest = None
        self.pending = {}

    def push(self, packet):
        if len(packet) < HEADER.size:
            self.bad += 1
            return None
        magic, version, fmt, fid, row, x, count, width, height = HEADER.unpack_from(packet)
        if (magic != b'CV' or version != 2 or fmt != 1 or
                not 1 <= width <= 2592 or not 1 <= height <= 1944 or
                row >= height or x >= width or x % SEGMENT_PIXELS or
                count != min(SEGMENT_PIXELS, width-x) or len(packet) != 16+count*2):
            self.bad += 1
            return None
        if self.width and (width, height) != (self.width, self.height):
            self.bad += 1
            return None
        self.width, self.height = width, height
        self.packets += 1
        if self.last_completed is not None and not newer(fid, self.last_completed):
            self.stale += 1
            return None
        if self.newest is None or newer(fid, self.newest):
            self.newest = fid
            for old in list(self.pending):
                if ((fid-old) & 65535) >= 2:
                    del self.pending[old]
                    self.incomplete += 1
        elif ((self.newest-fid) & 65535) >= 2:
            self.stale += 1
            return None
        chunks = (width+SEGMENT_PIXELS-1)//SEGMENT_PIXELS
        if fid not in self.pending:
            self.pending[fid] = [np.empty((height, width), dtype='<u2'),
                                 np.zeros(height*chunks, dtype=bool), 0]
        pixels, seen, received = self.pending[fid]
        index = row*chunks+x//SEGMENT_PIXELS
        if seen[index]:
            self.duplicates += 1
            return None
        pixels[row, x:x+count] = np.frombuffer(packet, dtype='<u2', offset=16)
        seen[index] = True
        received += 1
        self.pending[fid][2] = received
        if received != height*chunks:
            return None
        self.frames += 1
        if self.last_completed is not None:
            self.missing_frames += ((fid-self.last_completed) & 65535)-1
        self.last_completed = fid
        del self.pending[fid]
        for old in list(self.pending):
            if not newer(old, fid):
                del self.pending[old]
                self.incomplete += 1
        if self.raw:
            return pixels
        from camera_receiver import rgb565_to_bgr
        return rgb565_to_bgr(pixels)


class AutoAssembler:
    def __init__(self, width=640, height=480, raw=False):
        self.legacy_width, self.legacy_height = width, height
        self.raw = raw
        self.impl = None
        self.last_arrival = None
        self.reset_events = 0

    def push(self, packet):
        now = time.monotonic()
        # FPGA reset restarts the 16-bit frame counter. A small frame ID after
        # a real silence, outside ordinary modulo wraparound, opens a new epoch.
        if (self.last_arrival is not None and now-self.last_arrival > .5 and
                isinstance(self.impl, SegmentAssembler) and self.impl.last_completed is not None and
                packet[:4] in (b'CV\x02\x01', b'CV\x02\x02') and len(packet) >= 16):
            fid = struct.unpack_from('<H', packet, 4)[0]
            if fid <= 32 and not newer(fid, self.impl.last_completed):
                self.impl.incomplete += len(self.impl.pending)
                self.impl.pending.clear()
                self.impl.newest = self.impl.last_completed = None
                self.reset_events += 1
        self.last_arrival = now
        if self.impl is None:
            if packet[:4] == b'CV\x02\x02':
                self.impl = JpegAssembler(raw=self.raw)
            elif packet[:4] == b'CV\x02\x01':
                self.impl = SegmentAssembler(raw=self.raw)
            else:
                from camera_receiver import RowAssembler
                self.impl = RowAssembler(self.legacy_width, self.legacy_height)
        return self.impl.push(packet)

    def __getattr__(self, name):
        if self.impl is None:
            return 0
        return getattr(self.impl, name, 0)


class JpegAssembler(SegmentAssembler):
    def push(self, packet):
        if len(packet) < 16:
            self.bad += 1
            return None
        magic, version, fmt, fid, lo, hi, flags, width, height = HEADER.unpack_from(packet)
        count, last = flags & 32767, bool(flags & 32768)
        offset = lo | (hi << 16)
        if (magic != b'CV' or version != 2 or fmt != 2 or
                not 1 <= width <= 2592 or not 1 <= height <= 1944 or
                not 1 <= count <= 1440 or offset % 1440 or offset >= 8*1024*1024 or
                not last and count != 1440 or len(packet) != 16+count):
            self.bad += 1
            return None
        if self.width and (width, height) != (self.width, self.height):
            self.bad += 1
            return None
        self.width, self.height = width, height
        self.packets += 1
        if self.last_completed is not None and not newer(fid, self.last_completed):
            self.stale += 1
            return None
        if self.newest is None or newer(fid, self.newest):
            self.newest = fid
            for old in list(self.pending):
                if ((fid-old) & 65535) >= 2:
                    del self.pending[old]
                    self.incomplete += 1
        elif ((self.newest-fid) & 65535) >= 2:
            self.stale += 1
            return None
        segments, total = self.pending.setdefault(fid, [{}, None])
        if offset in segments:
            self.duplicates += 1
            return None
        segments[offset] = packet[16:]
        if last:
            if total is not None and total != offset+count:
                self.bad += 1
                return None
            total = offset+count
            self.pending[fid][1] = total
        if total is None or len(segments) != (total+1439)//1440:
            return None
        if any(k not in segments for k in range(0, total, 1440)):
            return None
        encoded = b''.join(segments[k] for k in range(0, total, 1440))
        del self.pending[fid]
        if len(encoded) != total or encoded[:2] != b'\xff\xd8' or encoded[-2:] != b'\xff\xd9':
            self.bad += 1
            return None
        self.frames += 1
        if self.last_completed is not None:
            self.missing_frames += ((fid-self.last_completed) & 65535)-1
        self.last_completed = fid
        for old in list(self.pending):
            if not newer(old, fid):
                del self.pending[old]
                self.incomplete += 1
        if self.raw:
            return encoded
        import cv2
        decoded = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None or decoded.shape[:2] != (height, width):
            self.bad += 1
            return None
        return decoded
