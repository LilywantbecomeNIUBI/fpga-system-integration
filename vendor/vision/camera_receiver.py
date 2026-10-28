"""Receive ACG720 camera UDP rows: <H row, followed by RGB565 LE pixels.

Run: python camera_receiver.py --bind 0.0.0.0 --port 6102
No camera -> no image is fabricated. --demo explicitly renders a synthetic fixture.
"""
from __future__ import annotations
import struct
import numpy as np


def rgb565_to_bgr(pixels: np.ndarray) -> np.ndarray:
    p = pixels.astype(np.uint16, copy=False)
    r = ((p >> 11) * 255 // 31).astype(np.uint8)
    g = (((p >> 5) & 63) * 255 // 63).astype(np.uint8)
    b = ((p & 31) * 255 // 31).astype(np.uint8)
    return np.stack((b, g, r), axis=-1)


class RowAssembler:
    def __init__(self, width=640, height=480):
        self.width, self.height = width, height
        self.pixels = np.zeros((height, width), dtype=np.uint16)
        self.seen = np.zeros(height, dtype=bool)
        self.started = False
        self.packets = self.frames = self.bad = self.duplicates = self.incomplete = 0

    def push(self, packet: bytes):
        if len(packet) != 2 + self.width * 2:
            self.bad += 1
            return None
        row = struct.unpack_from('<H', packet)[0]
        if row >= self.height:
            self.bad += 1
            return None
        self.packets += 1
        if row == 0:
            # Protocol has no frame ID. Row zero is the only frame boundary.
            if self.started and self.seen[1:].any():
                self.incomplete += 1
                self.seen[:] = False
                self.pixels[:] = 0
            if not self.started:
                self.started = True
                self.seen[:] = False
        if not self.started:
            return None
        if self.seen[row]:
            self.duplicates += 1
        self.pixels[row] = np.frombuffer(packet, dtype='<u2', offset=2, count=self.width)
        self.seen[row] = True
        if self.seen.all():
            self.frames += 1
            self.started = False
            result = rgb565_to_bgr(self.pixels)
            self.seen[:] = False
            return result
        return None


def synthetic_rows(width, height):
    palette = np.array([0xffff, 0xffe0, 0x07ff, 0x07e0, 0xf81f, 0xf800, 0x001f, 0], dtype='<u2')
    line = palette[np.minimum(np.arange(width) * 8 // width, 7)]
    for row in range(height):
        yield struct.pack('<H', row) + line.astype('<u2').tobytes()


def main():
    from camera_viewer import main as run_viewer
    run_viewer()


if __name__ == "__main__":
    main()
