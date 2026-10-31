"""V1 FPGA semantic feedback protocol; standard library only, Python 3.6+."""
import struct

DETECTION = struct.Struct('!4sBB11HI')
STATUS = struct.Struct('!4sBBBB8HII')
PORT = 6201

def detection(frame_id, bbox=None, class_id=39, confidence=900,
              width=1280, height=720, inference_ms=0):
    """bbox is x1,y1,x2,y2 in the original JPEG image, with exclusive x2/y2."""
    if not 0 <= frame_id <= 65535:
        raise ValueError('frame_id must fit the existing uint16 JPEG frame ID')
    if not 1 <= width <= 65535 or not 1 <= height <= 65535:
        raise ValueError('invalid image geometry')
    if bbox is None:
        fields = [0] * 7
    else:
        x1, y1, x2, y2 = bbox
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError('bbox must be inside the original image')
        if not 0 <= class_id <= 65535 or not 0 <= confidence <= 1000 or not 0 <= inference_ms <= 65535:
            raise ValueError('invalid class/confidence/inference time')
        fields = [class_id, confidence, x1, y1, x2, y2, inference_ms]
    return DETECTION.pack(b'VDET', 1, int(bbox is not None), frame_id, width, height,
                          *(fields + [0, 0]))

def query():
    return b'VQRY' + bytes(28)

def parse_status(payload):
    if len(payload) != STATUS.size:
        raise ValueError('status must contain 32 bytes')
    values = STATUS.unpack(payload)
    magic, version, flags, reason, direction = values[:5]
    if magic != b'VSTA' or version != 1 or flags & 0xf0 or direction > 3 or values[12] != 0:
        raise ValueError('invalid VSTA header/reserved field')
    keys = ('result_frame', 'current_frame', 'class_id', 'confidence', 'center_x', 'center_y',
            'age_frames', 'reserved', 'accepted', 'rejected')
    result = dict(zip(keys, values[5:]))
    result.update(target_valid=bool(flags & 1), frame_seen=bool(flags & 2),
                  sequence_seen=bool(flags & 4), stream_ready=bool(flags & 8),
                  reason=reason, direction=('none', 'left', 'center', 'right')[direction])
    result.pop('reserved')
    return result
