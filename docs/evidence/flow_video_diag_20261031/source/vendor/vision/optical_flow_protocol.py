"""Optical-flow diagnostic packets. Python 3.6+, standard library only."""
import struct

PORT = 6202
PACKET = struct.Struct('!4sBBBB8HII')
STATES = ('unknown', 'forward', 'left', 'right', 'brake')


def signed16(value):
    return value-65536 if value & 32768 else value


def parse_packet(payload):
    if len(payload) != 32:
        raise ValueError('optical-flow packet must contain 32 bytes')
    magic, version, stage, flags, state, *fields = PACKET.unpack(payload)
    if magic not in (b'OFLW', b'OFST') or version != 1 or stage > 2:
        raise ValueError('invalid optical-flow header')
    if magic == b'OFLW':
        fid, x, y, u, v, seq, width, height, cycles, drops = fields
        if flags & ~1 or state != 0 or width != 160 or height != 90 or x >= width or y >= height or seq >= 190:
            raise ValueError('invalid flow geometry/flags/sequence')
        return dict(kind='flow', stage=stage, frame_id=fid, valid=bool(flags & 1),
                    x=x, y=y, u=signed16(u)/128., v=signed16(v)/128., sequence=seq,
                    width=width, height=height, cycles=cycles, dropped_frames=drops)
    if flags & ~63 or state >= len(STATES):
        raise ValueError('invalid avoidance flags/state')
    fid, nl, nc, nr, rl, rc, rr, control, cycles, drops = fields
    steering = control & 255
    steering = steering-256 if steering & 128 else steering
    if steering not in (-1, 0, 1) or any(r > n for n, r in zip((nl, nc, nr), (rl, rc, rr))):
        raise ValueError('invalid avoidance counts/control')
    stop, confident, stale = bool(flags & 1), bool(flags & 2), bool(flags & 32)
    if (stale or not confident) and (not stop or state != 0 or control != 0):
        raise ValueError('invalid unavailable-control mask')
    return dict(kind='status', stage=stage, frame_id=fid, state=STATES[state],
                stop_request=stop, confident=confident, stale=stale, hazard=(flags >> 2) & 7,
                valid_counts=[nl, nc, nr], risk_counts=[rl, rc, rr], steer=steering,
                speed_limit=control >> 8, cycles=cycles, dropped_frames=drops)
