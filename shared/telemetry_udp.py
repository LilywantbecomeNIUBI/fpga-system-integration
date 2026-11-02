"""TLM1 fixed-size UDP telemetry. No sockets or control commands in this codec."""
import struct
import time
from host.motor_protocol import crc16

HEADER = struct.Struct('>4sB3sII')


class TelemetryUDP:
    def __init__(self, peer_ip='192.168.50.30', peer_port=6205):
        self.peer = (peer_ip, peer_port)
        self.sequence = None
        self.device_ms = None
        self.elapsed_ms = 0
        self.origin = None
        self.accepted = self.rejected = self.duplicates = self.missing = self.stale = 0
        self.restart_required = False
        self.received = None
        self.frame_age = 0
        self.clock_update_time = None

    def feed(self, data, peer, now=None):
        now = time.monotonic() if now is None else now
        if tuple(peer) != self.peer or len(data) != 64:
            self.rejected += 1
            return None
        magic, version, reserved, sequence, device_ms = HEADER.unpack_from(data)
        frame = data[16:]
        motor = frame[:2] == b'\xc3\x3c'
        mecanum = frame[:3] in (b'M4\x03', b'P4\x03')
        if (magic != b'TLM1' or version != 1 or reserved != b'\x00'*3
                or not (motor and frame[2] in range(1,8) or mecanum or frame[:3] == b'\xa5\x5a\x02')
                or crc16(frame[:46]) != struct.unpack('<H' if motor or mecanum else '>H', frame[46:])[0]):
            self.rejected += 1
            return None
        delta = 1 if self.sequence is None else (sequence-self.sequence) & 0xffffffff
        tick = 0 if self.device_ms is None else (device_ms-self.device_ms) & 0xffffffff
        if not 0 < delta < 0x80000000:
            if (self.received is not None and now-self.received >= .4 and sequence < 1024
                    and tick >= 0x80000000):self.restart_required = True
            self.duplicates += 1
            return None
        if self.device_ms is not None and tick >= 0x80000000:
            # A firmware reset cannot silently renew an existing control lease.
            # Restart/reconnect the gateway to explicitly bind the new stream.
            self.restart_required = True
            self.rejected += 1
            return None
        self.elapsed_ms = device_ms if self.device_ms is None else self.elapsed_ms+tick
        # Track small oscillator drift without allowing a queueing delay to
        # instantly redefine old telemetry as fresh. Budget at most 200ppm,
        # charged once per advancing packet, including rejected stale packets.
        candidate = now-self.elapsed_ms/1000.0
        drift = 0 if self.clock_update_time is None else max(0,min(1,now-self.clock_update_time))*.0002
        self.origin = candidate if self.origin is None else min(self.origin+drift,candidate)
        self.clock_update_time = now
        self.sequence, self.device_ms = sequence, device_ms
        self.missing += delta-1
        self.frame_age = max(0, now-self.origin-self.elapsed_ms/1000.0)
        if self.frame_age >= .4:
            self.stale += 1
            return None
        self.accepted += 1
        self.received = now
        return frame

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        return dict(transport='udp', peer=list(self.peer), accepted=self.accepted,
                    rejected=self.rejected, duplicates=self.duplicates, missing=self.missing,
                    stale=self.stale, restart_required=self.restart_required, source_age_ms=self.frame_age*1000,
                    age_ms=(now-self.received)*1000 if self.received is not None else None)
