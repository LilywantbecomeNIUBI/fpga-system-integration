"""Headless IMU v2 parser/statistics compatible with Nano Python 3.6.

Adapted from vendor/imu/pc/imu_viewer.py (frozen source retained).
Only the GUI/CLI imports and Python 3.9 return annotation are omitted.
Source SHA256: eb57728203d1376771ae76aa2343b1f708482d6dcbf3a0068171104f242d725b
"""
import struct

MAGIC = b"\xa5\x5a"
FRAME_SIZE = 48
BODY = struct.Struct("<2sBBII7h4B3HIHHBB")
ERRORS = {0: "none", 1: "I2C_NACK", 2: "I2C_TIMEOUT", 3: "BUS_STUCK",
          4: "BAD_LENGTH", 0x10: "WHO_AM_I", 0x11: "CONFIG_READBACK",
          0x12: "RESET_NOT_DONE", 0x13: "DATA_STALE", 0x1F: "FSM"}
FIELDS = ["host_time", "type", "seq", "timestamp_us", "ax_raw", "ay_raw", "az_raw",
          "gx_raw", "gy_raw", "gz_raw", "temp_raw", "ax_g", "ay_g", "az_g",
          "gx_dps", "gy_dps", "gz_dps", "temp_c", "ready", "fault", "who_am_i",
          "last_error", "config_id", "error_count", "invalid_count", "tx_drop_count",
          "raw_seq", "stale", "temperature_valid", "fast_bus", "low_latency_filter",
          "observe_to_commit_us", "raw_interval_us", "decimation", "poll_us"]


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = self.discarded = self.unsupported = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self.buffer.endswith(MAGIC[:1]) else 0
                self.discarded += len(self.buffer) - keep
                del self.buffer[:len(self.buffer) - keep]
                break
            if start:
                self.discarded += start
                del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break
            packet = bytes(self.buffer[:FRAME_SIZE])
            if crc16(packet[:46]) != int.from_bytes(packet[46:], "big"):
                self.bad_crc += 1
                self.discarded += 1
                del self.buffer[0]
                continue
            del self.buffer[:FRAME_SIZE]
            values = BODY.unpack(packet[:46])
            _, version, kind, raw_seq, stamp, ax, ay, az, gx, gy, gz, temp, flags, who, err, config, errors, invalid, dropped, debug_seq, latency, interval, decimation, poll_us = values
            if version != 2 or kind not in (1, 2) or config not in (2, 3) or not decimation:
                self.unsupported += 1
                continue
            frames.append(dict(type="sample" if kind == 1 else "status", seq=debug_seq, raw_seq=raw_seq, timestamp_us=stamp,
                ax_raw=ax, ay_raw=ay, az_raw=az, gx_raw=gx, gy_raw=gy, gz_raw=gz, temp_raw=temp,
                ax_g=ax / 8192, ay_g=ay / 8192, az_g=az / 8192,
                gx_dps=gx * 1000 / 32768, gy_dps=gy * 1000 / 32768, gz_dps=gz * 1000 / 32768,
                temp_c=temp / 132.48 + 25, ready=bool(flags & 1), fault=bool(flags & 2),
                who_am_i=who, last_error=err, config_id=config, error_count=errors,
                invalid_count=invalid, tx_drop_count=dropped, stale=bool(flags & 4),
                temperature_valid=bool(flags & 8), fast_bus=bool(flags & 16), low_latency_filter=bool(flags & 32),
                observe_to_commit_us=latency, raw_interval_us=interval, decimation=decimation, poll_us=poll_us))
        return frames


class Statistics:
    def __init__(self):
        self.samples = self.status = self.gaps = self.resets = self.duplicates = 0
        self.previous = None
        self.intervals = []
        self.last = None
        self.raw_intervals = []
        self.latencies = []
        self.raw_stride_anomalies = 0

    def add(self, frame):
        self.last = frame
        if frame["type"] == "status":
            self.status += 1
            return
        self.samples += 1
        self.latencies.append(frame["observe_to_commit_us"])
        if len(self.latencies) > 10000:
            del self.latencies[:1000]
        if self.previous is not None:
            delta = (frame["seq"] - self.previous["seq"]) & 0xFFFFFFFF
            dt = (frame["timestamp_us"] - self.previous["timestamp_us"]) & 0xFFFFFFFF
            if delta == 0:
                self.duplicates += 1
            elif delta >= 0x80000000:
                self.resets += 1
            else:
                self.gaps += delta - 1
                if delta == 1 and 0 < dt < 100000:
                    self.intervals.append(dt)
                    if len(self.intervals) > 10000:
                        del self.intervals[:1000]
                raw_delta = (frame["raw_seq"] - self.previous["raw_seq"]) & 0xFFFFFFFF
                if raw_delta != delta * frame["decimation"]:
                    self.raw_stride_anomalies += 1
                if delta == 1 and 0 < dt < 100000 and 0 < raw_delta < 100:
                    self.raw_intervals.append((raw_delta, dt))
                    if len(self.raw_intervals) > 10000:
                        del self.raw_intervals[:1000]
        self.previous = frame

    def report(self, parser):
        mean = sum(self.intervals) / len(self.intervals) if self.intervals else 0
        raw_rate = (sum(x[0] for x in self.raw_intervals) * 1e6 / sum(x[1] for x in self.raw_intervals)) if self.raw_intervals else None
        latency = sorted(self.latencies)
        return dict(samples=self.samples, status_frames=self.status, serial_sequence_gaps=self.gaps,
            sequence_resets=self.resets, duplicate_sequences=self.duplicates,
            crc_failures=parser.bad_crc, discarded_bytes=parser.discarded,
            unsupported_frames=parser.unsupported, trailing_bytes=len(parser.buffer),
            observed_rate_hz=1000000 / mean if mean else None, debug_rate_hz=1000000 / mean if mean else None,
            raw_read_rate_hz=raw_rate, raw_stride_anomalies=self.raw_stride_anomalies,
            observe_to_commit_us=dict(mean=sum(latency)/len(latency), maximum=latency[-1],
                p99=latency[min(len(latency)-1,int(len(latency)*.99))]) if latency else None,
            last_frame=self.last,
            note="seq is debug-frame sequence; raw_seq counts successful FPGA reads. Deliberate decimation is not packet loss. Observation-to-commit is not total sensor latency.")
