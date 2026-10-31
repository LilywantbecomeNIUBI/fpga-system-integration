"""ICM42688 1kHz acquisition / 200Hz debug viewer; protocol v2, 115200 8N1.

Offline replay and parser tests need only Python's standard library.
Live serial needs pyserial==3.5, loaded from ../.deps when installed there.
"""
from __future__ import annotations

import argparse
import csv
import json
import struct
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".deps"))
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

    def feed(self, data: bytes) -> list[dict]:
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


class LiveWindow:
    def __init__(self):
        import tkinter as tk
        self.tk = tk
        self.root = tk.Tk()
        self.root.title("ICM42688 · 1kHz 六轴采集 / 200Hz 调试 · 115200 baud")
        self.root.geometry("960x690")
        self.root.configure(bg="#101827")
        self.closed = False
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.title = tk.Label(self.root, text="等待 IMU 数据…", bg="#101827", fg="#e7edf8", font=("Microsoft YaHei", 15))
        self.title.pack(pady=14)
        self.values = tk.Label(self.root, text="", bg="#101827", fg="#e7edf8", font=("Consolas", 15), justify="left")
        self.values.pack(pady=6)
        self.status = tk.Label(self.root, text="", bg="#101827", fg="#94a3b8", font=("Consolas", 10))
        self.status.pack(pady=5)
        self.canvas = tk.Canvas(self.root, bg="#101827", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=20, pady=10)
        self.history = deque(maxlen=240)
        self.last_plot = 0.0

    def close(self):
        self.closed = True
        self.root.destroy()

    def poll(self):
        if not self.closed:
            self.root.update()
        return not self.closed

    def link_stale(self):
        if not self.closed:
            self.title.config(text="串口超过 2.5 秒未收到有效数据", fg="#f8bd66")

    def show(self, frame, stats, parser):
        if self.closed:
            return
        active = frame["ready"] and not frame["fault"] and not frame["stale"] and frame["type"] == "sample"
        detail = ERRORS.get(frame["last_error"], hex(frame["last_error"])) if frame["last_error"] else "等待初始化"
        self.title.config(text="正在采集六轴数据" if active else "IMU 状态：" + detail,
                          fg="#68e0b0" if active else "#f8bd66")
        if frame["type"] == "sample":
            self.values.config(text=(f"Accel  X {frame['ax_g']:+8.4f}   Y {frame['ay_g']:+8.4f}   Z {frame['az_g']:+8.4f}  g\n"
                                     f"Gyro   X {frame['gx_dps']:+8.2f}   Y {frame['gy_dps']:+8.2f}   Z {frame['gz_dps']:+8.2f}  deg/s\n"
                                     f"Temp   {frame['temp_c']:.2f} C       Raw {frame['raw_seq']}   Debug {frame['seq']}\n"
                                     f"I2C read {frame['observe_to_commit_us']} us      Raw interval {frame['raw_interval_us']} us"))
        self.status.config(text=f"WHO=0x{frame['who_am_i']:02X}   Ready={int(frame['ready'])}   Fault={int(frame['fault'])}   "
                                f"I2C errors={frame['error_count']}   Invalid={frame['invalid_count']}   "
                                f"TX drop={frame['tx_drop_count']}   Stale={int(frame['stale'])}   CRC={parser.bad_crc}")
        current = time.monotonic()
        if current - self.last_plot < 0.04 or not active:
            return
        self.last_plot = current
        self.history.append(tuple(frame[key] for key in ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")))
        self.canvas.delete("all")
        width, height = max(200, self.canvas.winfo_width()), max(160, self.canvas.winfo_height())
        colors = ("#56b4ff", "#68e0b0", "#f8bd66")
        for group, label in enumerate(("Acceleration / g", "Angular rate / deg/s")):
            top, bottom = group * height / 2 + 28, (group + 1) * height / 2 - 18
            center = (top + bottom) / 2
            values = [abs(row[group * 3 + axis]) for row in self.history for axis in range(3)]
            limit = max(1.25 if group == 0 else 5.0, max(values, default=1) * 1.2)
            self.canvas.create_text(8, top - 14, text=f"{label}   range ±{limit:.2f}", anchor="w", fill="#94a3b8")
            self.canvas.create_line(0, center, width, center, fill="#334155")
            for axis, color in enumerate(colors):
                points = []
                for i, row in enumerate(self.history):
                    points.extend((i * width / 239, center - row[group * 3 + axis] / limit * (bottom - top) / 2))
                if len(points) >= 4:
                    self.canvas.create_line(*points, fill=color, width=2)
                self.canvas.create_text(width - 100 + axis * 35, top - 14, text="XYZ"[axis], fill=color)


def serial_module():
    try:
        import serial
        import serial.tools.list_ports
        return serial
    except ImportError as exc:
        raise RuntimeError("请先运行 scripts/install_serial.ps1 安装工程本地 pyserial 依赖") from exc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    source = ap.add_mutually_exclusive_group()
    source.add_argument("--port", help="串口，例如 COM12")
    source.add_argument("--input", type=Path, help="离线解析 UART 原始字节文件")
    ap.add_argument("--list", action="store_true", help="仅列出串口")
    ap.add_argument("--gui", action="store_true", help="显示六轴数值与实时曲线")
    ap.add_argument("--duration", type=float, default=0, help="采集秒数；0 表示直到关闭窗口或 Ctrl+C")
    ap.add_argument("--out", type=Path, help="日志目录，默认 results/capture_时间")
    args = ap.parse_args(argv)
    if args.duration < 0:
        ap.error("--duration must be nonnegative")
    if args.list:
        serial = serial_module()
        for port in serial.tools.list_ports.comports():
            print(f"{port.device}: {port.description}")
        return 0
    connection = None
    if args.input is None:
        serial = serial_module()
        if not args.port:
            ports = list(serial.tools.list_ports.comports())
            if len(ports) != 1:
                ap.error("请用 --list 查看串口，再通过 --port COM编号 指定 FPGA 串口")
            args.port = ports[0].device
        connection = serial.Serial(port=None, baudrate=115200, timeout=0.05)
        connection.dtr = False
        connection.rts = False
        connection.port = args.port
        connection.open()
    window = None
    out = args.out or ROOT / "results" / datetime.now().strftime("capture_%Y%m%d_%H%M%S_%f")
    if args.input and args.input.resolve() == (out / "uart.bin").resolve():
        ap.error("--out 不能覆盖正在回放的 uart.bin，请使用新的输出目录")
    parser, stats = Parser(), Statistics()
    started, last_display = time.monotonic(), 0.0
    last_good, capture_error = started, None
    try:
        if args.gui:
            window = LiveWindow()
        out.mkdir(parents=True, exist_ok=True)
        with (args.input.open("rb") if args.input else connection) as source_stream, \
             (out / "uart.bin").open("wb") as raw, (out / "samples.csv").open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
            writer.writeheader()
            print(f"Logging to {out}", flush=True)
            try:
                while True:
                    if window and not window.poll():
                        break
                    if args.duration and time.monotonic() - started >= args.duration:
                        break
                    chunk = source_stream.read(4096 if args.input else max(1, min(4096, source_stream.in_waiting)))
                    if not chunk:
                        if args.input:
                            break
                        if window and time.monotonic() - last_good > 2.5:
                            window.link_stale()
                        continue
                    raw.write(chunk)
                    frames = parser.feed(chunk)
                    if frames:
                        last_good = time.monotonic()
                    elif window and time.monotonic() - last_good > 2.5:
                        window.link_stale()
                    for frame in frames:
                        stats.add(frame)
                        writer.writerow(dict(host_time=datetime.now().astimezone().isoformat(), **frame))
                    if frames and time.monotonic() - last_display >= 0.2:
                        latest = frames[-1]
                        last_display = time.monotonic()
                        print(f"{latest['type']:6} raw={latest['raw_seq']:8} debug={latest['seq']:7} read={latest['observe_to_commit_us']}us "
                              f"a=({latest['ax_g']:+.3f},{latest['ay_g']:+.3f},{latest['az_g']:+.3f})g "
                              f"w=({latest['gx_dps']:+.2f},{latest['gy_dps']:+.2f},{latest['gz_dps']:+.2f})dps "
                              f"ready={int(latest['ready'])} fault={int(latest['fault'])} "
                              f"error={ERRORS.get(latest['last_error'],hex(latest['last_error']))}", flush=True)
                    if window and frames:
                        window.show(frames[-1], stats, parser)
            except KeyboardInterrupt:
                pass
            except OSError as exc:
                capture_error = str(exc)
                print(f"Serial capture stopped: {exc}", file=sys.stderr)
        report = stats.report(parser)
        report.update(port=args.port, baud=115200, elapsed_s=time.monotonic() - started,
                      hardware_acceptance="not_evaluated", capture_error=capture_error,
                      input=str(args.input) if args.input else None)
        (out / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1 if capture_error else (0 if stats.samples else 2)
    finally:
        if connection and connection.is_open:
            connection.close()
        if window and not window.closed:
            window.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as exc:
        print(f"IMU viewer: {exc}", file=sys.stderr)
        raise SystemExit(1)
