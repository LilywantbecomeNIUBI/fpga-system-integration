"""Dedicated UDP observer process; never opens a command connection (Python 3.6+)."""
import multiprocessing
import queue
import socket
import threading
import time


def receive(config, stop, output):
    from nano.web_console import ViewerGateway
    config = dict(config, telemetry_process=False, record_telemetry=False)
    gateway = ViewerGateway(config, live=True)
    gateway.control.connect()
    gateway.control.reason = 'receive_only'
    published = 0.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
        stream.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
        stream.bind((config.get('telemetry_bind', '0.0.0.0'), config.get('telemetry_port', 6205)))
        stream.settimeout(.05)
        while not stop.is_set():
            try:
                data, peer = stream.recvfrom(2048)
                gateway.ingest_udp(data, peer)
                gateway.udp_error = ('FPGA复位：请重连网关以绑定新遥测序列'
                                     if gateway.udp_telemetry.restart_required else None)
            except socket.timeout:
                pass
            now = time.monotonic()
            if now-published >= .1:
                state = gateway.state()
                try:
                    output.put_nowait((now, state))
                except queue.Full:
                    pass
                published = now


class TelemetryReceiver:
    def __init__(self, config):
        context = multiprocessing.get_context('spawn')
        self.stop = context.Event()
        self.output = context.Queue(1)
        self.process = context.Process(target=receive, args=(config, self.stop, self.output), daemon=True)
        self.lock = threading.Lock()
        self.latest = None
        self.reader = threading.Thread(target=self.read, daemon=True)

    def start(self):
        self.process.start()
        self.reader.start()

    def read(self):
        while not self.stop.is_set():
            try:
                value = self.output.get(timeout=.1)
            except queue.Empty:
                continue
            with self.lock:
                self.latest = value

    def state(self):
        with self.lock:
            latest = self.latest
        if latest is None:
            return None
        sent, value = latest
        # Completed scans are immutable. Copy only the status dictionaries whose
        # ages/freshness change, rather than thousands of point lists per poll.
        state = dict(value)
        for key in ('lidar', 'telemetry', 'imu', 'control'):
            state[key] = dict(value[key])
        if state['lidar'].get('diagnostic'):
            state['lidar']['diagnostic'] = dict(state['lidar']['diagnostic'])
        state['imu']['attitude'] = dict(state['imu']['attitude'])
        elapsed = max(0, time.monotonic()-sent)*1000
        lidar = state['lidar']
        for row in (lidar, lidar.get('diagnostic'), state.get('telemetry'), state.get('imu')):
            if row and row.get('age_ms') is not None:
                row['age_ms'] += elapsed
        if lidar.get('age_ms') is None or lidar['age_ms'] >= 500:
            lidar.update(online=False, scan=None)
        if elapsed >= 500 or not self.process.is_alive():
            lidar.update(online=False, scan=None)
            state['imu']['online'] = False
            state['imu']['attitude']['valid'] = False
            state['control']['motor_fresh'] = False
            state['telemetry_error'] = 'Telemetry receiver unavailable or expired'
        state['telemetry']['receiver_process'] = True
        state['telemetry']['receiver_pid'] = self.process.pid
        state['telemetry']['snapshot_age_ms'] = elapsed
        return state

    def close(self):
        self.stop.set()
        self.process.join(2)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(2)
        self.reader.join(1)
        self.output.close()
