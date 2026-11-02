"""UART commands, UDP status. Construction performs no IO.

Telemetry destination must already point at this machine. No automatic network
reconfiguration and no UART status fallback that could hide wrong firmware.
"""
from pathlib import Path
import socket, sys
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from shared.telemetry_udp import TelemetryUDP


class SplitLink:
    def __init__(self, port, timeout=.02, write_timeout=.2,
                 bind='0.0.0.0', peer='192.168.50.30'):
        self.port, self.timeout, self.write_timeout = port, timeout, write_timeout
        self.bind = bind
        self.decoder = TelemetryUDP(peer)
        self.device = self.udp = None

    def __enter__(self):
        import serial
        try:
            # Bind before opening the command port; fail cleanly if another
            # telemetry receiver owns 6205. Never silently reuse that socket.
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.bind((self.bind,6205))
            self.udp.settimeout(self.timeout)
            self.device = serial.Serial(self.port,115200,timeout=self.timeout,
                                        write_timeout=self.write_timeout)
            return self
        except Exception:
            self.__exit__(None,None,None)
            raise

    def __exit__(self,*args):
        if self.device is not None:
            self.device.close()
        if self.udp is not None:
            self.udp.close()

    def write(self,data):
        return self.device.write(data)

    def read(self,size):
        if size < 48:
            raise ValueError('A complete status frame needs at least 48 bytes')
        if self.decoder.restart_required:
            raise OSError('FPGA restarted; reconnect to bind the new telemetry stream')
        try:
            data,peer = self.udp.recvfrom(2048)
        except socket.timeout:
            return b''
        frame = self.decoder.feed(data,peer)
        return frame or b''
