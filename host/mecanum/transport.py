"""Router UDP commands (or legacy UART), UDP status. Construction performs no IO.

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


class RouterLink(SplitLink):
    """One V3 command per UDP6206 datagram, status on existing UDP6205.

    Opening a link only binds sockets. It never reconfigures endpoints or arms.
    No retransmission: state acknowledgments and watchdog remain authoritative.
    """
    def __init__(self, port=None, timeout=.02, write_timeout=.2,
                 bind='0.0.0.0', peer='192.168.50.30', command_port=6206):
        import ipaddress
        peer=str(ipaddress.IPv4Address(peer))
        if not 1 <= command_port <= 65535:
            raise ValueError('Invalid command UDP port')
        super().__init__(port,timeout,write_timeout,bind,peer)
        self.peer,self.command_port=peer,command_port

    def __enter__(self):
        try:
            self.udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.udp.bind((self.bind,6205))
            self.udp.settimeout(self.timeout)
            self.device=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.device.bind((self.bind,0))
            self.device.connect((self.peer,self.command_port))
            self.device.settimeout(self.write_timeout)
            return self
        except Exception:
            self.__exit__(None,None,None)
            raise

    def write(self,data):
        if len(data)!=24:
            raise ValueError('One complete 24-byte V3 command is required')
        return self.device.send(data)
