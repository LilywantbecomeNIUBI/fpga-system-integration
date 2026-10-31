"""Loopback burst, sticky fault, silence expiry and process shutdown checks."""
from pathlib import Path
import sys,socket,time,struct,json
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'vendor/vision')]
from shared.perception_process import PerceptionProcess
from optical_flow_protocol import PACKET

def main():
    sink=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sink.bind(('127.0.0.1',0));sink.settimeout(2)
    consumer=PerceptionProcess('127.0.0.1','127.0.0.2',relay=sink.getsockname())
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sender.bind(('127.0.0.2',6202))
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if consumer.snapshot()['stats']:break
            time.sleep(.02)
        else:raise AssertionError('Receiver startup failed')
        for fid in range(1,31):
            for seq in range(190):
                sender.sendto(PACKET.pack(b'OFLW',1,2,1,0,fid,8+seq%19*8,8+seq//19*8,0,0,seq,160,90,1000,0),('127.0.0.1',6202))
            sender.sendto(PACKET.pack(b'OFST',1,2,2,1,fid,10,10,10,0,0,0,255<<8,1000,0),('127.0.0.1',6202))
            time.sleep(.01)
        time.sleep(.06);s=consumer.snapshot();assert s['flow']['control']['fresh'],s
        payload,peer=sink.recvfrom(2048);assert len(payload)==32 and peer==('127.0.0.1',6203)
        fault=struct.pack('!4s4B3H4B2H2B2I',b'OFDG',1,2,0,0,30,8,8,0,63,0,128,0,0,1,1,1000,0)
        sender.sendto(fault,('127.0.0.1',6202));time.sleep(.06)
        s=consumer.snapshot();assert s['flow']['health_fault'] and s['flow']['control']['stop_request']
        time.sleep(.25);s=consumer.snapshot();assert not s['flow']['control']['fresh'] and s['flow']['control']['speed_limit']==0
        print(json.dumps(dict(passed=True,packets_sent=5731,received=s['stats']['packets'],hardware_tested=False)))
    finally:
        sender.close();consumer.close();sink.close();assert not consumer.process.is_alive()

if __name__=='__main__':main()
