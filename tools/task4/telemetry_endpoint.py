"""Configure/query the existing second Ethernet endpoint; never sends motor commands."""
import argparse
import ipaddress
import json
from pathlib import Path
import secrets
import socket
import struct
import time

REQUEST=struct.Struct('!4sBBHI4sHH')
REPLY=struct.Struct('!4sBBBBI4sH6sII')


def transact(bind,board='192.168.50.30',target=None,video_port=None):
    board=str(ipaddress.IPv4Address(board));bind=str(ipaddress.IPv4Address(bind))
    if target is not None:
        for parent in Path(__file__).resolve().parents:
            if (parent/'.fpga-av-active.json').exists():
                raise RuntimeError('Active AV ownership marker: configure the shared endpoint through its owner')
        target=str(ipaddress.IPv4Address(target));network=ipaddress.IPv4Network(board+'/24',strict=False)
        if ipaddress.IPv4Address(target) not in network or target in (str(network.network_address),str(network.broadcast_address),board):
            raise ValueError('Telemetry target must be a distinct host on the FPGA LAN')
        # Preserve a live camera/Nano video port unless explicitly changed.
        current=transact(bind,board)
        video_port=video_port or current['video_port'] or 6102
        if not 1<=video_port<=65535:raise ValueError('Invalid video port')
    rid=secrets.randbits(32);opcode=5 if target is not None else 7
    request=REQUEST.pack(b'NCFG',1,opcode,0,rid,ipaddress.IPv4Address(target or bind).packed,video_port or 6102,0)
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream:
        stream.bind((bind,0));stream.connect((board,6200));stream.settimeout(1)
        for _ in range(12):
            stream.send(request);deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                try:data=stream.recv(2048)
                except (socket.timeout,ConnectionResetError):break
                if len(data)!=32:continue
                magic,version,op,error,flags,reply_id,ip,port,mac,requests,received=REPLY.unpack(data)
                if magic!=b'NSTA' or version!=1 or op!=(opcode|128) or reply_id!=rid:continue
                if error:raise RuntimeError('FPGA rejected endpoint request, error=%d'%error)
                return dict(board=board,bind=bind,target=str(ipaddress.IPv4Address(ip)),video_port=port,
                            telemetry_port=6205,enabled=bool(flags&1),arp_valid=bool(flags&2),
                            arp_requests=requests,received_commands=received)
        raise TimeoutError('No FPGA endpoint acknowledgment')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--bind',required=True)
    parser.add_argument('--board',default='192.168.50.30');parser.add_argument('--target')
    parser.add_argument('--video-port',type=int);args=parser.parse_args()
    print(json.dumps(transact(args.bind,args.board,args.target,args.video_port),indent=2))


if __name__=='__main__':main()
