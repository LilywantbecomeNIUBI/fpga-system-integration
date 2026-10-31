"""Listen to FPGA state/forced-inference events; does not start or change a model service."""
import argparse
import json
import socket
from vision_v23_protocol import EVENT_PORT,parse_event

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local-ip',required=True)
    parser.add_argument('--fpga',default='192.168.50.30')
    parser.add_argument('--port',type=int,default=EVENT_PORT)
    args=parser.parse_args()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind((args.local_ip,args.port))
        while True:
            data,peer=sock.recvfrom(2048)
            if peer!=(args.fpga,EVENT_PORT):continue
            try:result=parse_event(data)
            except ValueError:continue
            print(json.dumps(result,sort_keys=True),flush=True)
if __name__=='__main__':main()
