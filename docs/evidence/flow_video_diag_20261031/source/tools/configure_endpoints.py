"""Explicit endpoint setup; default is a dry run and never invoked by console startup."""
from pathlib import Path
import argparse,ipaddress,json,sys
from urllib.parse import urlsplit
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'vendor/vision'))
from network_control import transact
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--apply',action='store_true');a=ap.parse_args()
    config=json.loads(a.config.read_text(encoding='utf-8'));board=ipaddress.IPv4Address(config['fpga'])
    targets=[str(ipaddress.IPv4Address(config['bind'])),str(ipaddress.IPv4Address(urlsplit(config['nano_url']).hostname))]
    network=ipaddress.IPv4Network(str(board)+'/24',strict=False);port=config.get('video_port',6102)
    if type(port) is not int or not 1<=port<=65535:raise ValueError('Invalid video port')
    for target in targets:
        address=ipaddress.IPv4Address(target)
        if address not in network or address in (board,network.network_address,network.broadcast_address):raise ValueError('Endpoints must be host addresses on the FPGA /24 LAN')
    if targets[0]==targets[1]:raise ValueError('PC and Nano endpoints must be distinct')
    print(json.dumps(dict(fpga=str(board),slot0=targets[0],slot1=targets[1],port=port,apply=a.apply,change_os_network=False),indent=2))
    if a.apply:
        for slot,target in enumerate(targets):print(json.dumps(transact(str(board),operation='set',target=target,port=port,local_ip=targets[0],slot=slot),indent=2))
if __name__=='__main__':main()
