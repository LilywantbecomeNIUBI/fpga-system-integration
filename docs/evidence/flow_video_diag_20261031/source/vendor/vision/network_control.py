"""Configure FPGA same-LAN video destination at runtime; no target MAC argument."""
import argparse
import ipaddress
import json
import secrets
import socket
import struct
import subprocess
import sys
import time

REQUEST=struct.Struct('!4sBBHI4sHH')
REPLY=struct.Struct('!4sBBBBI4sH6sII')
OPS={'status':1,'set':2,'stop':3,'clear':4}
SECOND_OPS={'status':7,'set':5,'stop':6,'clear':8}


def choose_local_ip(board):
    """Choose an up, directly connected LAN interface; socket.connect can be hijacked by a TUN."""
    remote=ipaddress.IPv4Address(board)
    candidates=[]
    try:
        import psutil
        stats=psutil.net_if_stats()
        for name,addresses in psutil.net_if_addrs().items():
            if name not in stats or not stats[name].isup:continue
            for address in addresses:
                if address.family!=socket.AF_INET or not address.netmask:continue
                network=ipaddress.IPv4Network('%s/%s'%(address.address,address.netmask),strict=False)
                if remote in network and network.prefixlen>0 and address.address!=str(remote):
                    candidates.append((network.prefixlen,stats[name].speed,address.address))
    except ImportError:
        if sys.platform=='win32':
            command='$cameraUpIndices=@(Get-NetAdapter | Where-Object Status -eq Up | Select-Object -ExpandProperty ifIndex); Get-NetIPAddress -AddressFamily IPv4 | Where-Object {$_.AddressState -eq "Preferred" -and $_.InterfaceIndex -in $cameraUpIndices} | Select-Object IPAddress,PrefixLength | ConvertTo-Json -Compress'
            data=subprocess.check_output(['powershell','-NoProfile','-Command',command],creationflags=subprocess.CREATE_NO_WINDOW)
            rows=json.loads(data.decode('utf-8-sig'));rows=rows if isinstance(rows,list) else [rows]
            for row in rows:
                network=ipaddress.IPv4Network('%s/%s'%(row['IPAddress'],row['PrefixLength']),strict=False)
                if remote in network and network.prefixlen>0 and row['IPAddress']!=str(remote):candidates.append((network.prefixlen,0,row['IPAddress']))
        else:
            rows=json.loads(subprocess.check_output(['ip','-j','-4','addr','show','up']).decode())
            for row in rows:
                if row.get('operstate')!='UP':continue
                for address in row.get('addr_info',[]):
                    network=ipaddress.IPv4Network('%s/%s'%(address['local'],address['prefixlen']),strict=False)
                    if remote in network and network.prefixlen>0 and address['local']!=str(remote):candidates.append((network.prefixlen,0,address['local']))
    if not candidates:raise RuntimeError('No active local interface in the FPGA LAN; connect Ethernet or robot Wi-Fi, or use --local-ip')
    candidates.sort(reverse=True)
    return candidates[0][2]


def transact(board='192.168.50.30',control_port=6200,operation='status',target='auto',port=6102,local_ip=None,slot=0):
    # AV_OWNERSHIP_GUARD: a standalone launcher must not replace a running AV session.
    from pathlib import Path as _AVPath
    if operation != 'status' and any((p / '.fpga-av-active.json').exists() for p in _AVPath(__file__).resolve().parents):
        raise RuntimeError('FPGA is owned by AV integration. Stop it and run scripts/av/control.py release before using a legacy launcher.')
    local_ip=local_ip or choose_local_ip(board)
    if operation=='set':
        destination=ipaddress.IPv4Address(local_ip if target=='auto' else target)
        network=ipaddress.IPv4Network(board+'/24',strict=False)
        if destination not in network or destination in (network.network_address,network.broadcast_address,ipaddress.IPv4Address(board)) or not 1<=port<=65535:
            raise ValueError('Target must be a same-LAN host and port must be 1..65535')
        # Force fresh ARP on explicit reconfiguration, including a replacement NIC retaining the same IP.
        transact(board,control_port,'clear',local_ip=local_ip,slot=slot)
    request_id=secrets.randbits(32)
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind((local_ip,0))
        sock.connect((board,control_port))
        actual_local=sock.getsockname()[0]
        destination=actual_local if target=='auto' else target
        operation_code=(SECOND_OPS if slot else OPS)[operation]
        packet=REQUEST.pack(b'NCFG',1,operation_code,0,request_id,ipaddress.IPv4Address(destination).packed,port,0)
        sock.settimeout(1)
        # SRAM reconfiguration restarts PHY negotiation and may outlast the first ARP retry window.
        for attempt in range(12):
            sock.send(packet)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                try:data=sock.recv(2048)
                except (socket.timeout,ConnectionResetError):break
                if len(data)!=REPLY.size:continue
                magic,version,opcode,error,flags,rid,ip_raw,video_port,mac,requests,rx_packets=REPLY.unpack(data)
                if magic!=b'NSTA' or version!=1 or rid!=request_id or opcode!=(operation_code|128):continue
                result=dict(board_ip=board,local_ip=actual_local,target_ip=str(ipaddress.IPv4Address(ip_raw)),
                            target_port=video_port,enabled=bool(flags&1),arp_valid=bool(flags&2),
                            target_mac=':'.join('%02X'%x for x in mac),arp_requests=requests,
                            received_small_frames=rx_packets,error=error,request_id=rid,
                            slot=slot,supported_slots=2 if flags&4 else 1,perception_events=bool(flags&16))
                if error:raise RuntimeError('FPGA rejected configuration: '+json.dumps(result))
                return result
    raise TimeoutError('No FPGA configuration reply; check ARP/RGMII/LAN and UDP 6200')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation',choices=list(OPS),nargs='?',default='status')
    parser.add_argument('--board',default='192.168.50.30')
    parser.add_argument('--control-port',type=int,default=6200)
    parser.add_argument('--target',default='auto',help='auto selects current route/interface address; or specify Nano/PC IPv4')
    parser.add_argument('--port',type=int,default=6102)
    parser.add_argument('--local-ip',help='Choose a specific current PC interface for configuration requests')
    parser.add_argument('--slot',type=int,choices=(0,1),default=0,help='0=PC; 1=second target on dual profile')
    parser.add_argument('--wait-arp',type=float,default=0)
    args=parser.parse_args()
    if not 1<=args.port<=65535 or not 1<=args.control_port<=65535:parser.error('Port must be 1..65535')
    result=transact(args.board,args.control_port,args.operation,args.target,args.port,args.local_ip,args.slot)
    if args.wait_arp:
        expected=result['target_ip'];deadline=time.monotonic()+args.wait_arp
        while not result['arp_valid'] and time.monotonic()<deadline:
            time.sleep(.2)
            result=transact(args.board,args.control_port,'status',local_ip=args.local_ip,slot=args.slot)
            if result['target_ip']!=expected:raise RuntimeError('Target changed during ARP resolution')
        if not result['arp_valid']:raise TimeoutError('Target did not answer ARP: '+json.dumps(result))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
