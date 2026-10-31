"""Independent wire fixtures and byte-exact dual-send/FCS audit."""
from pathlib import Path
import struct,ipaddress,zlib,subprocess,json
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'tests/network_dual_20261014';OUT.mkdir(parents=True,exist_ok=True)
LOCAL=bytes.fromhex('020a3501fec0');PC=bytes.fromhex('7e51518f44b4');NANO=bytes.fromhex('48b02d5bd174');OTHER=bytes.fromhex('021122334455')
def ip(value):return ipaddress.IPv4Address(value).packed
def check(data):
    if len(data)&1:data+=b'\0'
    total=sum(struct.unpack('!'+str(len(data)//2)+'H',data))
    while total>>16:total=(total&65535)+(total>>16)
    return (~total)&65535
def frame(body,bad=False):
    body=body.ljust(60,b'\0');return b'\x55'*7+b'\xd5'+body+struct.pack('<I',zlib.crc32(body)^int(bad))
def arp(sender,mac):
    return frame(LOCAL+mac+b'\x08\x06'+struct.pack('!HHBBH',1,0x800,6,4,2)+mac+ip(sender)+LOCAL+ip('192.168.50.30'))
def cmd(op=5,target='192.168.50.20',port=6102,rid=1,badip=False,badudp=False,badfcs=False,frag=0):
    payload=struct.pack('!4sBBHI4sHH',b'NCFG',1,op,0,rid,ip(target),port,0)
    udp=struct.pack('!4H',44000,6200,28,0)+payload
    cs=check(ip('192.168.50.100')+ip('192.168.50.30')+struct.pack('!BBH',0,17,28)+udp) or 65535
    udp=udp[:6]+struct.pack('!H',cs^int(badudp))+udp[8:]
    head=struct.pack('!BBHHHBBH4s4s',0x45,0,48,1,frag,64,17,0,ip('192.168.50.100'),ip('192.168.50.30'))
    head=head[:10]+struct.pack('!H',check(head)^int(badip))+head[12:]
    return frame(LOCAL+PC+b'\x08\x00'+head+udp,badfcs)
fixtures={'set_pc':cmd(2,'192.168.50.100'), 'set_nano':cmd(), 'set_nano_new':cmd(target='192.168.50.21',port=7000),
          'arp_pc':arp('192.168.50.100',PC),'arp_nano':arp('192.168.50.20',NANO),'arp_nano_new':arp('192.168.50.21',OTHER),
          'bad_fcs':cmd(target='192.168.50.99',badfcs=True),'bad_ip':cmd(target='192.168.50.99',badip=True),
          'bad_udp':cmd(target='192.168.50.99',badudp=True),'fragment':cmd(target='192.168.50.99',frag=0x2000),
          'off_subnet':cmd(target='10.0.0.2'),'duplicate_target':cmd(target='192.168.50.100'),
          'clear_pc':cmd(4),'clear_nano':cmd(8),'stop_pc':cmd(3),'stop_nano':cmd(6),'status_pc':cmd(1),'status_nano':cmd(7)}
for name,data in fixtures.items():
    (OUT/(name+'.hex')).write_text('\n'.join('%02x'%x for x in data)+'\n');(OUT/(name+'.len')).write_text(str(len(data)))
args=['iverilog','-g2012','-s','tb_network_dual','-o',str(OUT/'test.vvp'),'sim/tb_network_dual.v','src/network_runtime_dual.v',
      'src/eth_udp_gmii/eth_udp_tx_gmii.v','src/eth_udp_gmii/ip_checksum.v','src/eth_udp_gmii/crc32_d8.v']
r=subprocess.run(args,cwd=ROOT,capture_output=True,text=True);(OUT/'compile.log').write_text(r.stdout+r.stderr)
if r.returncode:raise SystemExit(r.stderr)
r=subprocess.run(['vvp',str(OUT/'test.vvp')],cwd=ROOT,capture_output=True,text=True,timeout=45)
(OUT/'sim.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr)
if r.returncode or 'PASS' not in r.stdout:raise SystemExit(1)
packets=[];p=[]
for line in (OUT/'tx.hex').read_text().splitlines():
    if line=='END':
        if p:packets.append(bytes(p));p=[]
    else:p.append(int(line,16))
video=[];acks=[]
for p in packets:
    assert p[:8]==b'\x55'*7+b'\xd5'
    b=p[8:];assert zlib.crc32(b[:-4])==int.from_bytes(b[-4:],'little'),'Ethernet CRC'
    if b[12:14]!=b'\x08\x00':continue
    assert check(b[14:34])==0,'IP checksum'
    source,port,ulen,cs=struct.unpack_from('!4H',b,34);payload=b[42:42+ulen-8]
    if source==5000:
        assert payload==bytes((i&255)^(i>>8) for i in range(len(payload))),'Replay/source payload mismatch'
        video.append(dict(ip=str(ipaddress.IPv4Address(b[30:34])),mac=b[:6].hex(),port=port,length=len(payload),payload=payload.hex()))
    else:
        assert payload[:4]==b'NSTA' and payload[7]&4
        acks.append(dict(op=payload[5],error=payload[6]))
assert len(video)==21,('unexpected target copy count',len(video))
expected=[('192.168.50.100',1456),('192.168.50.20',1456),('192.168.50.100',31),('192.168.50.20',31),('192.168.50.100',1),('192.168.50.20',1),
          ('192.168.50.20',31),('192.168.50.20',31),('192.168.50.100',31),('192.168.50.20',31),('192.168.50.20',31),
          ('192.168.50.100',1456),('192.168.50.20',1456),('192.168.50.100',31),('192.168.50.100',31),
          ('192.168.50.100',31),('192.168.50.21',31),('192.168.50.100',31),('192.168.50.100',31),
          ('192.168.50.100',31),('192.168.50.20',31)]
assert [(v['ip'],v['length']) for v in video]==expected,'Recipient frame/start independence'
for v in video:
    assert v['mac']=={'192.168.50.100':PC.hex(),'192.168.50.20':NANO.hex(),'192.168.50.21':OTHER.hex()}[v['ip']]
    assert v['port']==(7000 if v['ip'].endswith('.21') else 6102)
    del v['payload']
summary=dict(status='PASS',frames_checked=len(packets),video=video,acks=acks,
             checks=['byte identical copies including 1456/31/1-byte payloads','source read once and released after copies','PC .100/Nano .20 cache collision avoided',
                     'independent cache clear/stop and offline target','in-flight destination snapshot','per-target complete-frame start','bad FCS/IP/UDP/fragment rejection','TTL','CRC and IP checksum'])
(OUT/'verification.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='video'},indent=2))
