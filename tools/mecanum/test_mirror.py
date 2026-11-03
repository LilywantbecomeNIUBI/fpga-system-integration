"""Verify byte-identical telemetry copies at both existing router endpoints."""
from pathlib import Path
import struct,ipaddress,zlib,subprocess,json
from test_router import ROOT,OUT,wire,checksum

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    for name,op,ip in [('set_pc',2,'192.168.50.10'),('set_nano',5,'192.168.50.20'),('stop_pc',3,'192.168.50.10'),('stop_nano',6,'192.168.50.20')]:
        data=wire(struct.pack('!4sBBHI4sHH',b'NCFG',1,op,0,123,ipaddress.IPv4Address(ip).packed,6102,0),port=6200)
        (OUT/(name+'.hex')).write_text('\n'.join('%02x'%b for b in data))
    for name,ip,mac in [('pc','192.168.50.10','020000000001'),('nano','192.168.50.20','020000000002')]:
        local=bytes.fromhex('020a3501fec0');peer=bytes.fromhex(mac)
        body=local+peer+b'\x08\x06'+struct.pack('!HHBBH',1,0x800,6,4,2)+peer+ipaddress.IPv4Address(ip).packed+local+ipaddress.IPv4Address('192.168.50.30').packed
        body=body.ljust(60,b'\0');data=b'\x55'*7+b'\xd5'+body+struct.pack('<I',zlib.crc32(body))
        (OUT/('arp_'+name+'.hex')).write_text('\n'.join('%02x'%b for b in data))
    files=['tests/mecanum/tb_router_mirror.sv','fpga/vision/src/network_runtime_dual.v','fpga/vision/src/eth_udp_gmii/eth_udp_tx_gmii.v','fpga/vision/src/eth_udp_gmii/ip_checksum.v','fpga/vision/src/eth_udp_gmii/crc32_d8.v']
    for label,cmd in [('compile',['iverilog','-g2012','-s','tb_router_mirror','-o',str(OUT/'mirror.vvp'),*files]),('simulation',['vvp',str(OUT/'mirror.vvp')])]:
        p=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=120)
        (OUT/('mirror_'+label+'.log')).write_text(p.stdout+p.stderr)
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
        if label=='simulation':assert 'PASS' in p.stdout;print(p.stdout,end='')
    packets=[];current=[]
    for line in (OUT/'mirror_tx.hex').read_text().splitlines():
        if line=='END':packets.append(bytes(current));current=[]
        else:current.append(int(line,16))
    copies=[];video=[]
    for packet in packets:
        frame=packet[8:];assert zlib.crc32(frame[:-4])==struct.unpack('<I',frame[-4:])[0]
        if frame[12:14]!=b'\x08\x00':continue
        assert checksum(frame[14:34])==0
        source,dest,length,cs=struct.unpack_from('!4H',frame,34)
        if source==5000:
            assert frame[42:42+length-8]==bytes(range(32))
            video.append(str(ipaddress.IPv4Address(frame[30:34])))
        if source==6205:
            assert dest==6205 and length==72
            address=str(ipaddress.IPv4Address(frame[30:34]))
            assert frame[:6].hex()=={'192.168.50.10':'020000000001','192.168.50.20':'020000000002'}[address]
            copies.append((address,frame[42:106]))
    assert copies==[('192.168.50.10',bytes([0x3c])*64),('192.168.50.20',bytes([0x3c])*64),('192.168.50.20',bytes([0x5a])*64),('192.168.50.10',bytes([0xa5])*64)],copies
    assert video==['192.168.50.10','192.168.50.20'],video
    print('PASS four wire copies, correct MAC/IP/port/payload/FCS; Nano destination unchanged')
    (OUT/'mirror_result.json').write_text(json.dumps(dict(passed=True,copies=len(copies),concurrent_video_copies=len(video),hardware_tested=False),indent=2))

if __name__=='__main__':main()
