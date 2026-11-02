"""Real Ethernet wire fixtures -> async command mailbox -> production chassis."""
from pathlib import Path
import struct,ipaddress,zlib,sys,subprocess,json
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'host/mecanum'))
from console import packet
OUT=ROOT/'build/mecanum/router_tests'

def checksum(data):
    if len(data)%2:data+=b'\0'
    n=sum(struct.unpack('!%dH'%(len(data)//2),data))
    while n>>16:n=(n&65535)+(n>>16)
    return (~n)&65535

def wire(payload,port=6206,bad=None,zero=False):
    src=ipaddress.IPv4Address('10.0.0.10' if bad=='subnet' else '192.168.50.10').packed
    dst=ipaddress.IPv4Address('192.168.50.30').packed
    udp=struct.pack('!4H',43000,port,8+len(payload),0)+payload
    cs=checksum(src+dst+struct.pack('!BBH',0,17,len(udp))+udp) or 65535
    if bad=='udp':cs^=1
    udp=udp[:6]+struct.pack('!H',0 if zero else cs)+udp[8:]
    ip=struct.pack('!BBHHHBBH4s4s',0x45,0,20+len(udp),1,0x2000 if bad=='fragment' else 0,64,17,0,src,dst)
    cs=checksum(ip)^(1 if bad=='ip' else 0)
    ip=ip[:10]+struct.pack('!H',cs)+ip[12:]
    data=bytes.fromhex('020a3501fec00200000000010800')+ip+udp
    if bad=='mac':data=b'\x04'+data[1:]
    data+=b'\0'*max(0,60-len(data))
    fcs=zlib.crc32(data)^(1 if bad=='fcs' else 0)
    return b'\x55'*7+b'\xd5'+data+struct.pack('<I',fcs)

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    frames={name:wire(packet(3,1),bad=name) for name in ('udp','ip','fragment','subnet','mac','fcs')}
    frames.update(clear=wire(packet(3,1)),arm=wire(packet(1,2)),pose=wire(packet(6,3)),keep=wire(packet(7,4)),stop=wire(packet(0,5)),arm6=wire(packet(1,6),zero=True),wrong_port=wire(packet(3,1),port=6200),short=wire(packet(3,1)[:-1]),long=wire(packet(3,1)+b'\0'))
    bad=bytearray(packet(3,1));bad[-1]^=1;frames['inner']=wire(bad)
    for name,data in frames.items():(OUT/(name+'.hex')).write_text('\n'.join('%02x'%b for b in data))
    files=[*sorted((ROOT/'fpga/mecanum').glob('*.sv')),ROOT/'fpga/motor/frame_transport.sv',ROOT/'fpga/motor/uart_byte.sv',ROOT/'fpga/vision/src/network_runtime_dual.v',*sorted((ROOT/'fpga/vision/src/eth_udp_gmii').glob('*.v'))]
    commands=[['iverilog','-g2012','-s','tb_router_commands','-o',str(OUT/'test.vvp'),str(ROOT/'tests/mecanum/tb_router_commands.sv'),*map(str,files)],['vvp',str(OUT/'test.vvp')]]
    for name,cmd in zip(('compile','simulation'),commands):
        p=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=180)
        (OUT/(name+'.log')).write_text(p.stdout+p.stderr,encoding='utf-8')
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
        if name=='simulation':
            assert 'PASS' in p.stdout
            print(p.stdout,end='')
    (OUT/'result.json').write_text(json.dumps(dict(passed=True,fixtures=len(frames),hardware_tested=False),indent=2))

if __name__=='__main__':main()
