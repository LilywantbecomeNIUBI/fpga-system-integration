from pathlib import Path
import hashlib,struct,subprocess
ROOT=Path(__file__).resolve().parents[2]
SOURCE=Path('D:/活动/比赛/电赛/FPGA/设计方案/雷达前端_50MHz时序通过_20261030/task4_lidar')
def crc8(data):
    c=0
    for b in data:
        c^=b
        for _ in range(8):c=((c<<1)^0x4d)&255 if c&128 else (c<<1)&255
    return c
def run():
    source_hashes={}
    for name in ('stl27l_uart_rx.v','stl27l_node_parser.v'):
        imported=ROOT/'fpga/navigation/lidar'/name;original=SOURCE/'src'/name
        assert imported.read_bytes()==original.read_bytes(),name
        source_hashes[name]=hashlib.sha256(imported.read_bytes()).hexdigest()
    raw=(SOURCE/'results/com7_921600.bin').read_bytes();packets=[]
    for i in range(len(raw)-46):
        p=raw[i:i+47]
        if p[:2]==b'\x54\x2c' and crc8(p[:46])==p[46]:packets.append(p)
        if len(packets)==8:break
    p=bytearray(b'\x54\x2c'+struct.pack('<HH',3600,35900))
    for i in range(12):p.extend(struct.pack('<HB',1000+i*3,100))
    p.extend(struct.pack('<HH',300,12345));p.append(crc8(p));packets.append(bytes(p))
    bad=bytearray(packets[0]);bad[-1]^=1
    (ROOT/'build/task4/lidar_input.hex').write_text('\n'.join('%02x'%b for b in bytes(bad)+b''.join(packets)))
    expected=[]
    for p in packets:
        start=struct.unpack_from('<H',p,4)[0];end=struct.unpack_from('<H',p,42)[0];span=(end-start)%36000
        for i in range(12):
            d,q=struct.unpack_from('<HB',p,6+i*3);expected.append((i,d,q,(start+(i*span+5)//11)%36000))
    subprocess.run(['iverilog','-g2012','-s','tb_lidar_import','-o','build/task4/lidar.vvp','tests/navigation/tb_lidar_import.sv','fpga/navigation/lidar/stl27l_uart_rx.v','fpga/navigation/lidar/stl27l_node_parser.v'],cwd=ROOT,check=True)
    subprocess.run(['vvp','build/task4/lidar.vvp'],cwd=ROOT,check=True)
    actual=[tuple(map(int,r.split())) for r in (ROOT/'build/task4/lidar_output.txt').read_text().splitlines()]
    assert actual==expected,'Imported UART/parser differs from raw-record reference'
    print('PASS imported radar',len(actual),'recorded/synthetic points')
    return dict(points=len(actual),source_sha256=source_hashes,capture_sha256=hashlib.sha256(raw).hexdigest())
if __name__=='__main__':run()
