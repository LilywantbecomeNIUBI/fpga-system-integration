"""Raw UART -> production parser -> display RTL -> mixed UART -> Python scan.

No hardware IO. Data is synthetic and deliberately CRC-damaged once.
"""
from pathlib import Path
import hashlib,json,struct,subprocess,sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from shared.streams import MixedStream,crc8
from shared.lidar_display import FpgaLidar

def packet(start):
    body=b'\x54\x2c'+struct.pack('<HH',3600,start)
    for i in range(12):
        angle=start+(i*999+5)//11
        body+=struct.pack('<HB',1000+angle//500,100)
    body+=struct.pack('<HH',start+999,1234)
    return body+bytes([crc8(body)])

def run():
    out=ROOT/'build/task4';out.mkdir(parents=True,exist_ok=True)
    damaged=bytearray(packet(0));damaged[-1]^=1
    data=bytes(damaged)+b''.join(packet(i*1000) for _ in range(3) for i in range(36))
    assert len(data)==5123
    (out/'lidar_display_input.hex').write_text('\n'.join('%02x'%b for b in data))
    sources=['tests/navigation/tb_lidar_display.sv','fpga/navigation/lidar_scan_source.sv',
        'fpga/navigation/lidar/stl27l_uart_rx.v','fpga/navigation/lidar/stl27l_node_parser.v',
        'fpga/motor/frame_transport.sv','fpga/motor/uart_byte.sv','fpga/motor/motor_telemetry.sv']
    for command in (['iverilog','-g2012','-s','tb_lidar_display','-o',str(out/'display.vvp')]+sources,
                    ['vvp',str(out/'display.vvp')]):
        r=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=180)
        (out/('display_compile.log' if command[0]=='iverilog' else 'display.log')).write_text(r.stdout+r.stderr)
        if r.returncode:raise RuntimeError(r.stdout+r.stderr)
    parser=MixedStream();state=FpgaLidar();rows=[]
    raw=(out/'lidar_display_uart.bin').read_bytes()
    # Byte fragmentation is intentionally unrelated to the 48-byte boundaries.
    for i in range(0,len(raw),7):
        for kind,row in parser.feed(raw[i:i+7]):
            rows.append((kind,row));state.ingest(kind,row,now=i/11520.)
    assert parser.bad_crc==0
    assert state.scan and len(state.scan['points'])==72
    assert all(p==[i*5+2.5,1000+i,100] for i,p in enumerate(state.scan['points'])),state.scan
    diag=state.diagnostic
    assert diag and diag['bytes']==len(data) and diag['packets']==108 and diag['crc_errors']==1,diag
    assert any(k=='motor' and not r['armed'] and r['left_applied']==0 for k,r in rows)
    assert not state.snapshot(now=100)['online']
    result=dict(passed=True,hardware_tested=False,programmed=False,uart_bytes=len(raw),
        records={kind:sum(k==kind for k,_ in rows) for kind,_ in rows},bins=72,diagnostic=diag,
        source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources})
    (out/'display_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print('PASS display end-to-end: 72 exact bins, mixed UART CRC0, reception diagnostic counters, stale clearing')
    return result

if __name__=='__main__':run()
