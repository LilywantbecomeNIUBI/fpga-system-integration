"""Offline all-point FIFO regression: tuples, backpressure, overload and expiry."""
from pathlib import Path
import hashlib,json,struct,subprocess,sys
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from host.motor_protocol import crc16


def main():
    out=ROOT/'build/lidar_raw_ethernet_20261102';out.mkdir(parents=True,exist_ok=True)
    sources=['tests/navigation/tb_lidar_points.sv','fpga/navigation/lidar_points_source.sv','fpga/motor/frame_transport.sv']
    for name,command in [('points_compile',['iverilog','-g2012','-Wall','-s','tb_lidar_points','-o',str(out/'points.vvp')]+sources),
                         ('points_sim',['vvp',str(out/'points.vvp')])]:
        result=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=60)
        (out/(name+'.log')).write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        print(result.stdout,end='',flush=True)
    seen={phase:[] for phase in range(1,5)};count=0
    for line in (out/'points_frames.hex').read_text().splitlines():
        phase,value=line.split();phase=int(phase);frame=bytes.fromhex(value)[::-1]
        assert len(frame)==48 and crc16(frame[:46])==int.from_bytes(frame[46:],'little')
        if frame[:4]!=b'\xc3\x3c\x07\x01':continue
        seq,age,captured,half,flag=struct.unpack_from('<I H I BB',frame,4)
        assert age<400 and half in (0,1) and flag==int(phase==4 and seq==2)
        seen[phase].append((seq,half))
        for i in range(half*6,half*6+6):
            index=(seq-1)*12+i;angle=(index*100+37)%36000
            if seq%30==5 and i==2:angle=((index-1)*100+37)%36000
            distance=0 if seq%30==6 and i==2 else 1000+index
            quality=0 if seq%30==7 and i==3 else 50+i
            assert struct.unpack_from('<HHB',frame,16+(i-half*6)*5)==(angle,distance,quality)
            count+=1
    assert seen[1]==[(seq,half) for seq in range(1,92) for half in (0,1)]
    assert seen[3]==[(3,0),(3,1)]
    assert seen[4]==[(1,0),(1,1),(2,0),(2,1)]
    assert seen[2][-2:]==[(21,0),(21,1)] and len(seen[2])<42
    result=dict(passed=True,hardware_tested=False,all_points_checked=count,
                frames_by_phase={str(k):len(v) for k,v in seen.items()},
                source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources})
    (out/'points_regression.json').write_text(json.dumps(result,indent=2))
    print('PASS independent raw FIFO tuple and discontinuity audit')


if __name__=='__main__':main()
