"""Reproducible task-4 checks; artifacts remain in build/task4, never build/system."""
from pathlib import Path
import hashlib,json,math,os,struct,subprocess,sys,time
from test_math import run as math_test
from test_vfh import run as vfh_test
from test_lidar import run as lidar_test
from test_display import run as display_test
from simulate import run as simulate_test
from model import rpp,circular,vfh
from protocol import configuration_frames,decode_status,crc16,StatusStream
from build import sources
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'build/task4'
CORE=['fpga/navigation/'+n+'.sv' for n in ('nav_core','nav_math','nav_histogram','nav_vfh')]
def command(args,name,timeout=150):
    result=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,timeout=timeout)
    (OUT/(name+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode:raise RuntimeError(name+' failed: '+result.stdout[-2500:]+result.stderr[-2500:])
    return result.stdout
def bench(name,files):
    command(['iverilog','-g2012','-s',name,'-o',str(OUT/(name+'.vvp')),'tests/navigation/'+name+'.sv']+files,name+'_compile')
    output=command(['vvp',str(OUT/(name+'.vvp'))],name)
    assert 'PASS' in output;print(output.strip(),flush=True)
def protocol_test():
    pending=json.loads((ROOT/'docs/task4/calibration.pending.json').read_text());mission=json.loads((ROOT/'docs/task4/mission.example.json').read_text())
    try:configuration_frames(pending,mission)
    except ValueError:pass
    else:raise AssertionError('Pending calibration accepted')
    c=dict(confirmed=True,wheel_diameter_mm=65,left_counts_per_rev=1500,right_counts_per_rev=1500,track_mm=280,robot_radius_mm=180,gyro_axis_sign=2,gyro_bias_raw=0,lidar_angle_offset_q100=0,lidar_reverse=False)
    frames,seq=configuration_frames(c,mission)
    assert len(frames)==36 and frames[-1][3]==0x40 and frames[-1][6:12]==bytes.fromhex('00000f000000')
    for frame in frames:assert len(frame)==16 and crc16(frame[:14])==struct.unpack_from('<H',frame,14)[0] and frame[3] not in (1,0x42)
    body=b'\xc3\x3c\x04\x01'+struct.pack('<BBHiiihhhhHIIII',7,2,0,65536,-131072,6553600,120,-200,5,6,1234,42,20,1,0)
    frame=body+struct.pack('<H',crc16(body));row=decode_status(frame)
    assert row['x_mm']==1 and row['y_mm']==-2 and row['heading_degrees']==1 and row['v_mm_s']==120 and row['plans']==42
    parser=StatusStream();decoded=[]
    damaged=bytearray(frame);damaged[20]^=1
    stream=b'noise'+bytes(damaged)+frame
    for byte in stream:decoded+=parser.feed(bytes([byte]))
    assert decoded==[row] and parser.bad_crc==1
    print('PASS protocol preparation/calibration refusal/status resynchronization')
def rpp_test():
    bench('tb_rpp',CORE)
    rows=[list(map(int,r.split())) for r in (OUT/'rpp_output.txt').read_text().splitlines()]
    assert len(rows)==12
    for i,(gx,gy,done,v,w,h,bx,by,lookahead,retry) in enumerate(rows):
        if i==7:assert done and v==0 and w==0;continue
        assert not done and lookahead==300
        if i<10:
            ref=rpp((0,0,0),(0,0),(gx,gy),0)
            assert math.hypot(ref['carrot'][0]-bx,ref['carrot'][1]-by)<=2.5,(i,ref,bx,by)
            assert circular(h,ref['heading'])<30,(i,h,ref)
            if v:
                assert abs(v-ref['v'])<=2,(i,v,ref)
                assert abs(w-ref['w'])<=8,(i,w,ref)
            else:assert abs(ref['heading'])>=4470 and abs(w)==400
        elif i==10:assert v>0 and w<0,'VFH detour must progress when original carrot is behind'
        else:assert retry and v==0 and abs(w)>0,'No-valley turn-radius retry failed'
    print('PASS 12 RPP/scalar geometry + blocked-radius retry + detour progress cases')
def pose_test():
    bench('tb_pose_fusion',CORE)
    rows=[list(map(int,r.split())) for r in (OUT/'pose_output.txt').read_text().splitlines()]
    for i,((x,y,t),(dl,dr,mm_l,mm_r,gyro)) in enumerate(zip(rows,[(100,100,.1,.1,0),(-10,10,.1,.1,1341),(-10,10,.1,.1,1341),(10,10,.1,.2,670)])):
        ds=(dl*mm_l+dr*mm_r)/2
        enc=(dr*mm_r-dl*mm_l)/280
        gyr=gyro*2000/65536/100*math.pi/180
        turn=.25*enc+.75*gyr
        assert abs(t/65536/100*math.pi/180-turn)<.00002,(i,t,turn)
        assert abs(x/65536-ds*math.cos(turn/2))<.003,(i,x)
        assert abs(y/65536-ds*math.sin(turn/2))<.003,(i,y)
    print('PASS independent differential/gyro pose reference')
def full_top():
    prim=Path(os.environ.get('GOWIN_HOME','C:/Gowin/Gowin_V1.9.12.04_x64'))/'IDE/simlib/gw5a/prim_sim.v'
    full_hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources()}
    files=[str(p) for p in sources() if p.name!='ethernet_pll.v']
    command(['iverilog','-g2012','-s','tb_task4_system','-s','GSR','-o',str(OUT/'system.vvp'),
      'tests/navigation/tb_task4_system.sv','tests/imu/icm42688_model.sv','tests/ethernet_pll_model.v',str(prim)]+files,'full_compile')
    text=command(['vvp',str(OUT/'system.vvp')],'full_top',600);assert 'PASS' in text;print(text.strip().splitlines()[-2:],flush=True)
    parser=StatusStream();nav=parser.feed((OUT/'full_system_uart.bin').read_bytes())
    assert nav and not parser.bad_crc and all(not r['running'] and not r['permit'] for r in nav)
    sys.path.insert(0,str(ROOT));from shared.streams import MixedStream
    mixed=MixedStream();rows=mixed.feed((OUT/'full_system_uart.bin').read_bytes())
    assert not mixed.bad_crc and {'motor','imu','encoder_diagnostics','encoder_trace'}<={kind for kind,row in rows}
    from shared.lidar_display import FpgaLidar
    display=FpgaLidar()
    for kind,row in rows:display.ingest(kind,row,now=1)
    assert display.scan and len(display.scan['points'])==72,'No complete scan under full motor/encoder/IMU/nav load'
    assert all(p==[i*5+2.5,1000+i,100] for i,p in enumerate(display.scan['points']))
    assert display.diagnostic and display.diagnostic['packets']==108 and display.diagnostic['crc_errors']==1
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in full_hashes.items())
    return dict(nav_records=len(nav),other_records=len(rows),bad_crc=0,source_sha256=full_hashes)
def main():
    OUT.mkdir(parents=True,exist_ok=True);result=dict(hardware_tested=False,programmed=False)
    paths=list((ROOT/'fpga/navigation').rglob('*'))+list((ROOT/'tests/navigation').rglob('*.sv'))+list((ROOT/'tools/task4').glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}
    result['math_vectors']=math_test();result['vfh']=vfh_test();result['lidar']=lidar_test();result['lidar_display']=display_test();protocol_test()
    bench('tb_histogram',['fpga/navigation/nav_histogram.sv']);bench('tb_intent_mux',['fpga/navigation/nav_command_mux.sv'])
    rpp_test();pose_test();bench('tb_navigation',CORE);result['full_top']=full_top();result['kinematic']=simulate_test()
    result['source_sha256']=hashes;result['sources_unchanged']=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v for p,v in hashes.items());assert result['sources_unchanged']
    result['passed']=True;(OUT/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print('ALL TASK-4 OFFLINE CHECKS PASSED',flush=True)
if __name__=='__main__':main()
