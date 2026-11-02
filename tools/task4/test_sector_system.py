"""Frozen full system: restored 360-sector LiDAR, video, events and seven states."""
from pathlib import Path
import collections,hashlib,json,struct,subprocess,sys
from test_raw_ethernet import ROOT,OUT,make_fixture,audit_packets
from shared.lidar_raw import RawLidar
from tools.task4.protocol import StatusStream


def main():
    folder=Path(sys.argv[1]).resolve();meta=json.loads((folder/'manifest.json').read_text())
    assert meta['top']=='system_task4_top' and meta['lidar_mode']=='nearest_return_per_sector'
    assert meta['lidar_bins']==360 and not meta['programmed']
    packets,data,fixture=make_fixture()
    # Preserve the approved raw bench. Only select the fallback branch, output
    # file and branch-specific end counters in this generated frozen fixture.
    source=(ROOT/'tests/navigation/tb_system_raw_ethernet.sv').read_text(encoding='utf-8')
    source=source.replace('tb_system_raw_ethernet','tb_system_sector_ethernet').replace('.LIDAR_RAW(1)','.LIDAR_RAW(0)')
    source=source.replace('system_raw_top_tx.hex','system_sector_top_tx.hex')
    source=source.replace('dut.ethernet_raw.lidar_display.forwarded!=1000 || dut.ethernet_raw.lidar_display.dropped!=0',
        'dut.ethernet_scan.lidar_display.completed==0 || dut.ethernet_scan.lidar_display.input_loss_count!=0')
    bench=folder/'tb_system_sector_ethernet.sv';bench.write_text(source,encoding='utf-8')
    files=[folder/'source'/p for p in meta['source_sha256'] if Path(p).name!='ethernet_pll.v']
    prim='C:/Gowin/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v'
    for name,cmd in [('sector_compile',['iverilog','-g2012','-s','tb_system_sector_ethernet','-s','GSR','-o',str(folder/'sector.vvp'),
            str(bench),'tests/ethernet_pll_model.v','tests/imu/mpu6050_model.sv',prim]+list(map(str,files))),
            ('sector_simulation',['vvp',str(folder/'sector.vvp')])]:
        result=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=600)
        (folder/(name+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        print(result.stdout,end='',flush=True)
    trace=OUT/'system_sector_top_tx.hex';rows,scans,video,recorded=audit_packets(trace)
    (folder/trace.name).write_bytes(trace.read_bytes());counts=collections.Counter(k for k,r in rows)
    nav=StatusStream();counts['nav']=len(nav.feed(b''.join(recorded)))
    assert {'motor','encoder_diagnostics','encoder_trace','imu','nav','lidar_scan','lidar_diagnostic'}<=set(counts)
    assert counts['lidar_raw']==0 and video==1 and scans and nav.bad_crc==0
    # Independent native-angle, packet-error and nearest-return reference.
    assembler=RawLidar();expected=[]
    for index,packet in enumerate(packets):
        start=struct.unpack_from('<H',packet,4)[0];span=(struct.unpack_from('<H',packet,42)[0]-start)%36000
        points=[]
        for i in range(12):
            distance,intensity=struct.unpack_from('<HB',packet,6+i*3)
            points.append([((start+(i*span+5)//11)%36000)/100.,distance,intensity])
        for half in (0,1):
            before=assembler.scans
            assembler.ingest(dict(packet_sequence=index+1,half=half,captured_ms=index,source_age_ms=0,
                discontinuity=index==300,points=points[half*6:half*6+6]),now=1+index/1000.)
            if assembler.scans>before:
                bins=[[0.,0,0] for _ in range(360)]
                for point in assembler.scan['points']:
                    angle,distance,intensity=point;sector=int(round(angle*100))//100
                    if distance and intensity and (not bins[sector][1] or distance<bins[sector][1]):bins[sector]=point
                expected.append(bins)
    for scan in scans:assert len(scan['points'])==360 and scan['points'] in expected
    motors=[r for k,r in rows if k=='motor'];imus=[r for k,r in rows if k=='imu']
    assert motors and all(not r['armed'] and r['left_applied']==r['right_applied']==0 for r in motors)
    assert any(r['type']=='sample' and r['ready'] and not r['fault'] for r in imus)
    diagnostic=[r for k,r in rows if k=='lidar_diagnostic'][-1]
    assert diagnostic['packets']==1000 and diagnostic['crc_errors']==1
    result=dict(passed=True,programmed=False,hardware_tested=False,lidar_mode=meta['lidar_mode'],lidar_bins=360,
        source_sha256=meta['source_sha256'],generated_bench_sha256=hashlib.sha256(bench.read_bytes()).hexdigest(),
        counts=dict(counts),complete_scans=len(scans),points_per_scan=360,video_packets=video,fixture=fixture,
        all_motor_pads_low_while_unarmed=True,uart_idle=True,imu_model_samples=True,nearest_reference_verified=True)
    (folder/'test_result.json').write_text(json.dumps(result,indent=2))
    print('PASS full fallback system: 360 measured-angle nearest returns, video/event concurrency and all seven UDP states')


if __name__=='__main__':main()
