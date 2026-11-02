"""Frozen raw-stream sensor top: original STL packets, seven status types, GPIO guards."""
from pathlib import Path
import collections,hashlib,json,subprocess,sys
from test_raw_ethernet import ROOT,OUT,make_fixture,audit_packets
from tools.task4.protocol import StatusStream


def main():
    folder=Path(sys.argv[1]).resolve();meta=json.loads((folder/'manifest.json').read_text())
    system=meta['top']=='system_task4_top'
    assert meta['top'] in ('m2_sensor_eth_diag_top','system_task4_top') and meta['lidar_mode']=='raw_points'
    originals,data,fixture=make_fixture()
    bench=ROOT/('tests/navigation/tb_system_raw_ethernet.sv' if system else 'tests/diagnostics/tb_m2_sensor_raw_ethernet.sv')
    files=[folder/'source'/p for p in meta['source_sha256'] if Path(p).name!='ethernet_pll.v']
    prim='C:/Gowin/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v'
    for name,cmd in [('test_compile',['iverilog','-g2012','-s',bench.stem,'-s','GSR','-o',str(folder/'test.vvp'),str(bench),
                     'tests/ethernet_pll_model.v','tests/imu/mpu6050_model.sv',prim]+list(map(str,files))),
                     ('test_simulation',['vvp',str(folder/'test.vvp')])]:
        p=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=240)
        (folder/(name+'.log')).write_text(p.stdout+p.stderr)
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
        print(p.stdout,end='',flush=True)
    trace=OUT/('system_raw_top_tx.hex' if system else 'static_raw_top_tx.hex');rows,scans,video,recorded=audit_packets(trace)
    (folder/trace.name).write_bytes(trace.read_bytes())
    counts=collections.Counter(kind for kind,row in rows)
    nav=StatusStream();counts['nav']=len(nav.feed(b''.join(recorded)))
    assert {'motor','encoder_diagnostics','encoder_trace','imu','nav','lidar_raw','lidar_diagnostic'}<=set(counts),counts
    assert counts['lidar_raw']==2000 and video==int(system) and nav.bad_crc==0
    assert len(scans)>=2 and all(len(scan['points'])>2000 for scan in scans)
    motors=[row for kind,row in rows if kind=='motor'];imus=[row for kind,row in rows if kind=='imu']
    assert motors and all(not row['armed'] and row['left_applied']==row['right_applied']==0 for row in motors)
    assert any(row['type']=='sample' and row['ready'] and not row['fault'] for row in imus)
    # Every point must survive the actual top, including zero returns.
    import struct
    raw=[row for kind,row in rows if kind=='lidar_raw']
    for i,packet in enumerate(originals):
        start=struct.unpack_from('<H',packet,4)[0];end=struct.unpack_from('<H',packet,42)[0];span=(end-start)%36000
        for half in (0,1):
            row=raw[i*2+half]
            assert row['packet_sequence']==i+1 and row['half']==half
            for j,p in enumerate(row['points']):
                n=half*6+j;d,q=struct.unpack_from('<HB',packet,6+n*3)
                assert p==[((start+(n*span+5)//11)%36000)/100.,d,q]
    diag=[row for kind,row in rows if kind=='lidar_diagnostic'][-1]
    assert diag['dropped_packets']==0 and diag['crc_errors']==1 and diag['forwarded_packets']==1000
    result=dict(passed=True,programmed=False,hardware_tested=False,lidar_mode='raw_points',fixture=fixture,
                source_sha256=meta['source_sha256'],testbench_path=bench.relative_to(ROOT).as_posix(),
                testbench_sha256=hashlib.sha256(bench.read_bytes()).hexdigest(),counts=dict(counts),
                complete_scans=len(scans),points_per_scan=[len(s['points']) for s in scans],all_points_checked=12000,
                all_motor_pads_low=True,uart_idle=True,video_packets=video,imu_model_samples=True,imu_decimation_fixture=2,imu_decimation_production=20)
    (folder/'test_result.json').write_text(json.dumps(result,indent=2))
    print('PASS frozen raw production top: 12000 exact points, native-density scans, seven UDP status types, IMU model and motor GPIO guards')


if __name__=='__main__':main()
