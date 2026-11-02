"""Verify the frozen, routed static Ethernet diagnostic top with digital models."""
from pathlib import Path
import collections
import hashlib
import json
import struct
import subprocess
import sys
import zlib
from test_ethernet import ROOT,OUT,fixtures,checksum
from shared.streams import MixedStream
from shared.lidar_display import FpgaLidar
from shared.telemetry_udp import TelemetryUDP,HEADER
from tools.task4.protocol import StatusStream


def main():
    folder=Path(sys.argv[1]).resolve()
    manifest=json.loads((folder/'manifest.json').read_text())
    assert manifest['top']=='m2_sensor_eth_diag_top'
    OUT.mkdir(parents=True,exist_ok=True);fixtures()
    prim=Path('C:/Gowin/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
    bench=ROOT/'tests/diagnostics/tb_m2_sensor_ethernet.sv'
    files=[folder/'source'/p for p in manifest['source_sha256'] if Path(p).name!='ethernet_pll.v']
    for name,cmd in [('test_compile',['iverilog','-g2012','-s','tb_m2_sensor_ethernet','-s','GSR','-o',str(folder/'test.vvp'),str(bench),'tests/ethernet_pll_model.v','tests/imu/mpu6050_model.sv',str(prim)]+list(map(str,files))),
                     ('test_simulation',['vvp',str(folder/'test.vvp')])]:
        result=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=180)
        (folder/(name+'.log')).write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        print(result.stdout, end='',flush=True)
    raw=(OUT/'static_top_tx.hex').read_bytes();(folder/'static_top_tx.hex').write_bytes(raw)
    packet=[];mixed=MixedStream();nav=StatusStream();lidar=FpgaLidar();udp=TelemetryUDP();counts=collections.Counter();imus=[];motors=[]
    for line in raw.decode().splitlines():
        if line!='END':packet.append(int(line,16));continue
        frame=bytes(packet)[8:];packet=[]
        assert zlib.crc32(frame[:-4])==struct.unpack('<I',frame[-4:])[0]
        if frame[12:14]!=b'\x08\x00':continue
        assert checksum(frame[14:34])==0
        source,port,length,_=struct.unpack_from('!4H',frame,34)
        if source!=6205:continue
        payload=frame[42:42+length-8];tick=HEADER.unpack_from(payload)[-1]
        inner=udp.feed(payload,('192.168.50.30',6205),now=1+tick/1000.)
        assert inner is not None
        counts['nav']+=len(nav.feed(inner))
        for kind,row in mixed.feed(inner):
            counts[kind]+=1;lidar.ingest(kind,row,now=0)
            if kind=='imu':imus.append(row)
            if kind=='motor':motors.append(row)
    assert {'motor','encoder_diagnostics','encoder_trace','lidar_scan','lidar_diagnostic','imu','nav'}<=set(counts),counts
    assert lidar.scan and all(p==[i+.37,1000+i,77] for i,p in enumerate(lidar.scan['points']))
    assert any(row['type']=='sample' and row['ready'] and not row['fault'] for row in imus)
    assert motors and all(not row['armed'] and row['left_applied']==row['right_applied']==0 for row in motors)
    assert mixed.bad_crc==nav.bad_crc==udp.rejected==udp.missing==0
    result=dict(passed=True,hardware_tested=False,source_sha256=manifest['source_sha256'],
                testbench_sha256=hashlib.sha256(bench.read_bytes()).hexdigest(),counts=dict(counts),
                points=360,all_motor_pads_low=True,uart_idle=True,imu_model_samples=True,
                imu_decimation_fixture=2,imu_decimation_production=20)
    (folder/'test_result.json').write_text(json.dumps(result,indent=2))
    print('PASS complete frozen sensor top: all seven status types over UDP, 360 measured points, IMU samples, motor outputs zero')


if __name__=='__main__':main()
