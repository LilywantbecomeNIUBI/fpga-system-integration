"""Reproduce counter/safety-plane comparison against the published old RTL."""
import argparse,hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
REFERENCES={
 'fpga/vision/src/optical_flow/flow_event_bridge.v':'5ba4ba91af9898d5e6d6ae4ec7fbb220d2966c4719e89cdd42eb6b26c884acf2',
 'fpga/vision/src/vision/vision_result_core.v':'fc654b2783e164a128719f181d982987299b084ea69dcf5be1c4bda87b390116',
 'fpga/vision/src/vision/vision_v23_bridge.v':'298e4114c5f4c700de4cef2a5cb4757a7c36a608ad5fa3a9ff48f58c070201dc'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,default=ROOT/'releases/frame-read-pipeline-20261103/source')
    args=parser.parse_args();out=ROOT/'build/tests/timing_refactor';out.mkdir(parents=True,exist_ok=True)
    reference_files=[]
    for path,digest in REFERENCES.items():
        src=args.reference/path;assert sha(src)==digest,('Reference RTL is not the frozen baseline',src)
        name=src.stem;dest=out/(name+'_reference.v')
        dest.write_text(src.read_text().replace('module '+name,'module '+name+'_reference',1))
        reference_files.append(str(dest))
    cases=[('bridge_equivalence','tb_bridge_timing_equivalence',
        ['tests/vision/tb_bridge_timing_equivalence.v',*REFERENCES,*reference_files]),
        ('event_ingress_bursts','tb_event_ingress_bursts',
        ['tests/vision/tb_event_ingress_bursts.v','fpga/vision/src/optical_flow/flow_event_bridge.v',
         'fpga/vision/src/vision/vision_v23_bridge.v']),
        ('rx_length_boundaries','tb_rx_length_boundaries',
        ['tests/vision/tb_rx_length_boundaries.v','fpga/vision/src/network_runtime_dual.v',
         'fpga/vision/src/eth_udp_gmii/eth_udp_tx_gmii.v','fpga/vision/src/eth_udp_gmii/crc32_d8.v',
         'fpga/vision/src/eth_udp_gmii/ip_checksum.v']),
        ('stale_deadline','tb_stale_deadline',
        ['tests/imu/tb_stale_deadline.sv','fpga/imu/mpu6050_reader.v','fpga/imu/imu_fast_i2c_master.v'])]
    inputs=set();results={}
    for name,top,files in cases:
        binary=out/(name+'.vvp');commands=[['iverilog','-g2012','-s',top,'-o',str(binary),*files],['vvp',str(binary)]]
        for index,cmd in enumerate(commands):
            r=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=60)
            (out/(name+str(index)+'.log')).write_text(r.stdout+r.stderr)
            if r.returncode:raise RuntimeError(r.stdout+r.stderr)
            if index:print(r.stdout,end='');results[name]=dict(passed=True,stdout=r.stdout)
        inputs.update(f for f in files if not Path(f).is_absolute())
    record=dict(passed=True,reference_sha256=REFERENCES,results=results,
        source_sha256={p:sha(ROOT/p) for p in sorted(inputs)})
    (out/'result.json').write_text(json.dumps(record,indent=2))
if __name__=='__main__':main()
