"""Clock-domain frame ownership regression; no hardware or actuator access."""
from pathlib import Path
import hashlib,json,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
def main():
    out=ROOT/'build/tests';out.mkdir(parents=True,exist_ok=True)
    sources=['tests/vision/tb_stable_frame_cdc.v','fpga/vision/src/vision/vision_frame_cdc.v',
             'fpga/vision/src/vision/vision_async_fifo.v','fpga/vision/src/vision/vision_feature_grid.v',
             'fpga/vision/src/vision/vision_lk_tracker.v','fpga/vision/src/vision/vision_divider.v',
             'fpga/vision/src/optical_flow/flow_frame_store.v','fpga/vision/src/optical_flow/frame_memory.sv',
             'fpga/vision/src/optical_flow/flow_lk_patch.v']
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
    subprocess.run(['iverilog','-g2012','-s','tb_stable_frame_cdc','-o',str(out/'stable_cdc.vvp')]+sources,cwd=ROOT,check=True)
    cases=[]
    for wh,rh in [(5.95235,10.0),(5.0,10.31),(8.173,9.71)]:
        start=time.monotonic()
        p=subprocess.run(['vvp',str(out/'stable_cdc.vvp'),f'+WRITE_HALF={wh}',f'+READ_HALF={rh}'],cwd=ROOT,capture_output=True,text=True,timeout=600)
        log=p.stdout+p.stderr;name=f'stable_cdc_{len(cases)}.txt';(out/name).write_text(log,encoding='utf-8')
        if p.returncode or 'PASS stable CDC' not in log:raise RuntimeError(log)
        cases.append(dict(write_half_ns=wh,read_half_ns=rh,passed=True,seconds=time.monotonic()-start,log=name))
        print(log,flush=True)
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (out/'stable_cdc.json').write_text(json.dumps(dict(passed=True,hardware_tested=False,cases=cases,source_sha256=hashes),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
