"""Icarus regression; simulations only, no board/serial operations."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parents[1]
def main():
    folder=ROOT/'build/tests';folder.mkdir(parents=True,exist_ok=True)
    compiler=shutil.which('iverilog'); runner=shutil.which('vvp')
    if not compiler or not runner: raise SystemExit('Icarus iverilog/vvp required')
    sources=sorted((ROOT/'fpga/motor').glob('*.sv'))+[ROOT/'fpga/system/imu_status_source.sv']
    records=[]
    for bench in sorted((ROOT/'tests').glob('tb_*.sv')):
        if bench.stem=='tb_full_system':continue
        image=folder/(bench.stem+'.vvp')
        compile_result=subprocess.run([compiler,'-g2012','-Wall','-s',bench.stem,'-o',str(image),*map(str,sources),str(bench)],capture_output=True,text=True)
        if compile_result.returncode: print(compile_result.stdout+compile_result.stderr);raise SystemExit(1)
        run=subprocess.run([runner,str(image)],capture_output=True,text=True,timeout=90)
        output=run.stdout+run.stderr;print(output.strip())
        records.append(dict(case=bench.stem,passed=run.returncode==0 and 'PASS' in output,output=output))
        if not records[-1]['passed']: raise SystemExit(1)
    if not records:raise SystemExit('No benches found')
    subprocess.run([sys.executable,'-m','unittest','discover','-s',str(ROOT/'tests'),'-p','test_*.py','-v'],check=True)
    record=dict(utc=datetime.now(timezone.utc).isoformat(),cases=records,hardware_tested=False,
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources+sorted((ROOT/'tests').glob('tb_*.sv'))+sorted((ROOT/'tests').glob('test_*.py'))+sorted((ROOT/'host').glob('*.py'))})
    (folder/'result.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    subprocess.run([sys.executable,str(ROOT/'tools/audit_pins.py')],check=True)

if __name__=='__main__':main()
