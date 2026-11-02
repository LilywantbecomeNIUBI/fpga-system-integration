"""Run offline tests with Icarus or local ModelSim; never touches hardware."""
from pathlib import Path
import subprocess
import shutil
import tempfile
import sys
ROOT=Path(__file__).resolve().parents[1]
subprocess.run([sys.executable,'-m','unittest','discover','-s',str(ROOT/'tests'),'-p','test_*.py','-v'],check=True)
sources=sorted((ROOT/'rtl').glob('*.sv'))
benches=sorted((ROOT/'tests').glob('tb_*.sv'))
build=ROOT/'build/tests';build.mkdir(parents=True,exist_ok=True)
if shutil.which('iverilog') and shutil.which('vvp'):
    for bench in benches:
        out=build/(bench.stem+'.vvp')
        subprocess.run(['iverilog','-g2012','-s',bench.stem,'-o',str(out),*map(str,sources),str(bench)],check=True)
        result=subprocess.run(['vvp',str(out)],capture_output=True,text=True,timeout=90)
        print(result.stdout+result.stderr)
        if result.returncode or 'PASS' not in result.stdout:raise SystemExit(1)
else:
    tools=Path('C:/modeltech64_10.6d/win64')
    if not (tools/'vsim.exe').exists():raise SystemExit('Install Icarus or set up ModelSim')
    base=Path('C:/fpga_build');base.mkdir(exist_ok=True)
    stage=Path(tempfile.mkdtemp(prefix='mecanum_sim_',dir=base))
    for source in sources+benches:shutil.copy2(source,stage/source.name)
    subprocess.run([str(tools/'vlib.exe'),'work'],cwd=stage,check=True,capture_output=True)
    result=subprocess.run([str(tools/'vlog.exe'),'-sv',*[p.name for p in sources+benches]],cwd=stage,capture_output=True,text=True)
    (build/'compile.log').write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode:print(result.stdout+result.stderr);raise SystemExit(1)
    for bench in benches:
        result=subprocess.run([str(tools/'vsim.exe'),'-c','-onfinish','exit',f'work.{bench.stem}','-do','onerror {quit -code 1 -f}; run -all; quit -code 0 -f'],cwd=stage,capture_output=True,text=True,timeout=90)
        text=result.stdout+result.stderr
        (build/(bench.stem+'.log')).write_text(text,encoding='utf-8')
        print('\n'.join(line for line in text.splitlines() if 'PASS' in line or 'Error' in line or 'Fatal' in line))
        if result.returncode or 'PASS' not in text or '** Fatal' in text:raise SystemExit(1)
print('PASS all offline tests; no hardware tested')
