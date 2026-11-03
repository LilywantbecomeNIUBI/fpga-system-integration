"""Offline regressions against the namespaced, integrated RTL. No hardware IO."""
from pathlib import Path
import hashlib, json, re, subprocess, sys
ROOT=Path(__file__).resolve().parents[2]
TEAM=ROOT/'vendor/mecanum_team'
OUT=ROOT/'build/mecanum/tests'


def run(name, command, timeout=180):
    p=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=timeout)
    (OUT/(name+'.log')).write_text(p.stdout+p.stderr,encoding='utf-8')
    if p.returncode:
        raise RuntimeError(name+'\n'+p.stdout+p.stderr)
    return p.stdout


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((TEAM/'manifest.json').read_text(encoding='utf-8'))
    assert all(hashlib.sha256((TEAM/p).read_bytes()).hexdigest()==h
               for p,h in manifest['source_sha256'].items())
    run('upstream_python',[sys.executable,'-m','unittest','discover','-s',str(TEAM/'tests'),'-p','test_*.py','-v'])
    modules=manifest['module_names']
    pattern=re.compile(r'\b('+'|'.join(map(re.escape,modules))+r')\b')
    overrides=json.loads((ROOT/'fpga/mecanum/local_overrides.json').read_text())
    for p in (TEAM/'rtl').glob('*.sv'):
        if p.name!='mecanum_top.sv':
            expected=pattern.sub(lambda m:modules[m[0]],p.read_text(encoding='utf-8-sig'))
            if p.name in overrides:
                assert hashlib.sha256((ROOT/'fpga/mecanum'/p.name).read_bytes()).hexdigest()==overrides[p.name]['sha256'],p
            else:assert expected==(ROOT/'fpga/mecanum'/p.name).read_text(encoding='utf-8'),p
    rtl=list((ROOT/'fpga/mecanum').glob('*.sv'))
    rtl+=list((ROOT/'tests/mecanum/reference').glob('*.sv'))
    rtl+=[ROOT/'fpga/motor/frame_transport.sv',ROOT/'fpga/motor/uart_byte.sv',ROOT/'fpga/motor/wheel_speed_hybrid.sv']
    # Upstream test.py omits the I2C sensor model for tb_pose_serial_top.
    # Include its existing source explicitly; selecting -s prevents its other
    # top-level testbench from running as an accidental simulation root.
    model=OUT/'mpu_model.sv'
    model.write_text((TEAM/'tests/tb_mpu6050.sv').read_text(encoding='utf-8'),encoding='utf-8')
    passed=[]
    for bench in sorted((TEAM/'tests').glob('tb_*.sv')):
        adapted=OUT/bench.name
        adapted.write_text(pattern.sub(lambda m:modules[m[0]],bench.read_text(encoding='utf-8')),encoding='utf-8')
        extra=[] if bench.name=='tb_mpu6050.sv' else [model]
        output=OUT/(bench.stem+'.vvp')
        run(bench.stem+'_compile',['iverilog','-g2012','-s',bench.stem,'-o',str(output),*map(str,rtl),str(adapted),*map(str,extra)])
        text=run(bench.stem,['vvp',str(output)])
        if 'PASS' not in text:
            raise RuntimeError('No PASS: '+bench.name)
        passed.append(bench.stem)
        print(text.splitlines()[0],flush=True)
    run('integration_python',[sys.executable,'-m','unittest','discover','-s',str(ROOT/'tests/mecanum'),'-p','test_*.py','-v'])
    for bench in sorted((ROOT/'tests/mecanum').glob('tb_*.sv')):
        if bench.stem=='tb_router_mirror':
            print(run('router_mirror',[sys.executable,'tools/mecanum/test_mirror.py']),flush=True)
            passed.append(bench.stem);continue
        if bench.stem=='tb_router_commands':
            print(run('router_wire',[sys.executable,'tools/mecanum/test_router.py']),flush=True)
            passed.append(bench.stem);continue
        output=OUT/(bench.stem+'.vvp')
        run(bench.stem+'_compile',['iverilog','-g2012','-s',bench.stem,'-o',str(output),*map(str,rtl),str(bench)])
        text=run(bench.stem,['vvp',str(output)])
        if 'PASS' not in text:raise RuntimeError('No PASS: '+bench.name)
        passed.append(bench.stem)
        print(text.splitlines()[0],flush=True)
    (OUT/'result.json').write_text(json.dumps(dict(passed=True,benches=passed,
        upstream_python_tests=28,programmed=False,hardware_tested=False),indent=2)+'\n')


if __name__=='__main__':
    main()
