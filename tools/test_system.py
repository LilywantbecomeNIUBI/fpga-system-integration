"""System regressions. --full requires local Gowin digital primitive models."""
from pathlib import Path
import argparse,hashlib,json,os,subprocess,sys,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
def run(name,args,timeout=240):
    out=ROOT/'build/tests';out.mkdir(parents=True,exist_ok=True)
    try:r=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        output=exc.stdout or b'';output=output.decode('utf-8',errors='replace') if isinstance(output,bytes) else output
        (out/(name+'_timeout.log')).write_text(output,encoding='utf-8');raise RuntimeError(name+' timeout\n'+output[-2500:]) from exc
    log=r.stdout+r.stderr;(out/(name+'.log')).write_text(log,encoding='utf-8')
    if r.returncode or ('vvp' in Path(args[0]).name and ('PASS' not in log or 'FATAL' in log)):raise RuntimeError(name+'\n'+log[-3500:])
    print(name+': '+log[-1400:],flush=True)
    return dict(passed=True,log_sha256=hashlib.sha256(log.encode('utf-8')).hexdigest())
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--full',action='store_true');a=ap.parse_args()
    record=dict(hardware_tested=False,cases={})
    record['cases']['unit_and_motor']=run('unit_and_motor',[sys.executable,'tools/test_motor.py'])
    from shared_check import check
    check()
    sys.path.insert(0,str(ROOT/'shared'));sys.path.insert(0,str(ROOT/'vendor/vision'))
    import lk_model
    data=ROOT/'build/tests/vision_lk';data.mkdir(parents=True,exist_ok=True)
    for index,dx in enumerate((0,.5,1,2,0,1)):
        image=lk_model.image(80,45,dx=dx,flat=index==4);red=0 if index==5 else 16
        (data/('frame_%d.hex'%index)).write_text(''.join('%04x\n'%((red<<8)|v) for row in image for v in row))
    original=(ROOT/'tests/vision/tb_vision_lk_tracker.v').read_text(encoding='utf-8')
    bench=ROOT/'build/tests/tb_lk_adapted.v';bench.write_text(original.replace('tests/vision_lk','build/tests/vision_lk'),encoding='utf-8')
    run('lk_compile',['iverilog','-g2012','-s','tb_vision_lk_tracker','-o','build/tests/lk.vvp',str(bench),'fpga/vision/src/vision/vision_lk_tracker.v','fpga/vision/src/optical_flow/flow_lk_patch.v','fpga/vision/src/vision/vision_divider.v'])
    record['cases']['lk']=run('lk',['vvp','build/tests/lk.vvp'])
    original=(ROOT/'vendor/vision/test_network_dual_original.py').read_text(encoding='utf-8')
    script=ROOT/'build/tests/dual_adapted.py'
    adapted=original.replace('ROOT=Path(__file__).resolve().parents[1]','ROOT=Path('+repr(str(ROOT))+')').replace('tests/network_dual_20261014','build/tests/network_dual_20261014').replace("'sim/tb_network_dual.v'","'tests/vision/tb_network_dual.v'").replace("'src/","'fpga/vision/src/")
    # The bench uses the same fixture directory literal as its upstream harness.
    dual=(ROOT/'tests/vision/tb_network_dual.v').read_text(encoding='utf-8').replace('tests/network_dual_20261014','build/tests/network_dual_20261014')
    (ROOT/'build/tests/tb_dual_adapted.v').write_text(dual,encoding='utf-8')
    adapted=adapted.replace("'tests/vision/tb_network_dual.v'","'build/tests/tb_dual_adapted.v'");script.write_text(adapted,encoding='utf-8')
    record['cases']['dual']=run('dual',[sys.executable,str(script)])
    sys.path.insert(0,str(ROOT));from shared.streams import MixedStream
    rows=MixedStream().feed((ROOT/'build/tests/mixed_uart.bin').read_bytes())
    assert len([r for k,r in rows if k=='motor'])>=3 and len([r for k,r in rows if k=='imu'])>=10
    if a.full:
        from build_system import prepare
        folder,sources=prepare();sources=[str(p) for p in sources if p.suffix in ('.v','.sv') and p.name!='ethernet_pll.v']
        prim=str(Path(os.environ.get('GOWIN_HOME','C:/Gowin/Gowin_V1.9.12.04_x64'))/'IDE/simlib/gw5a/prim_sim.v')
        run('full_compile',['iverilog','-g2012','-s','tb_full_system','-s','GSR','-o','build/tests/full_system.vvp','tests/tb_full_system.sv','tests/imu/icm42688_model.sv','tests/ethernet_pll_model.v',prim]+sources)
        record['cases']['full_top']=run('full_top',['vvp','build/tests/full_system.vvp'],timeout=300)
        parser=MixedStream();full=parser.feed((ROOT/'build/tests/full_system_uart.bin').read_bytes())
        assert parser.bad_crc==0 and {kind for kind,row in full}=={'motor','imu'}
        record['cases']['full_top']['decoded_records']=len(full)
    paths=[]
    for directory in ('fpga','host','nano','shared','tests','tools','config'):
        paths.extend(p for p in (ROOT/directory).rglob('*') if p.is_file() and p.suffix in ('.py','.sv','.v','.html','.json') and '__pycache__' not in p.parts)
    record['source_sha256']={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    record['passed']=True;(ROOT/('build/tests/system_full.json' if a.full else 'build/tests/system.json')).write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print('PASS system regressions',list(record['cases']))
if __name__=='__main__':main()
