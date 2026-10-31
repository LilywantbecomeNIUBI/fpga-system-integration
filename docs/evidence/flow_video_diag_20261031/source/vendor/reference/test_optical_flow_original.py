"""Generate deterministic vectors and compare actual RTL to a scalar integer oracle."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'reference/optical_flow'))
import model

OUT = ROOT/'tests/optical_flow'
SOURCES = ['reference/optical_flow/upstream/rtl/frame_memory.sv',
           'src/vision/vision_divider.v', 'src/optical_flow/flow_lk_patch.v',
           'src/optical_flow/flow_frame_store.v', 'src/optical_flow/flow_grid.v',
           'src/optical_flow/flow_grid_pyramid.v',
           'src/optical_flow/flow_avoidance.v',
           'src/optical_flow/flow_pipeline.v']


def run(cmd, log):
    p = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True, timeout=180)
    log.write_text(p.stdout+p.stderr, encoding='utf-8')
    if p.returncode:
        raise RuntimeError(f'{cmd[0]} exit={p.returncode}: {p.stdout+p.stderr}')
    return p.stdout


def mem(path, a):
    path.write_text(''.join(f'{p:02x}\n' for row in a for p in row), encoding='ascii')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=int, choices=(0, 1, 2), default=2)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    binaries = {}
    for tb in ('patch', 'pipeline'):
        binary = OUT/f'{tb}.vvp'
        run(['iverilog', '-g2012', '-s', f'tb_flow_{tb}', '-o', str(binary),
             *SOURCES, f'sim/optical_flow/tb_flow_{tb}.v'], OUT/f'{tb}_compile.log')
        binaries[tb] = binary
    cases = [('still', 0, 0, False), ('right_half', .5, 0, False),
             ('left_half', -.5, 0, False), ('down_half', 0, .5, False),
             ('up_half', 0, -.5, False), ('right_one', 1, 0, False),
             ('diagonal', .5, -.5, False), ('flat', 1, 0, True),
             ('right_four', 4, 0, False)]
    report = []
    for name, dx, dy, flat in cases:
        folder = OUT/name
        folder.mkdir(exist_ok=True)
        a, b = model.image(64, 48, flat=flat), model.image(64, 48, dx, dy, flat=flat)
        mem(folder/'a.mem', a)
        mem(folder/'b.mem', b)
        vectors = []
        for y in range(8, 41, 8):
            for x in range(8, 57, 8):
                for seed in ((0, 0), (37, -29), (-128, 64)):
                    u, v, good = model.patch(a, b, x, y, seed)
                    vectors.append(f'{x} {y} {seed[0]} {seed[1]} {u} {v} {int(good)}\n')
        for x, y, su, sv in ((1, 1, 0, 0), (62, 46, 0, 0), (8, 8, -4096, 0)):
            u, v, good = model.patch(a, b, x, y, (su, sv))
            vectors.append(f'{x} {y} {su} {sv} {u} {v} {int(good)}\n')
        (folder/'patch.txt').write_text(''.join(vectors), encoding='ascii')
        text = run(['vvp', str(binaries['patch']), f'+CASE={folder.as_posix()}'], folder/'patch.log')
        if 'PASS' not in text:
            raise AssertionError(text)
        expected = model.grid(a, b)
        valid = [(x, y, u/128, v/128) for x, y, u, v, good in expected if good]
        if flat:
            assert not valid
        elif name == 'still':
            assert valid and all(u == 0 and v == 0 for _, _, u, v in valid)
        elif abs(dx)+abs(dy) <= 1:
            assert len(valid) >= 30
            for _, _, u, v in valid:
                if dx:
                    assert u*dx > 0, (name, u, v)
                if dy:
                    assert v*dy > 0, (name, u, v)
            assert sum(((u-dx)**2+(v-dy)**2)**.5 for _, _, u, v in valid)/len(valid) < .4
        quant = []
        for x, y, u, v in valid:
            uf, vf, g = model.patch(a, b, x, y, floating=True)
            if g:
                quant.append(((u-uf/128)**2+(v-vf/128)**2)**.5)
        report.append(dict(case=name, ground_truth=[dx, dy], valid=len(valid),
                           epe=sum(((u-dx)**2+(v-dy)**2)**.5 for _, _, u, v in valid)/max(1, len(valid)),
                           fixed_vs_float_epe=sum(quant)/max(1, len(quant)), patch_comparisons=len(vectors),
                           vectors_sha256=hashlib.sha256((folder/'patch.txt').read_bytes()).hexdigest()))
        if name == 'right_one':
            text = run(['vvp', str(binaries['pipeline']), f'+CASE={folder.as_posix()}'], folder/'pipeline.log')
            records = [list(map(int, row.split())) for row in (folder/'pipeline.txt').read_text().splitlines()]
            # Subsequent canceled pair may emit a prefix before overrun; the first pair must match in full.
            assert records[:35] == [[65535, x, y, u, v, int(g)] for x, y, u, v, g in expected]
            reverse = [[0, x, y, u, v, int(g)] for x, y, u, v, g in model.grid(b, a)]
            assert records[35:] == reverse[:len(records)-35], 'capture corrupted a frozen frame'
            counts,risk = model.regional_counts(expected,64,48)
            state,stop,steer,speed,_ = model.avoidance_decision(counts,risk)
            first_status = list(map(int,(folder/'status.txt').read_text().splitlines()[0].split()))
            assert first_status == [65535,*counts,*risk,state,int(stop),steer,speed], first_status
            print(text.strip())
        print(f'PASS {name}: {len(vectors)} bit-exact patches, valid={len(valid)}, EPE={report[-1]["epe"]:.3f}px')
    safety_checks = False
    if args.stage >= 1:
        binary = OUT/'avoidance.vvp'
        run(['iverilog', '-g2012', '-s', 'tb_flow_avoidance', '-o', str(binary),
             'src/optical_flow/flow_avoidance.v', 'sim/optical_flow/tb_flow_avoidance.v'], OUT/'avoidance_compile.log')
        text = run(['vvp', str(binary)], OUT/'avoidance.log')
        assert 'PASS' in text
        print(text.strip())
        safety_checks = True
    pyramid_cases = []
    if args.stage >= 2:
        binary = OUT/'pyramid.vvp'
        run(['iverilog', '-g2012', '-s', 'tb_flow_pipeline',
             '-Ptb_flow_pipeline.WIDTH=160', '-Ptb_flow_pipeline.HEIGHT=90', '-Ptb_flow_pipeline.PYRAMID=1',
             '-o', str(binary), *SOURCES, 'sim/optical_flow/tb_flow_pipeline.v'], OUT/'pyramid_compile.log')
        for name, dx, dy, flat in [('still',0,0,False),('right_half',.5,0,False),('right_one',1,0,False),
                                   ('right_two',2,0,False),('right_four',4,0,False),('left_four',-4,0,False),
                                   ('down_four',0,4,False),('diagonal_two',2,-2,False),('right_five',5,0,False),
                                   ('right_six',6,0,False),('flat',1,0,True),
                                   ('zoom_in',0,0,False),('zoom_out',0,0,False)]:
            folder = OUT/('pyramid_'+name)
            folder.mkdir(exist_ok=True)
            zoom=1.03 if name=='zoom_in' else .97 if name=='zoom_out' else 1
            a, b = model.image(160,90,flat=flat), model.image(160,90,dx,dy,flat=flat,zoom=zoom)
            mem(folder/'a.mem', a)
            mem(folder/'b.mem', b)
            expected = model.pyramid_grid(a,b)
            text = run(['vvp', str(binary), f'+CASE={folder.as_posix()}'], folder/'pipeline.log')
            assert 'PASS' in text
            records = [list(map(int, row.split())) for row in (folder/'pipeline.txt').read_text().splitlines()]
            assert records[:190] == [[65535,x,y,u,v,int(g)] for x,y,u,v,g in expected], name
            reverse = [[0,x,y,u,v,int(g)] for x,y,u,v,g in model.pyramid_grid(b,a)]
            assert records[190:] == reverse[:len(records)-190], (name,'owned RAM corruption')
            counts,risk = model.regional_counts(expected)
            state,stop,steer,speed,_ = model.avoidance_decision(counts,risk)
            first_status = list(map(int,(folder/'status.txt').read_text().splitlines()[0].split()))
            assert first_status == [65535,*counts,*risk,state,int(stop),steer,speed], (name,first_status,counts,risk)
            if name=='zoom_in':
                assert state==4 and stop and all(r*4>=n for n,r in zip(counts,risk)), (counts,risk)
            if name=='zoom_out':
                assert not any(risk), risk
            # Compare identical interior coordinates, and retain invalid counts separately.
            epe = {}
            for method, data in [('v0',model.grid(a,b)),('v2',expected)]:
                errors = [((u/128-dx-(zoom-1)*(x-80))**2+(v/128-dy-(zoom-1)*(y-45))**2)**.5 for x,y,u,v,g in data
                          if g and 24<=x<=136 and 16<=y<=72]
                epe[method] = dict(valid=len(errors), epe=sum(errors)/max(1,len(errors)))
            if 1<=abs(dx)+abs(dy)<=4 and not flat:
                assert epe['v2']['valid']>=100 and epe['v2']['epe']<.3, (name,epe)
                assert epe['v2']['epe']<epe['v0']['epe'], (name,epe)
            pyramid_cases.append(dict(case=name,ground_truth=[dx,dy],zoom=zoom,metrics=epe,bit_exact_points=190,
                                      pipeline_stdout=text.strip()))
            print(f'PASS pyramid {name}: V0 EPE={epe["v0"]["epe"]:.3f}, V2 EPE={epe["v2"]["epe"]:.3f}px')
    result = dict(stage=f'V{args.stage}', passed=True, hardware_tested=False, cases=report,
                  safety_checks=safety_checks, pyramid_cases=pyramid_cases)
    (OUT/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
