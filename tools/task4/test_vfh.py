from pathlib import Path
import random,subprocess
from model import vfh
ROOT=Path(__file__).resolve().parents[2]
def run():
    rng=random.Random(20261031);cases=[]
    def add(d,k=None,t=0,p=0,r=300,turn=0):cases.append((d,k if k is not None else [True]*72,t,p,r,turn))
    add([4000]*72);add([0]*72,[False]*72);add([300]*72);add([1200]*72);add([1300]*72);add([1400]*72)
    for angle in (0,1,5,17,18,35,36,53,54,65,70,71):
        for distance in (299,301,500,900,1199,1200,1300,1399,1400,1401):
            d=[4000]*72;d[angle]=distance;add(d,turn=300)
    for n in range(100):
        d=[rng.choice([500,700,1100,1300,1600,4000]) if rng.random()<.13 else 4000 for _ in range(72)]
        k=[rng.random()>.02 for _ in range(72)]
        add(d,k,rng.randrange(-17999,18000),rng.randrange(-9000,9000),rng.choice([150,250,350]),rng.choice([0,100,300,1000]))
    folder=ROOT/'build/task4'
    with (folder/'vfh_input.txt').open('w') as f:
        for d,k,t,p,r,turn in cases:
            f.write(f'{t} {p} {r} {turn} {sum(int(v)<<i for i,v in enumerate(k)):018x}\n')
            f.write(' '.join(map(str,d))+'\n')
    subprocess.run(['iverilog','-g2012','-s','tb_vfh','-o',str(folder/'vfh.vvp'),'tests/navigation/tb_vfh.sv','fpga/navigation/nav_vfh.sv'],cwd=ROOT,check=True)
    subprocess.run(['vvp',str(folder/'vfh.vvp')],cwd=ROOT,check=True)
    rows=(folder/'vfh_output.txt').read_text().splitlines();assert len(rows)==len(cases)
    binary=None;max_cycles=0
    for num,((d,k,t,p,r,turn),row) in enumerate(zip(cases,rows)):
        ref=vfh(d,k,t,p,r,turn,binary);binary=ref['binary']
        f,h,b,c,fk,rc,fm,cycles=row.split();actual=dict(found=bool(int(f)),blocked=int(b,16),cost=int(c),front_known=bool(int(fk)),rotation_clear=bool(int(rc)),front_min=int(fm))
        for key,value in actual.items():assert value==ref[key],(num,key,value,ref[key],cases[num])
        if ref['found']:assert int(h)==ref['heading'],(num,h,ref['heading'])
        max_cycles=max(max_cycles,int(cycles))
    print('PASS',len(rows),'VFH geometry/valley/cost cases; max cycles',max_cycles)
    return dict(cases=len(rows),max_cycles=max_cycles)
if __name__=='__main__':run()
