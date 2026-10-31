from pathlib import Path
import math, random, subprocess
ROOT=Path(__file__).resolve().parents[2]
def run():
    rng=random.Random(41031); vectors=[]
    for a,b in [(0,0),(-1,1),(-2147483648,-1),(2147483647,2147483647)]+[(rng.randint(-2**31,2**31-1),rng.randint(-2**31,2**31-1)) for _ in range(150)]:
        vectors.append((0,a,b,a*b,0))
    for a,b in [(0,1),(-2**62,65535),(2**62-1,2147483647)]+[(rng.randint(-2**62,2**62),rng.randint(1,2**31-1)) for _ in range(150)]:
        vectors.append((1,a,b,(-1 if a<0 else 1)*(abs(a)//b),0))
    for a in [0,1,2,3,4,2**64-1]+[rng.randrange(2**64) for _ in range(150)]:vectors.append((2,a,0,math.isqrt(a),0))
    for x,y in [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,-1)]+[(rng.randint(-16000,16000),rng.randint(-16000,16000)) for _ in range(180)]:
        vectors.append((3,x,y,math.degrees(math.atan2(y,x))*100,3))
    for a in range(-18000,18000,83):vectors.append((4,a,0,None,3))
    folder=ROOT/'build/task4';folder.mkdir(parents=True,exist_ok=True)
    (folder/'math_input.txt').write_text(''.join(f'{o} {a&((1<<64)-1):016x} {b&0xffffffff:08x}\n' for o,a,b,e,t in vectors))
    subprocess.run(['iverilog','-g2012','-s','tb_math','-o',str(folder/'math.vvp'),'tests/navigation/tb_math.sv','fpga/navigation/nav_math.sv'],cwd=ROOT,check=True)
    subprocess.run(['vvp',str(folder/'math.vvp')],cwd=ROOT,check=True)
    rows=(folder/'math_output.txt').read_text().splitlines();assert len(rows)==len(vectors)
    def signed(n,bits):return n-(1<<bits) if n>>(bits-1) else n
    for i,(line,(op,a,b,expected,tol)) in enumerate(zip(rows,vectors)):
        value,err=line.split();n=int(value,16);assert err=='0',(i,line)
        if op==4:
            cs= signed(n>>32,32);sn=signed(n&0xffffffff,32)
            assert abs(cs-math.cos(math.radians(a/100))*65536)<=tol,(i,a,cs,sn)
            assert abs(sn-math.sin(math.radians(a/100))*65536)<=tol,(i,a,cs,sn)
        else:
            actual=signed(n,64) if op!=2 else n
            assert abs(actual-expected)<=tol,(i,op,a,b,actual,expected)
    print('PASS',len(rows),'independent arithmetic vectors')
    return len(rows)
if __name__=='__main__':run()
