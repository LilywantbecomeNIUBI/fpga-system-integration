"""Prove RTL held-data/handshake ordering for the mailbox timing exceptions."""
from pathlib import Path
import hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[1]
def main():
    out=ROOT/'build/tests';src=['tests/vision/tb_mailbox_contract.v','fpga/vision/src/vision/vision_mailbox.v']
    subprocess.run(['iverilog','-g2012','-s','tb_mailbox_contract','-o',str(out/'mailbox_contract.vvp')]+src,cwd=ROOT,check=True)
    cases=[]
    for sh,dh in [(10,4),(4,10),(6.13,10.47),(10.59,3.8)]:
        p=subprocess.run(['vvp',str(out/'mailbox_contract.vvp'),f'+SRC_HALF={sh}',f'+DST_HALF={dh}'],cwd=ROOT,capture_output=True,text=True,timeout=15)
        log=p.stdout+p.stderr;assert p.returncode==0 and 'PASS mailbox' in log,log
        cases.append(dict(src_half_ns=sh,dst_half_ns=dh,passed=True,log=log));print(log)
    src+=['tools/test_mailbox_contract.py']
    record=dict(passed=True,hardware_tested=False,cases=cases,source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in src})
    (out/'mailbox_contract.json').write_text(json.dumps(record,indent=2)+'\n')
if __name__=='__main__':main()
