"""Require one five-bit feedback state bank in each synthesized LK instance."""
import hashlib,json,re
from pathlib import Path

def main():
    p=Path('build/system/impl/gwsynthesis/system.vg');net=p.read_text(errors='replace');rows=[]
    for name in ['flow_lk_patch','flow_lk_patch_0','vision_tracker']:
        body=net.split('module '+name+' (')[1].split('endmodule')[0]
        cells=re.findall(r'\b(DFF\w*)\s+(state_\w+)\s*\((.*?)\);',body,re.S)
        bits={}
        for typ,cell,ports in cells:
            bit=re.match(r'state_(\d+)',cell)[1]
            bits.setdefault(bit,[]).append(dict(cell=cell,type=typ,ports=ports.strip()))
        assert set(bits)==set(map(str,range(5))),(name,bits)
        assert all(len(c)==1 for c in bits.values()),(name,bits)
        rows.append(dict(module=name,state_ff_count=len(cells),bits=bits))
    result=dict(passed=True,netlist_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),instances=rows,
                limitation='Counts synthesis FFs by preserved state names; routed implementation still needs verification.')
    Path('build/tests/lk_state_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
