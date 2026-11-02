"""Count actual synthesized reset loads through hierarchy and polarity inverters."""
import argparse,collections,json,re
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);a=ap.parse_args()
    text=(a.build/'impl/gwsynthesis/sensor.vg').read_text()
    modules={}
    for m in re.finditer(r'\bmodule (\w+) \((.*?)endmodule',text,re.S):
        body=m[2];dims={}
        for decl in re.finditer(r'^(?:input|output|wire)\s+(?:\[(\d+):(\d+)\]\s+)?(\\?[^;]+);',body,re.M):
            if decl[1]:dims[decl[3].strip()]=(int(decl[1]),int(decl[2]))
        cells=[]
        for c in re.finditer(r'^  (\w+) (\\\S+|\w+)\s+\((.*?)^\);',body,re.M|re.S):
            ports=dict((p[1],p[2].strip()) for p in re.finditer(r'\.(\w+)\(([^)]+)\)',c[3]))
            cells.append((c[1],c[2],ports))
        modules[m[1]]=(dims,cells)
    def bits(expr,dims):
        expr=expr.strip()
        if expr.startswith('{'):return sum((bits(x,dims) for x in expr[1:-1].split(',')),[])
        m=re.fullmatch(r'(.+?)\s*\[(\d+)(?::(\d+))?\]',expr)
        if m:
            name=m[1].strip();hi=int(m[2]);lo=int(m[3] or m[2]);step=1 if lo>=hi else -1
            return [name+'['+str(i)+']' for i in range(hi,lo+step,step)]
        if expr in dims:
            hi,lo=dims[expr];step=1 if lo>=hi else -1
            return [expr+'['+str(i)+']' for i in range(hi,lo+step,step)]
        return [expr]
    def loads(mod,net,path,seen=None):
        seen=set() if seen is None else set(seen)
        if (mod,net,path) in seen:raise RuntimeError('Reset loop')
        seen.add((mod,net,path));dims,cells=modules[mod];rows=[]
        for typ,name,ports in cells:
            if typ in modules:
                subdims=modules[typ][0]
                for port,expr in ports.items():
                    actual=bits(expr,dims)
                    if net not in actual:continue
                    formal=bits(port,subdims)
                    assert len(formal)==len(actual),(typ,port,actual,formal)
                    for index,b in enumerate(actual):
                        if b==net:rows+=loads(typ,formal[index],path+'/'+name.strip('\\'),seen)
            elif typ=='INV' and net in bits(ports.get('I',''),dims):
                rows+=loads(mod,ports['O'],path,seen)
            else:
                for port,expr in ports.items():
                    if port in ('O','Q','F','V','G'):continue
                    if net in bits(expr,dims):rows.append(dict(cell=path+'/'+name.strip('\\'),type=typ,pin=port))
        return rows
    cam=next(k for k in modules if k=='camera_perception')
    dims,cells=modules[cam]
    reset=next(c for c in cells if c[1]=='perception_reset')
    qnets=[]
    rdims,rcells=modules[reset[0]]
    # Map the synchronizer's output-port bits back to camera-level nets.
    for typ,name,ports in rcells:
        if typ.startswith('DFF') and re.match(r'release_pipe_(?:[2-9]|1[01])_s',name):
            q=ports['Q'];mapped=None
            for port,expr in reset[2].items():
                formal=bits(port,rdims)
                if q in formal:mapped=bits(expr,dims)[formal.index(q)]
            assert mapped,(name,q)
            qnets.append((name,mapped))
    rows=[]
    for ff,net in qnets:
        sinks=loads(cam,net,'camera')
        rows.append(dict(flipflop=ff,net=net,fanout=len(sinks),clear_pins=sum(s['pin']=='CLEAR' for s in sinks),sinks=sinks))
    result=dict(passed=len(rows)==10 and max(r['fanout'] for r in rows)<=200,branches=rows,
                maximum_fanout=max(r['fanout'] for r in rows),note='Synthesis leaf input loads through reset polarity inverters; routed fanout is checked separately.')
    (a.build/'reset_distribution_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='branches'},indent=2))
    print([(r['flipflop'],r['net'],r['fanout']) for r in rows])
    assert result['passed']
if __name__=='__main__':main()
