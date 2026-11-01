"""Verify actual synthesized processing clocks and FIFO synchronizer stages."""
import argparse,hashlib,json,re
from pathlib import Path
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--netlist',type=Path,default=Path('build/system/impl/gwsynthesis/system.vg'));ap.add_argument('--output',type=Path,default=Path('build/tests/control50_audit.json'));args=ap.parse_args()
    p=args.netlist;text=p.read_text();rows=[]
    names=['vision_tracker','flow_observer','flow_pipeline','flow_frame_store','flow_grid_pyramid',
           'flow_lk_patch','flow_avoidance','flow_event_source','vision_feature_grid','vision_divider','frame_memory']
    bodies=dict(re.findall(r'\bmodule (\w+) \((.*?)endmodule',text,re.S))
    for prefix in names:
        found=[(n,b) for n,b in bodies.items() if re.fullmatch(re.escape(prefix)+r'(?:_\d+)?',n)]
        assert found,prefix
        for name,body in found:
            assert 'camera_pclk_d' not in body,(name,'PCLK leaked into processing')
            clocks=re.findall(r'\.(?:CLK|CLKA|CLKB)\(([^)]+)\)',body)
            assert all(all(t in ('clk50','GND','VCC') for t in re.findall(r'\w+',c)) for c in clocks),(name,set(clocks))
            rows.append(dict(module=name,clock_connections=len(clocks),clocks=sorted(set(clocks))))
    fifo=bodies['vision_async_fifo'];stages=[]
    for stem,clk in [('ws0','clk50'),('ws1','clk50'),('rs0','camera_pclk_d'),('rs1','camera_pclk_d')]:
        cells=re.findall(r'\bDFF\w*\s+('+stem+r'_\d+_s\d+)\s*\((.*?)\);',fifo,re.S)
        assert len(cells)==9,(stem,len(cells))
        assert all('.CLK('+clk+')' in ports for _,ports in cells),(stem,clk)
        stages.append(dict(stage=stem,count=len(cells),clock=clk))
    # One RAM write port in PCLK, independent RAM read port in control50.
    assert '.CLKA(camera_pclk_d)' in fifo and '.CLKB(clk50)' in fifo
    result=dict(passed=True,netlist_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),processing=rows,fifo_stages=stages,
                limitation='Synthesis connectivity audit; physical CDC delay/clock routing are checked separately.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
