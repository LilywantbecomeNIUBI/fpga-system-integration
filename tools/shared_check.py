from pathlib import Path
import hashlib,json,re,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
def check():
    manifest=json.loads((ROOT/'docs/integration_sources.json').read_text(encoding='utf-8'))
    for rel,row in manifest['files'].items():
        if not row.get('adapted'):assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==row['sha256'],rel
    from build_system import prepare
    folder,sources=prepare()
    for p in sources:p.resolve().relative_to(ROOT);assert p.is_file(),p
    cst=(folder/'system.cst').read_text(encoding='utf-8');pins=re.findall(r'IO_LOC "([^"]+)" (\w+);',cst)
    assert len(pins)==len(dict(pins))==len(set(b for _,b in pins))
    ports=set(re.findall(r'\b(?:left|right)_enc_[ab]\b',(ROOT/'fpga/system/system_top.sv').read_text()))
    assert ports.issubset(dict(pins)) and 'm2_pwm_al' not in dict(pins)
    sdc=(folder/'system.sdc').read_text(encoding='utf-8')
    assert 'period 20 ' in sdc and 'period 11.9047 ' in sdc and 'period 8 ' in sdc
    assert 'multicycle' not in sdc
    (ROOT/'build/system/pin_audit.json').write_text(json.dumps(dict(count=len(pins),pins=dict(pins),unique=True,hardware_checked=False),indent=2)+'\n')
    print('PASS immutable sources, self-contained project and',len(pins),'unique integrated pins')
if __name__=='__main__':check()
