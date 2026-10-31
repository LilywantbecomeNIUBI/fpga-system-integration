"""Explicit one-time import of the sensing candidate; never runs during a build."""
from pathlib import Path
import argparse,hashlib,json,shutil,subprocess
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);a=ap.parse_args()
    source=a.source.resolve();manifest=ROOT/'docs/integration_sources.json'
    if manifest.exists():raise SystemExit('Snapshot exists; review explicitly before replacing')
    git=['git','-c','safe.directory='+str(source),'-C',str(source)]
    record=dict(repository='fpga-robot',head=subprocess.check_output(git+['rev-parse','HEAD'],text=True).strip(),
        status=subprocess.check_output(git+['status','--porcelain'],text=True),files={})
    def copy(rel,dest=None):
        p=source/rel;t=ROOT/(dest or rel);t.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,t)
        record['files'][t.relative_to(ROOT).as_posix()]=dict(source=rel,sha256=sha(p))
    for directory in ('fpga/vision','fpga/imu','vendor/vision','vendor/imu','vendor/notices','tests/vision','tests/imu'):
        for p in (source/directory).rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts:copy(p.relative_to(source).as_posix())
    for rel in ('docs/source_manifest.json','docs/validation_v1.md','fpga/constraints/robot.sdc','fpga/constraints/robot.cst'):
        copy(rel,'vendor/reference/'+Path(rel).name)
    for name in ('vision_dashboard_agent.py','vision_dashboard_service.py'):copy('nano/'+name,'nano/vision/'+name)
    copy('host/dashboard.py','vendor/reference/sensing_dashboard.py');copy('host/index.html','vendor/reference/sensing_index.html')
    for name in ('ethernet_pll_model.v',):copy('tests/'+name)
    copy('host/dashboard.py','host/sensing_base.py');copy('host/index.html','host/index.html')
    manifest.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Imported',len(record['files']),'files; originals unchanged')
if __name__=='__main__':main()
