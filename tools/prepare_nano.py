"""Package code/config examples only; no SSH, model changes or device access."""
from pathlib import Path
import argparse,ast,hashlib,json,shutil
ROOT=Path(__file__).resolve().parents[1]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'dist/nano_system_v1');a=ap.parse_args()
    target=a.output.resolve()
    if target.exists():raise SystemExit('Package already exists; choose a new output or preserve it before rebuilding')
    target.mkdir(parents=True)
    files=[]
    for name in ('nano','shared','vendor/imu/pc','vendor/vision','vendor/notices'):
        files.extend(p for p in (ROOT/name).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    files += [ROOT/'host/motor_protocol.py',ROOT/'config/nano.example.json',ROOT/'docs/system_runbook.md']
    hashes={}
    for p in files:
        rel=p.relative_to(ROOT);dest=target/rel;dest.parent.mkdir(parents=True,exist_ok=True)
        if p.suffix=='.py':ast.parse(p.read_text(encoding='utf-8'),feature_version=(3,6))
        shutil.copyfile(p,dest);hashes[rel.as_posix()]=hashlib.sha256(dest.read_bytes()).hexdigest()
    (target/'MANIFEST.json').write_text(json.dumps(dict(files=hashes,deployed=False,models_included=False),indent=2)+'\n',encoding='utf-8')
    print('Prepared',target,'files=',len(hashes),'not deployed')
if __name__=='__main__':main()
