"""Snapshot a teammate project without touching it; generate namespaced RTL.

Run once into an empty destination. The manifest records exact upstream bytes.
No hardware IO and no vendor build invocation.
"""
from pathlib import Path
import argparse, hashlib, json, re, shutil

ROOT = Path(__file__).resolve().parents[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source', type=Path)
    args = ap.parse_args()
    src = args.source.resolve()
    snapshot = ROOT/'vendor/mecanum_team'
    if snapshot.exists():
        raise SystemExit('Snapshot exists; do not overwrite a reviewed import')
    files = []
    for directory in ('rtl', 'tests', 'host', 'config', 'hardware', 'docs', 'project', 'tools'):
        files += [p for p in (src/directory).rglob('*') if p.is_file()
                  and '__pycache__' not in p.parts and p.suffix in
                  ('.sv', '.v', '.py', '.json', '.csv', '.md', '.sdc', '.cst', '.gprj', '.tcl')]
    files += [src/'README.md', src/'requirements.txt']
    hashes = {}
    for p in files:
        relative = p.relative_to(src)
        dest = snapshot/relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, dest)
        hashes[relative.as_posix()] = hashlib.sha256(dest.read_bytes()).hexdigest()
    # Preserve source instructions as provenance, without making the snapshot
    # a second active workspace with ambiguous inherited instructions.
    shutil.copyfile(src/'AGENTS.md', snapshot/'SOURCE_INSTRUCTIONS.md')
    (snapshot/'.gitattributes').write_text('* -text\n', encoding='utf-8')
    modules = {}
    for p in (snapshot/'rtl').glob('*.sv'):
        for name in re.findall(r'^module\s+(\w+)', p.read_text(encoding='utf-8-sig'), re.M):
            modules[name] = 'mc_'+name
    pattern = re.compile(r'\b('+'|'.join(map(re.escape, modules))+r')\b')
    target = ROOT/'fpga/mecanum'
    target.mkdir(parents=True, exist_ok=True)
    # Keep frame/UART/CORDIC versions isolated too; standalone tests exercise
    # the same imported implementation rather than a similarly named library.
    for p in (snapshot/'rtl').glob('*.sv'):
        text = pattern.sub(lambda m: modules[m[0]], p.read_text(encoding='utf-8-sig'))
        (target/p.name).write_text(text, encoding='utf-8')
    (snapshot/'manifest.json').write_text(json.dumps(dict(
        source_directory=str(src), imported_on='2026-11-03',
        source_sha256=hashes, module_names=modules,
        hardware_verified_by_importer=False, programmed=False), indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print('Imported',len(files),'files; namespaced',len(modules),'modules')


if __name__ == '__main__':
    main()
