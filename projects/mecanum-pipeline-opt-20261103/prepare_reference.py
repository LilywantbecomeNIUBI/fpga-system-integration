"""Verify and unpack the whole-system incremental reference; no tool launch."""
from pathlib import Path
import hashlib,json,shutil,zipfile
ROOT=Path(__file__).resolve().parent

def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()

def main():
    info=json.loads((ROOT/'reference/reference.json').read_text(encoding='utf-8'))
    archive=ROOT/'reference/pnr_reference.zip'
    if not archive.exists():
        temporary=archive.with_name(archive.name+'.tmp')
        with temporary.open('wb') as dst:
            for part in info['archive_parts']:
                source=ROOT/'reference'/part['name']
                if sha(source)!=part['sha256']:raise ValueError('Reference archive part hash mismatch')
                with source.open('rb') as src:shutil.copyfileobj(src,dst)
        if sha(temporary)!=info['archive_sha256']:raise ValueError('Combined reference hash mismatch')
        temporary.replace(archive)
    if sha(archive)!=info['archive_sha256']:raise ValueError('Reference archive hash mismatch')
    manifest=json.loads((ROOT/'manifest.json').read_text(encoding='utf-8'))
    with zipfile.ZipFile(archive) as zipped:
        previous=json.loads(zipped.read('baseline_manifest.json'))
        result=json.loads(zipped.read('baseline_result.json'))
        if not previous.get('system') or result.get('actual_exit_code')!=0 or result.get('timed_out'):
            raise ValueError('Reference did not complete whole-system P&R')
        for key in ('top','pin_profile'):
            if previous[key]!=manifest[key]:raise ValueError('Reference mismatch: '+key)
        for name in ('sensor.cst','sensor.sdc'):
            if zipped.read('baseline_'+name)!=(ROOT/name).read_bytes():raise ValueError('Constraint mismatch: '+name)
        for source,target in [('sensor.pr','impl/pnr/sensor.pr'),('baseline_manifest.json','manifest.json'),
            ('baseline_result.json','result.json'),('baseline_sensor.cst','sensor.cst'),('baseline_sensor.sdc','sensor.sdc')]:
            destination=ROOT/'reference_baseline'/target
            if destination.exists():
                if source=='sensor.pr' and sha(destination)==info['reference_sha256']:continue
                if source!='sensor.pr' and destination.read_bytes()==zipped.read(source):continue
                raise ValueError('Existing reference differs: '+str(destination))
            destination.parent.mkdir(parents=True,exist_ok=True)
            temporary=destination.with_name(destination.name+'.tmp')
            with zipped.open(source) as src,temporary.open('wb') as dst:shutil.copyfileobj(src,dst)
            if source=='sensor.pr' and sha(temporary)!=info['reference_sha256']:raise ValueError('PR hash mismatch')
            temporary.replace(destination)
    print('PASS verified complete incremental reference; no synthesis/P&R started')

if __name__=='__main__':main()
