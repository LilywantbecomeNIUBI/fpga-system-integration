"""Attach a frozen, complete whole-system PnR reference; never falls back."""
from pathlib import Path
import hashlib
import json
import shutil
import re


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fallback_warning(log):
    """Gowin can override inc_pnr and silently start a clean implementation."""
    for line in log.splitlines():
        if re.search(r'\bPA2136\b',line) or re.search(r'default placement and routing flow will be run',line,re.I):
            return line.strip()
    return None


def attach(folder, baseline):
    folder=Path(folder).resolve();baseline=Path(baseline).resolve()
    if baseline.suffix.lower()=='.pr':
        previous=baseline.parent.parent.parent
        routed=baseline
    else:
        previous=baseline;routed=previous/'impl/pnr/sensor.pr'
    current=json.loads((folder/'manifest.json').read_text())
    reference=json.loads((previous/'manifest.json').read_text())
    result=json.loads((previous/'result.json').read_text())
    if not current.get('system') or not reference.get('system'):
        raise ValueError('Incremental PnR requires a whole-system reference')
    for key in ('top','pin_profile'):
        if current[key]!=reference[key]:raise ValueError('Incremental reference mismatch: '+key)
    if result.get('actual_exit_code')!=0 or result.get('timed_out'):
        raise ValueError('Reference did not complete Gowin PnR')
    if not routed.is_file() or not routed.stat().st_size:
        raise ValueError('Missing complete whole-system .pr reference')
    for name in ('sensor.cst','sensor.sdc'):
        if sha(folder/name)!=sha(previous/name):
            raise ValueError('Incremental reference constraints differ: '+name)
    destination=folder/'incremental_base/sensor.pr'
    destination.parent.mkdir(exist_ok=True)
    before=sha(routed);shutil.copy2(routed,destination)
    if sha(destination)!=before or sha(routed)!=before:
        raise RuntimeError('Incremental reference changed while freezing')
    script=folder/'build.tcl';text=script.read_text()
    if 'run all' not in text:raise ValueError('Expected a frozen whole-system build script')
    text=text.replace('run all','set_option -inc_place 0\nset_option -inc_pnr {incremental_base/sensor.pr}\nrun all',1)
    script.write_text(text)
    current['incremental_pnr']=dict(requested=True,baseline_build=str(previous),
        reference_path='incremental_base/sensor.pr',reference_sha256=before,
        baseline_bitstream_sha256=result.get('bitstream_sha256'),
        baseline_violations=result.get('violations'),constraints_unchanged=True,
        full_rebuild_fallback=False)
    (folder/'manifest.json').write_text(json.dumps(current,indent=2)+'\n')
    return current
