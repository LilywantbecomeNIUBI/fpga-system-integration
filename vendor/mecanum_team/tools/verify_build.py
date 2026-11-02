"""Verify that the delivered bitstream corresponds to the current RTL and constraints."""
from pathlib import Path
import hashlib
import json
import re
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify():
    folder=ROOT/'project'
    stage=Path((folder/'build_stage.txt').read_text(encoding='utf-8'))
    sources=sorted((ROOT/'rtl').glob('*.sv'))+[folder/'mecanum.cst',folder/'mecanum.sdc',folder/'mecanum.gprj',folder/'build.tcl']
    same=all(p.exists() and (stage/p.relative_to(ROOT)).exists() and sha(p)==sha(stage/p.relative_to(ROOT)) for p in sources)
    content=(folder/'impl/pnr/mecanum_tr_content.html').read_text(encoding='utf-8',errors='replace')
    violations={}
    for kind in ('Setup','Hold'):
        match=re.search(r'Numbers of '+kind+r' Violated Endpoints</td>\s*<td[^>]*>(\d+)',content)
        violations[kind]=int(match[1]) if match else None
    tables={}
    for title,anchor in [('setup','Setup_Slack_Table'),('hold','Hold_Slack_Table'),('recovery','Recovery_Slack_Table'),('removal','Removal_Slack_Table'),('pulse_width','MIN_PULSE_WIDTH_TABLE')]:
        section=content.split('name="'+anchor+'"',1)[1].split('</table>',1)[0]
        slacks=re.findall(r'<tr>\s*<td[^>]*>\s*\d+\s*</td>\s*<td[^>]*>\s*(-?\d+\.\d+)\s*</td>',section)
        tables[title]=min(map(float,slacks)) if slacks else None
    bit=folder/'impl/pnr/mecanum.fs'
    stage_bit=stage/'project/impl/pnr/mecanum.fs'
    bit_matches=bit.exists() and stage_bit.exists() and sha(bit)==sha(stage_bit)
    result=dict(verified_utc=datetime.now(timezone.utc).isoformat(),stage=str(stage),sources_match_build=same,
                violations=violations,worst_slack_ns=tables,bitstream_matches_build=bit_matches,
                bitstream_sha256=sha(bit) if bit.exists() else None,
                source_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in sources},
                hardware_tested=False,programmed=False)
    # Pulse width / recovery / removal are also checked, if reported.
    result['passed']=same and bit_matches and violations=={'Setup':0,'Hold':0} and all(v is not None and v>=0 for v in tables.values())
    (folder/'verification.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'},indent=2))
    return result


if __name__=='__main__':
    raise SystemExit(0 if verify()['passed'] else 1)
