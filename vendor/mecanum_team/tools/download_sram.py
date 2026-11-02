"""Download the verified V3 bitstream into SRAM; never sends motor commands."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
from datetime import datetime, timezone, timedelta
from verify_build import verify,ROOT


def main():
    verified=verify()
    if not verified['passed']:raise RuntimeError('Build verification failed')
    cli=Path('C:/Gowin/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe')
    # The English build path avoids Programmer's Unicode-path limitations.
    bit=Path(verified['stage'])/'project/impl/pnr/mecanum.fs'
    actual=hashlib.sha256(bit.read_bytes()).hexdigest()
    if actual!=verified['bitstream_sha256']:raise RuntimeError('Bitstream SHA256 mismatch')
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    folder=ROOT/'runs'/('sram_'+stamp);folder.mkdir(parents=True,exist_ok=False)
    record=dict(started_at_shanghai=stamp,bitstream_sha256=actual,bitstream=str(bit),
                operation='SRAM Program',flash_written=False,motion_commands_sent=False,download_confirmed=False)

    def run(args):
        command=[str(cli),*args]
        result=subprocess.run(command,capture_output=True,timeout=55)
        output=(result.stdout+result.stderr).decode('utf-8',errors='replace')
        with (folder/'programmer.log').open('a',encoding='utf-8') as log:log.write(repr(command)+'\n'+output+'\n')
        print(output,flush=True)
        if result.returncode or re.search(r'Error:',output,re.I):raise RuntimeError('Programmer reported an error')
        return output

    try:
        scan=run(['--scan-cables','F'])
        locations=re.findall(r'USB location:\s*(\d+)',scan)
        if len(locations)!=1:raise RuntimeError('Expected exactly one download cable')
        cable=['--cable-index','1','--location',locations[0]]
        detected=run([*cable,'--scan'])
        if not re.search(r'(?:0x)?0011081B',detected,re.I) or not re.search(r'1 device\(s\) found',detected):
            raise RuntimeError('Unexpected JTAG chain/device')
        output=run([*cable,'--device','GW5AT-138B','--operation_index','2','--fsFile',str(bit)])
        if 'Finished.' not in output:raise RuntimeError('Missing successful completion marker')
        record['download_confirmed']=True
    finally:
        (folder/'download_result.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print('SUCCESS: SRAM programmed; flash untouched; no ARM/DRIVE sent.',flush=True)
    print('Record:',folder/'download_result.json',flush=True)


if __name__=='__main__':main()
