"""Read-only stage/CPU/artifact observations for a frozen Gowin build."""
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys
import time


def inspect(folder):
    folder=Path(folder).resolve();now=time.time()
    log=folder/'build.log';text=log.read_text(encoding='utf-8',errors='replace') if log.exists() else ''
    markers=[]
    patterns=[('synthesis',r'^.*(?:Compiling module|Running netlist conversion|Optimizing Phase|Inferring Phase|Tech-Mapping Phase|GowinSynthesis finish).*$',),
              ('placement',r'^.*(?:Running placement|Placement Phase).*$',),
              ('routing',r'^.*(?:Running routing|Routing Phase).*$',),
              ('timing',r'^.*(?:Running timing analysis|Timing analysis completed|Placement and routing completed).*$',),
              ('bitstream',r'^.*Bitstream generation.*$',),
              ('reports',r'^.*(?:Power analysis completed|Generate file .*sensor\.tr.*completed).*$',)]
    for stage,pattern in patterns:
        markers += [(m.start(),stage,m[0]) for m in re.finditer(pattern,text,re.M)]
    _,stage,marker=max(markers,default=(-1,'starting','No stage marker yet'))
    query="Get-CimInstance Win32_Process | Where-Object {$_.Name -match '^gw_(sh|syn|pnr)\\.exe$'} | Select-Object ProcessId,ParentProcessId,Name,KernelModeTime,UserModeTime,WorkingSetSize,ReadTransferCount,WriteTransferCount,ThreadCount,CommandLine | ConvertTo-Json -Compress"
    process=subprocess.run(['powershell','-NoProfile','-Command',query],capture_output=True,text=True,timeout=20)
    records=json.loads(process.stdout) if process.stdout.strip() else []
    if isinstance(records,dict):records=[records]
    own={p['ProcessId'] for p in records if str(folder).lower() in (p.get('CommandLine') or '').lower()}
    for _ in range(3):own.update(p['ProcessId'] for p in records if p.get('ParentProcessId') in own)
    processes=[]
    for p in records:
        if p['ProcessId'] in own:
            processes.append(dict(pid=p['ProcessId'],name=p['Name'],cpu_s=(int(p['KernelModeTime'] or 0)+int(p['UserModeTime'] or 0))/1e7,
                                  memory_mb=round(int(p['WorkingSetSize'] or 0)/1048576,1),
                                  read_bytes=int(p['ReadTransferCount'] or 0),write_bytes=int(p['WriteTransferCount'] or 0),thread_count=p.get('ThreadCount')))
    memory=subprocess.run(['powershell','-NoProfile','-Command',
        'Get-CimInstance Win32_OperatingSystem | Select-Object TotalVisibleMemorySize,FreePhysicalMemory | ConvertTo-Json -Compress'],capture_output=True,text=True,timeout=20)
    host_memory=json.loads(memory.stdout) if memory.stdout.strip() else None
    files={}
    for name in ('build.log','impl/gwsynthesis/sensor.vg','impl/pnr/sensor.p','impl/pnr/sensor.pr','impl/pnr/sensor.fs','impl/pnr/sensor.tr'):
        p=folder/name
        if p.exists():files[name]=dict(bytes=p.stat().st_size,modified_age_s=round(now-p.stat().st_mtime,1))
    entry=dict(observed_at=datetime.now(timezone.utc).isoformat(),stage=stage,marker=marker,processes=processes,files=files,host_memory_kib=host_memory)
    history=folder/'stage_observations.jsonl'
    with history.open('a',encoding='utf-8') as f:f.write(json.dumps(entry)+'\n')
    print(json.dumps(entry,indent=2))


if __name__=='__main__':
    inspect(sys.argv[1])
