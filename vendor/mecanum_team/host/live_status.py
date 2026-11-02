"""Atomic, read-only telemetry export. Never opens or writes a serial port."""
import json
import os
import time
from datetime import datetime,timezone
from pathlib import Path

class LiveStatusWriter:
    def __init__(self,path,interval=.5):
        self.path=Path(path);self.interval=interval;self.last_write=-1e9;self.sequence=0

    def write(self,status,force=False):
        now=time.monotonic()
        if not force and now-self.last_write<self.interval:return
        result=dict(status,exported_at_utc=datetime.now(timezone.utc).isoformat(),
                    exporter_pid=os.getpid(),export_sequence=self.sequence+1)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.path.with_name(self.path.name+f'.{os.getpid()}.tmp')
        temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        temporary.replace(self.path)
        self.sequence+=1;self.last_write=now
