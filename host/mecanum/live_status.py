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
        # Each GUI also owns an independent file; an old window cannot erase
        # another window's current data. Keep the legacy file for compatibility.
        process_path=self.path.with_name(self.path.stem+f'.{os.getpid()}'+self.path.suffix)
        process_tmp=process_path.with_suffix('.tmp')
        process_tmp.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        process_tmp.replace(process_path)
        temporary=self.path.with_name(self.path.name+f'.{os.getpid()}.tmp')
        temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        temporary.replace(self.path)
        self.sequence+=1;self.last_write=now


class TelemetryJournal:
    """Bounded, per-process command/feedback journal; never sends commands."""
    def __init__(self,directory):
        import logging
        from logging.handlers import RotatingFileHandler
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        self.path=directory/f'telemetry.{os.getpid()}.jsonl'
        self.logger=logging.Logger('mecanum.'+str(os.getpid()));self.logger.setLevel(logging.INFO)
        self.handler=RotatingFileHandler(self.path,maxBytes=5*1024*1024,backupCount=1,encoding='utf-8')
        self.logger.addHandler(self.handler)

    def record(self,kind,**data):
        self.logger.info(json.dumps(dict(time_utc=datetime.now(timezone.utc).isoformat(),
            monotonic=time.monotonic(),pid=os.getpid(),kind=kind,**data),ensure_ascii=False))

    def close(self):
        self.handler.close();self.logger.removeHandler(self.handler)
