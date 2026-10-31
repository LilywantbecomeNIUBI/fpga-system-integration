"""Bound cleanup of an owned CLI after it has generated all PnR outputs."""
import subprocess,time

def run_gowin(exe,script,log_path,pnr=False):
    with log_path.open('w',encoding='utf-8') as log:
        proc=subprocess.Popen([exe,str(script)],stdout=log,stderr=subprocess.STDOUT)
        completed_at=None;terminated=False
        while proc.poll() is None:
            time.sleep(2)
            if pnr:
                text=log_path.read_text(encoding='utf-8',errors='replace')
                complete='[100%] Power analysis completed' in text and 'system.tr" completed' in text
                if complete and completed_at is None:completed_at=time.monotonic()
                if completed_at is not None and time.monotonic()-completed_at>30:
                    proc.terminate();proc.wait(timeout=15);terminated=True;break
        if proc.returncode and not terminated:raise subprocess.CalledProcessError(proc.returncode,[exe,str(script)])
    return dict(exit_code=proc.returncode,cli_terminated_after_completed_outputs=terminated)
