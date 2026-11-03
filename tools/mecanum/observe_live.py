"""Read GUI exports only; no network sockets or actuator commands."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,json,time
ROOT=Path(__file__).resolve().parents[2]

def select_status(folder,now=None):
    now=datetime.now(timezone.utc) if now is None else now
    fresh=[];seen=[]
    for path in folder.glob('live_status*.json'):
        try:
            row=json.loads(path.read_text(encoding='utf-8'))
            age=(now-datetime.fromisoformat(row['exported_at_utc'])).total_seconds()
            ma=row.get('motor_age_ms');pa=row.get('pose_age_ms')
            valid=(row.get('connected') and not row.get('demo') and row.get('motor') and row.get('pose')
                   and -.5<=age<=2 and ma is not None and pa is not None and max(ma,pa)+max(0,age)*1000<=1000)
            seen.append(dict(pid=row.get('exporter_pid'),export_age_s=round(age,3),motor_age_ms=ma,pose_age_ms=pa))
            if valid:fresh.append((age,row))
        except (OSError,ValueError,KeyError,TypeError):continue
    if not fresh:return dict(state='no_fresh_telemetry',sources=seen)
    fresh.sort(key=lambda item:item[0]);row=fresh[0][1]
    return dict(state='fresh',pid=row['exporter_pid'],running=row.get('running'),
                motor=row['motor'],pose=row['pose'],motor_age_ms=row['motor_age_ms'],
                pose_age_ms=row['pose_age_ms'],message=row.get('message'),
                source_pids=sorted(set(r['exporter_pid'] for _,r in fresh)))

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--seconds',type=float,default=0)
    ap.add_argument('--out',type=Path);args=ap.parse_args();samples=[];start=time.monotonic()
    while True:
        samples.append(select_status(ROOT/'host/runs'))
        if time.monotonic()-start>=args.seconds:break
        time.sleep(.2)
    result=dict(latest=samples[-1],samples=samples,read_only=True)
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['latest'],ensure_ascii=False,indent=2))

if __name__=='__main__':main()
