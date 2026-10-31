"""Read-only live perception observation. Never opens serial or sends commands."""
import argparse,json,time,statistics,urllib.request
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--url',default='http://127.0.0.1:8875')
    ap.add_argument('--seconds',type=float,default=300);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=True);rows=[];errors=[];start=time.monotonic()
    with (args.out/'observations.jsonl').open('w') as f:
        while time.monotonic()-start<args.seconds+2:
            try:
                s=json.load(urllib.request.urlopen(args.url+'/api/state',timeout=2))
                assert s['mode']=='live' and not s['simulation'] and not s['replay']
                n=s['nano']['data'];flow=s['flow']
                row=dict(t=time.monotonic(),pc=s['pc'],nano=n['capture'],inferences=n['inferences'],
                    flow_epoch=flow['epoch'],nano_flow_epoch=(n.get('flow') or {}).get('epoch'),nano_poll_age_ms=s['nano'].get('poll_age_ms'),
                    event_receiver=n.get('event_receiver'),receiver_error=n.get('event_receiver_error'),
                    flow_frame=flow['newest_frame_id'],flow_packets=flow['packets'],control=flow['control'],
                    diagnostic=flow['diagnostic'],health_fault=flow['health_fault'],
                    motor_outputs_enabled=s['motor_outputs_enabled'],imu_online=s['imu']['online'],
                    imu_healthy=s['imu']['healthy'],imu_stats=s['imu']['statistics'])
                rows.append(row);f.write(json.dumps(row)+'\n');f.flush()
            except Exception as exc:errors.append(dict(t=time.monotonic(),error=repr(exc)))
            time.sleep(1)
    assert len(rows)>1,'No valid live samples'
    a,b=rows[0],rows[-1];seconds=b['t']-a['t'];faults=[r for r in rows if r['health_fault']]
    fresh=sum(r['control']['fresh'] for r in rows);confident=sum(r['control']['confident'] for r in rows)
    result=dict(seconds=seconds,samples=len(rows),errors=errors,
        pc_fps=(b['pc']['frames']-a['pc']['frames'])/seconds,nano_fps=(b['nano']['frames']-a['nano']['frames'])/seconds,
        inference_fps=(b['inferences']-a['inferences'])/seconds,
        pc_missing=b['pc']['missing_frames']-a['pc']['missing_frames'],nano_missing=b['nano']['missing_frames']-a['nano']['missing_frames'],
        queue_ms_p50_median=statistics.median(r['nano']['queue_ms_p50'] for r in rows),
        flow_frame_advance=(b['flow_frame']-a['flow_frame'])&65535,flow_packets=b['flow_packets']-a['flow_packets'],
        fault_samples=len(faults),first_fault=faults[0]['diagnostic'] if faults else None,
        fresh_samples=fresh,confident_samples=confident,max_grid_wait=max(r['diagnostic']['grid_wait_cycles'] for r in rows),
        direct_event_samples=sum(r['flow_epoch']!=r['nano_flow_epoch'] for r in rows),
        max_control_age_ms=max(r['control']['age_ms'] for r in rows),
        max_nano_poll_age_ms=max(r['nano_poll_age_ms'] for r in rows),
        imu_crc_delta=b['imu_stats']['crc_failures']-a['imu_stats']['crc_failures'],
        imu_gap_delta=b['imu_stats']['serial_sequence_gaps']-a['imu_stats']['serial_sequence_gaps'],
        motor_outputs_enabled=any(r['motor_outputs_enabled'] for r in rows),
        static_health_passed=seconds>=args.seconds and not errors and not faults and fresh==len(rows) and confident==len(rows)
            and all(r['imu_online'] and r['imu_healthy'] and not r['motor_outputs_enabled'] and not r['receiver_error'] for r in rows),
        dynamic_obstacle_acceptance=False)
    (args.out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
