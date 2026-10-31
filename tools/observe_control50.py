"""Read-only concurrent board evidence; no motor, serial or network commands."""
import argparse,json,statistics,time,urllib.request
from pathlib import Path
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=float,default=300);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=True);rows=[];errors=[];start=time.monotonic()
    with (a.out/'observations.jsonl').open('w',encoding='utf-8') as log:
        while time.monotonic()-start<a.seconds+2:
            try:
                s=json.load(urllib.request.urlopen('http://127.0.0.1:8875/api/state',timeout=2))
                assert s['mode']=='live' and not s['simulation'] and not s['replay']
                n=s['nano']['data'];flow=s['flow'];result=n.get('result') or {};tracking=n.get('event') or {}
                bottles=[d for d in result.get('detections',[]) if d['class_id']==39 and d['confidence']>=.5]
                row=dict(t=time.monotonic(),pc=s['pc'],nano=n['capture'],inferences=n['inferences'],
                         flow_frame=flow['newest_frame_id'],flow_packets=flow['packets'],control=flow['control'],
                         diagnostic=flow['diagnostic'],health_fault=flow['health_fault'],tracking=tracking,
                         bottle_detections=len(bottles),feedback=result.get('feedback'),inference_ms=result.get('inference_ms'),
                         inference_frame=result.get('frame_id'),event_receiver=n.get('event_receiver'),receiver_error=n.get('event_receiver_error'),
                         flow_epoch=flow['epoch'],nano_flow_epoch=n['flow']['epoch'],nano_poll_age_ms=s['nano']['poll_age_ms'],
                         imu_online=s['imu']['online'],imu_healthy=s['imu']['healthy'],imu_stats=s['imu']['statistics'],
                         motor_outputs_enabled=s['motor_outputs_enabled'])
                rows.append(row);log.write(json.dumps(row)+'\n');log.flush()
            except Exception as exc:errors.append(dict(t=time.monotonic(),error=repr(exc)))
            time.sleep(1)
    assert len(rows)>1,'No valid live samples'
    first,last=rows[0],rows[-1];seconds=last['t']-first['t'];faults=[r for r in rows if r['health_fault']]
    fresh=sum(r['control']['fresh'] for r in rows);confident=sum(r['control']['confident'] for r in rows)
    result=dict(seconds=seconds,samples=len(rows),errors=errors,hardware_tested=True,
                pc_fps=(last['pc']['frames']-first['pc']['frames'])/seconds,
                nano_fps=(last['nano']['frames']-first['nano']['frames'])/seconds,
                inference_fps=(last['inferences']-first['inferences'])/seconds,
                pc_missing=last['pc']['missing_frames']-first['pc']['missing_frames'],
                nano_missing=last['nano']['missing_frames']-first['nano']['missing_frames'],
                queue_ms_p50_median=statistics.median(r['nano']['queue_ms_p50'] for r in rows),
                flow_frame_advance=(last['flow_frame']-first['flow_frame'])&65535,
                flow_packets=last['flow_packets']-first['flow_packets'],fault_samples=len(faults),
                first_fault=faults[0]['diagnostic'] if faults else None,
                fresh_samples=fresh,confident_samples=confident,
                max_grid_wait=max(r['diagnostic']['grid_wait_cycles'] for r in rows),
                max_solver_cycles=max(r['diagnostic']['cycles'] for r in rows),
                dropped_counter_begin=first['diagnostic']['dropped_frames'],dropped_counter_end=last['diagnostic']['dropped_frames'],
                dropped_counter_delta=(last['diagnostic']['dropped_frames']-first['diagnostic']['dropped_frames'])&0xffffffff,
                bottle_samples=sum(bool(r['bottle_detections']) for r in rows),
                tracking_valid_samples=sum(r['tracking'].get('track_valid',False) for r in rows),
                semantic_valid_samples=sum(r['tracking'].get('semantic_valid',False) for r in rows),
                target_valid_samples=sum(r['tracking'].get('target_valid',False) for r in rows),
                direct_event_samples=sum(r['flow_epoch']!=r['nano_flow_epoch'] for r in rows),
                max_control_age_ms=max(r['control']['age_ms'] for r in rows),
                imu_crc_delta=last['imu_stats']['crc_failures']-first['imu_stats']['crc_failures'],
                imu_gap_delta=last['imu_stats']['serial_sequence_gaps']-first['imu_stats']['serial_sequence_gaps'],
                motor_outputs_enabled=any(r['motor_outputs_enabled'] for r in rows),
                static_health_passed=seconds>=a.seconds and not errors and not faults and fresh==len(rows) and confident==len(rows)
                    and all(r['imu_online'] and r['imu_healthy'] and not r['motor_outputs_enabled'] and not r['receiver_error'] for r in rows),
                dynamic_obstacle_acceptance=False,
                target_blackbox_visible=False)
    result['target_load_observed']=result['bottle_samples']>0 and result['tracking_valid_samples']>0
    (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
