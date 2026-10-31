"""Judge supplied stationary/moving metrics; never fabricates acceptance data."""
from pathlib import Path
import argparse,json,math
REQUIRED=('video_fps','yolo_fps','imu_hz','lidar_hz')
def number(value):return type(value) in (int,float) and math.isfinite(value)
def evaluate(baseline,moving):
    errors=[];ratios={}
    for key in REQUIRED:
        a=baseline.get(key);b=moving.get(key)
        if not number(a) or not number(b) or a<=0 or b<0:errors.append('Missing/invalid '+key);continue
        ratios[key]=b/a
        if ratios[key]<.9:errors.append(key+' below 90% baseline')
    if not number(moving.get('duration_s')) or moving['duration_s']<1800:errors.append('Less than 30 minutes')
    if moving.get('hardware_tested') is not True or moving.get('same_configuration') is not True:errors.append('Hardware/configuration evidence missing')
    for key in ('estop_passed','command_timeout_passed','motor_status_timeout_passed','release_stop_passed','no_auto_rearm_passed'):
        if moving.get(key) is not True:errors.append('Protection not passed: '+key)
    for key in ('record_drops','record_errors','lidar_crc_errors','uart_crc_errors','control_deadline_misses'):
        if type(moving.get(key)) is not int or moving[key]!=0:errors.append('Missing/nonzero '+key)
    # Acceptance limits must be chosen from actual commissioning, never silently defaulted.
    for key in ('stop_latency_ms','wheel_speed_error_pct'):
        value=moving.get(key);limit=moving.get(key+'_limit')
        if not number(value) or not number(limit) or value<0 or limit<=0 or value>limit:errors.append('Unmet/unset '+key)
    return dict(passed=not errors,ratios=ratios,errors=errors)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--baseline',type=Path,required=True);ap.add_argument('--moving',type=Path,required=True);a=ap.parse_args()
    result=evaluate(json.loads(a.baseline.read_text()),json.loads(a.moving.read_text()));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
if __name__=='__main__':main()
