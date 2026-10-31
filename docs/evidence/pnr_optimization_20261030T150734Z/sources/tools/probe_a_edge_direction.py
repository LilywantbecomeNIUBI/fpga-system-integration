"""One attended, bounded low-target closed-loop probe via the sole gateway."""
from pathlib import Path
import argparse,json,time,urllib.request,urllib.error,sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from shared.encoder_analysis import summarize_rows

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--power-on-and-attended-confirmed',action='store_true')
    ap.add_argument('--kp',type=int,choices=(1024,2048,4096),default=1024)
    ap.add_argument('--left-motor-reverse',action='store_true')
    ap.add_argument('--camera-index',type=int,help='Record local camera synchronously; camera must deliver a frame before ARM')
    ap.add_argument('--expected-left-decoder','--expected-decoder',choices=('a','b','recovery'),help='Require the selected wheel diagnostic mode before ARM')
    ap.add_argument('--left-target','--target',type=int,choices=(-8,8),default=-8)
    ap.add_argument('--wheel',choices=('left','right'),default='left')
    ap.add_argument('--ki',type=int,choices=(0,16,32,64,128),default=0)
    ap.add_argument('--duration-ms',type=int,choices=(300,1000),default=300)
    ap.add_argument('--gateway-config',type=Path,help='Local gateway_url/token_file config for Nano HTTP')
    ap.add_argument('--output-dir',type=Path)
    args=ap.parse_args()
    if not args.power_on_and_attended_confirmed:ap.error('Requires operator confirmation of power, suspended wheels and attendance')
    side=args.wheel;other='right' if side=='left' else 'left';axis='1' if side=='left' else '2'
    folder=args.output_dir or Path((ROOT/'runs/commissioning/speed_live_latest.txt').read_text().strip())
    folder.mkdir(parents=True,exist_ok=True)
    gateway_url='http://127.0.0.1:8873'
    token_file=folder/'session_token.json'
    if args.gateway_config:
        cfg=json.loads(args.gateway_config.read_text(encoding='utf-8-sig'))
        gateway_url=cfg['gateway_url'].rstrip('/')
        token_file=(args.gateway_config.resolve().parent/cfg['token_file']).resolve()
    token=json.loads(token_file.read_text())['token']
    output=folder/('a_edge_direction_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
    def call(path,data=None):
        req=urllib.request.Request(gateway_url+path,
            data=json.dumps(data).encode() if data is not None else None,
            headers={'X-Session-Token':token,'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=.12) as response:return json.load(response)
    rows=[];lease=None;seq=0;reason='not_started';before=None;after=None;camera=None;camera_result=None
    def action(name,**values):
        nonlocal seq
        seq+=1
        return call('/api/control',dict(action=name,lease=lease,sequence=seq,**values))
    def observe():
        if camera and (camera.error or time.monotonic()-camera.times[-1]>.3):
            raise RuntimeError('Camera unavailable or stale during probe')
        s=call('/api/state');c=s['control'];m=c.get('motor')
        rows.append(dict(monotonic=time.monotonic(),control=c,encoder_diagnostics=s.get('encoder_diagnostics',{}),
                         encoder_traces=s.get('encoder_traces',{})))
        if not c['motor_fresh'] or not m:raise RuntimeError('stale telemetry')
        return c,m
    try:
        c,before=observe()
        if c['owned'] or before['armed'] or before['left_applied'] or before['right_applied']:
            raise RuntimeError('requires unowned STOP with zero output')
        if before['faults'] not in (0,1):raise RuntimeError('existing non-boot fault; no automatic clear')
        if abs(before['left_velocity'])>1 or abs(before['right_velocity'])>1:raise RuntimeError('wheels not stationary')
        if args.expected_left_decoder:
            diag=rows[-1]['encoder_diagnostics'].get(axis,{})
            expected_mode={'a':'a_edge_x2_scaled4','b':'b_edge_x2_scaled4','recovery':'quadrature_continuity_x4'}[args.expected_left_decoder]
            if diag.get('age_ms',float('inf'))>=600 or diag.get('mode')!=expected_mode:
                raise RuntimeError('Expected left decoder mode not freshly confirmed')
        if args.camera_index is not None:
            from wheel_camera import WheelCamera
            camera=WheelCamera(args.camera_index,output.with_suffix(''))
        lease=call('/api/control',dict(action='claim'))['lease']
        action('arm',flags=3 if args.left_motor_reverse else 1,kp=args.kp,ki=args.ki)
        deadline=time.monotonic()+1.5
        while True:
            action('heartbeat',held=False)
            c,m=observe()
            if c['phase']=='active':break
            if c['phase']=='stopped' or time.monotonic()>deadline:raise RuntimeError('ARM handshake failed')
            time.sleep(.035)
        action('test',left=args.left_target if side=='left' else 0,right=args.left_target if side=='right' else 0,duration_ms=args.duration_ms)
        reason='duration_complete';deadline=time.monotonic()+args.duration_ms/1000+.35
        while True:
            try:
                action('heartbeat',held=False)
            except urllib.error.HTTPError as exc:
                # The bounded test intentionally revokes its lease at expiry.
                c,m=observe()
                if exc.code==409 and c['phase']=='stopped' and c['reason']=='timed_test_complete':
                    break
                raise
            c,m=observe()
            if m['faults']:raise RuntimeError('FPGA fault '+str(m['faults']))
            displacement=m[side+'_position']-before[side+'_position']
            if displacement*(1 if args.left_target>0 else -1)<-4:raise RuntimeError('opposite feedback direction')
            if abs(displacement)>(64 if args.duration_ms==300 else 1000):raise RuntimeError('probe displacement bound')
            if abs(m[other+'_position']-before[other+'_position'])>4:raise RuntimeError('unexpected other-wheel feedback')
            if abs(m[side+'_velocity'])>30 or abs(m[side+'_applied'])>(160 if args.kp==4096 else 100):raise RuntimeError('probe speed/output bound')
            if c['phase']=='stopped':break
            if time.monotonic()>deadline:raise RuntimeError('test did not stop on schedule')
            time.sleep(.035)
    except Exception as exc:
        reason=str(exc)
    finally:
        try:call('/api/control',dict(action='stop'))
        except Exception as exc:reason+='; STOP request failed: '+str(exc)
        for _ in range(10):
            time.sleep(.05)
            try:_,after=observe()
            except Exception:pass
        record=dict(reason=reason,before=before,after=after,rows=rows,
            config=dict(flags=3 if args.left_motor_reverse else 1,kp=args.kp,ki=args.ki,
                left=args.left_target if side=='left' else 0,right=args.left_target if side=='right' else 0,duration_ms=args.duration_ms),
            bitstream_sha256=(json.loads((ROOT/'runs/commissioning/active_motor_sram.json').read_text()).get('bitstream_sha256')
                if (ROOT/'runs/commissioning/active_motor_sram.json').exists() else None),
            scope='Single attended suspended-wheel direction probe; not PI tuning acceptance')
        if camera:
            try:camera_result=camera.close()
            except Exception as exc:camera_result=dict(error=str(exc))
        record['camera']=camera_result
        record['encoder_analysis']=summarize_rows(rows)
        output.write_text(json.dumps(record,indent=2),encoding='utf-8')
        print(json.dumps(dict(file=str(output),reason=reason,before=before,after=after,
            monitored_wheel=side,
            max_test_pwm=max((abs(r['control']['motor'][side+'_applied']) for r in rows if r['control'].get('motor')),default=0)),indent=2))

if __name__=='__main__':main()
