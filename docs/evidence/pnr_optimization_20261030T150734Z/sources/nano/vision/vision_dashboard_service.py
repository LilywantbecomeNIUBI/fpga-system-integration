"""Manage only this directory's dashboard agent; never stop an unrelated receiver."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request
from vision_models import MODEL_IDS,model_paths,resolve_model
BASE=Path(__file__).resolve().parent
PID=BASE/'agent_pid.json'


def owned(record):
    path=Path('/proc')/str(record['pid'])
    if not path.exists() or (path/'stat').read_text().split()[2]=='Z':return False
    if str(BASE/'vision_dashboard_agent.py').encode() not in (path/'cmdline').read_bytes().split(b'\0'):
        raise RuntimeError('Saved PID belongs to another process; refusing to stop it')
    return True


def stop_previous():
    if not PID.exists():return
    record=json.loads(PID.read_text())
    if not owned(record):return
    request=urllib.request.Request(record['url']+'/api/shutdown',data=b'',headers={'X-Session-Token':record['token']},method='POST')
    try:
        with urllib.request.urlopen(request,timeout=2):pass
    except Exception:os.kill(record['pid'],signal.SIGTERM)
    for _ in range(400):
        if not owned(record):return
        time.sleep(.1)
    raise RuntimeError('The dashboard agent has not stopped; check its log')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=('start','stop','status'))
    ap.add_argument('--bind',default='192.168.50.20');ap.add_argument('--model-dir');ap.add_argument('--token')
    ap.add_argument('--backend',choices=('tensorrt','opencv'),default='tensorrt')
    ap.add_argument('--model',choices=MODEL_IDS,default='yolov3-tiny')
    ap.add_argument('--engine')
    args=ap.parse_args()
    if args.action=='status':
        record=json.loads(PID.read_text()) if PID.exists() else None
        print(json.dumps(dict(running=bool(record and owned(record)),profile='fpga_yolo_dashboard_v1')));return
    resolve_model(args.model,args.backend)
    stop_previous()
    if args.action=='stop':print('Nano dashboard stopped');return
    if not args.model_dir or not args.token:ap.error('start requires --model-dir and --token')
    out=BASE/'runs'/time.strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=True)
    engine=args.engine or model_paths(BASE,args.model_dir,args.model)['engine']
    command=[sys.executable,str(BASE/'vision_dashboard_agent.py'),'--bind',args.bind,'--model-dir',args.model_dir,
             '--token',args.token,'--out',str(out),'--backend',args.backend,'--engine',engine,'--model',args.model]
    with (out/'agent.log').open('w') as log:
        process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    record=dict(pid=process.pid,url='http://%s:8766'%args.bind,token=args.token,output=str(out))
    PID.write_text(json.dumps(record,indent=2))
    for _ in range(400):
        if process.poll() is not None:raise RuntimeError((out/'agent.log').read_text()[-3000:])
        try:
            with urllib.request.urlopen(record['url']+'/health',timeout=.5) as response:status=json.load(response)
            if status.get('error'):
                stop_previous();raise RuntimeError(status['error'])
            if status['capture']['frames'] and status.get('result'):
                print(json.dumps(dict(pid=process.pid,url=record['url'],output=str(out),frames=status['capture']['frames'])));return
        except (OSError,ValueError):pass
        time.sleep(.15)
    stop_previous();raise RuntimeError('Nano did not receive images and produce inference; close other UDP6102 receivers and check V2/V3')


if __name__=='__main__':main()
