"""Observation-only wrapper around the production agent, adding raw event evidence."""
from pathlib import Path
import json, runpy, sys, threading, time
ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT), str(ROOT / 'vendor/vision'), str(ROOT / 'nano/vision')]
from shared.perception import PerceptionEvents
base_ingest = PerceptionEvents.ingest
raw = (ROOT / 'observation/raw_important_events.jsonl').open('a', buffering=65536)
lock = threading.Lock()
def logged_ingest(self, payload, now=None):
    if payload[:4] == b'OFLW':
        return base_ingest(self, payload, now)
    with lock:
        raw.write(json.dumps(dict(time=time.time(), hex=payload.hex()))+'\n')
        if payload[:4] == b'OFST':
            raw.flush()
    return base_ingest(self, payload, now)
PerceptionEvents.ingest = logged_ingest
token = json.loads((ROOT / 'session_token.json').read_text())['token']
sys.argv = [str(ROOT / 'nano/start_vision.py'), '--bind', '192.168.50.20',
    '--port', '8766', '--capture-process', '--capture-buffer-kib', '4096', '--model', 'yolov8n', '--backend', 'tensorrt',
    '--model-dir', '/home/liwei/robot_camera_yolo_dashboard_trt/models/yolov8n-coco-320',
    '--engine', '/home/liwei/robot_camera_yolo_dashboard_trt/models/yolov8n-coco-320/yolov8n-320-fp16.engine',
    '--token', token, '--out', str(ROOT / 'observation')]
try:
    runpy.run_path(str(ROOT / 'nano/start_vision.py'), run_name='__main__')
finally:
    raw.close()
