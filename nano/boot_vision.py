"""Systemd entry for the existing vision agent; models are never downloaded."""
from pathlib import Path
import argparse
import json
import os
import runpy
import secrets
import sys

ROOT = Path(__file__).resolve().parents[1]


def arguments(config):
    model_dir, engine = Path(config['model_dir']), Path(config['engine'])
    if not model_dir.is_dir() or not engine.is_file():
        raise ValueError('Configure the existing Nano model directory and TensorRT engine')
    return [str(ROOT / 'nano/start_vision.py'), '--bind', config.get('bind', '192.168.50.20'),
            '--port', str(config.get('port', 8766)), '--fpga', config.get('fpga', '192.168.50.30'),
            '--model', config.get('model', 'yolov8n'), '--backend', config.get('backend', 'tensorrt'),
            '--model-dir', str(model_dir), '--engine', str(engine),
            '--capture-process', '--events-process', '--token',
            os.environ.get('VISION_TOKEN') or secrets.token_hex(24), '--out', config.get('out', 'runs/vision')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    sys.argv = arguments(json.loads(args.config.read_text(encoding='utf-8')))
    runpy.run_path(str(ROOT / 'nano/start_vision.py'), run_name='__main__')


if __name__ == '__main__':
    main()
