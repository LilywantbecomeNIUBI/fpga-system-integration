"""Model identity and deployment paths shared by PC and Python 3.6 Nano."""
from pathlib import PurePosixPath

MODEL_IDS = ('yolov3-tiny', 'yolov8n')
BACKEND_SOURCES = ('vision_tensorrt.py', 'vision_dashboard_common.py', 'vision_models.py')


def resolve_model(requested, backend='tensorrt', default='yolov3-tiny'):
    model_id = requested or ('yolov3-tiny' if backend == 'opencv' else default)
    if model_id not in MODEL_IDS:
        raise ValueError('Unknown detection model: ' + str(model_id))
    if model_id == 'yolov8n' and backend != 'tensorrt':
        raise ValueError('YOLOv8n requires TensorRT; use yolov3-tiny for OpenCV CPU fallback')
    return model_id


def model_paths(remote_dir, tiny_dir, model_id):
    remote_dir = PurePosixPath(remote_dir)
    if model_id == 'yolov3-tiny':
        return dict(directory=str(tiny_dir), engine=str(remote_dir/'models/yolov3-tiny-320-fp16.engine'),
                    verification=str(remote_dir/'models/offline_verification.json'))
    if model_id == 'yolov8n':
        directory = remote_dir/'models/yolov8n-coco-320'
        return dict(directory=str(directory), engine=str(directory/'yolov8n-320-fp16.engine'),
                    verification=str(directory/'offline_verification.json'))
    raise ValueError('Unknown detection model: ' + str(model_id))
