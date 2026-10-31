"""COCO YOLOv8n / YOLOv3-tiny on JetPack 4 / TensorRT 8.2.

The convolutional graph runs on the GPU. NumPy decodes the two YOLO heads
with OpenCV 4.1.1 Region semantics; shared dashboard code applies final NMS.
CUDA resources belong to the thread that creates this detector.
"""
import ctypes as ct
import ctypes.util
import hashlib
import json
import platform
from pathlib import Path

import cv2
import numpy as np

FORMAT = 'dashboard-yolov3-tiny-trt-v1'
SIZE = 320


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_cfg(path):
    blocks = []
    for raw in Path(path).read_text().splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        if line.startswith('[') and line.endswith(']'):
            blocks.append(dict(type=line[1:-1]))
        else:
            key, value = line.split('=', 1)
            blocks[-1][key.strip()] = value.strip()
    if not blocks or blocks[0]['type'] != 'net':
        raise ValueError('Missing Darknet [net] block')
    return blocks


def fingerprint(directory, version, model_id='yolov3-tiny'):
    directory = Path(directory)
    names=('yolov3-tiny.cfg', 'yolov3-tiny.weights', 'coco.names') if model_id=='yolov3-tiny' else ('yolov8n.onnx','coco.names','export.json')
    return dict(format=FORMAT if model_id=='yolov3-tiny' else 'dashboard-yolov8n-trt-v1', input_shape=[1, 3, SIZE, SIZE], tensorrt=version,
                machine=platform.machine(),
                sources={name: sha256(directory / name) for name in names})


def build_yolov8_engine(directory, destination, precision='fp16', workspace_mb=128):
    import tensorrt as trt
    directory,destination=Path(directory),Path(destination)
    if not trt.__version__.startswith('8.2.'):
        raise RuntimeError('This builder targets TensorRT 8.2 on JetPack 4')
    export=json.loads((directory/'export.json').read_text())
    if export['onnx_sha256']!=sha256(directory/'yolov8n.onnx') or export['input_shape']!=[1,3,SIZE,SIZE]:
        raise ValueError('YOLOv8 ONNX export fingerprint mismatch')
    logger=trt.Logger(trt.Logger.WARNING)
    builder=trt.Builder(logger)
    network=builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser=trt.OnnxParser(network,logger)
    if not parser.parse((directory/'yolov8n.onnx').read_bytes()):
        raise RuntimeError('ONNX parse failed: '+ '\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    if network.num_inputs!=1 or tuple(network.get_input(0).shape)!=(1,3,SIZE,SIZE) or network.num_outputs!=1:
        raise ValueError('Expected fixed 320x320, batch-one, raw YOLOv8 detection graph')
    output=network.get_output(0)
    if tuple(output.shape)!=(1,84,2100):raise ValueError('Unexpected YOLOv8 output: '+str(output.shape))
    config=builder.create_builder_config();config.max_workspace_size=workspace_mb*1024*1024
    if precision=='fp16':
        if not builder.platform_has_fast_fp16:raise RuntimeError('GPU does not report fast FP16')
        config.set_flag(trt.BuilderFlag.FP16)
    elif precision!='fp32':raise ValueError('Unsupported precision')
    print('Building YOLOv8n COCO TensorRT '+precision+' on this Nano...',flush=True)
    engine=builder.build_engine(network,config)
    if engine is None:raise RuntimeError('YOLOv8 TensorRT engine construction failed')
    serialized=bytes(engine.serialize())
    metadata=fingerprint(directory,trt.__version__,'yolov8n')
    metadata.update(model_id='yolov8n',precision=precision,input_name=network.get_input(0).name,
                    output_name=output.name,output_shape=[1,84,2100],
                    engine_sha256=hashlib.sha256(serialized).hexdigest())
    destination.parent.mkdir(parents=True,exist_ok=True)
    temporary=destination.with_suffix(destination.suffix+'.tmp');temporary.write_bytes(serialized);temporary.replace(destination)
    destination.with_suffix(destination.suffix+'.json').write_text(json.dumps(metadata,indent=2))
    print('Built '+str(destination),flush=True)
    return metadata


def decode_yolov8(raw):
    """COCO YOLOv8: pixel xywh + 80 class probabilities, no objectness channel."""
    if raw.shape!=(1,84,2100) or not np.all(np.isfinite(raw)):
        raise ValueError('Invalid YOLOv8 raw output')
    predictions=raw[0].T
    if np.any(predictions[:,4:]<0) or np.any(predictions[:,4:]>1):
        raise ValueError('YOLOv8 class probabilities outside 0..1')
    rows=np.empty((2100,85),np.float32)
    rows[:,:4]=predictions[:,:4]/SIZE
    rows[:,4]=1.
    rows[:,5:]=predictions[:,4:]
    return rows


def build_engine(directory, destination, precision='fp16', workspace_mb=128):
    """Build locally on the target Nano; never download a precompiled engine."""
    import tensorrt as trt
    destination = Path(destination)
    directory = Path(directory)
    if not trt.__version__.startswith('8.2.'):
        raise RuntimeError('This builder targets TensorRT 8.2 on JetPack 4')
    blocks = read_cfg(directory / 'yolov3-tiny.cfg')
    if int(blocks[0].get('channels', 3)) != 3:
        raise ValueError('Only three-channel YOLOv3-tiny is supported')
    weights_path = directory / 'yolov3-tiny.weights'
    with weights_path.open('rb') as file:
        version = np.fromfile(file, dtype='<i4', count=3)
        if len(version) != 3:
            raise ValueError('Truncated Darknet header')
        major, minor, _ = version
        seen_bytes = 8 if major * 10 + minor >= 2 and major < 1000 and minor < 1000 else 4
        if len(file.read(seen_bytes)) != seen_bytes:
            raise ValueError('Truncated Darknet seen counter')
        weights = np.fromfile(file, dtype='<f4')
    offset = [0]
    retained = []

    def take(count):
        start = offset[0]
        offset[0] += count
        if offset[0] > weights.size:
            raise ValueError('Truncated Darknet weights')
        value = np.ascontiguousarray(weights[start:offset[0]])
        retained.append(value)
        return value

    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    config.max_workspace_size = int(workspace_mb) * 1024 * 1024
    if precision == 'fp16':
        if not builder.platform_has_fast_fp16:
            raise RuntimeError('Target GPU does not report fast FP16 support')
        config.set_flag(trt.BuilderFlag.FP16)
    elif precision != 'fp32':
        raise ValueError('Unsupported precision')
    tensor = network.add_input('images', trt.float32, (1, 3, SIZE, SIZE))
    outputs = []
    heads = []
    for index, block in enumerate(blocks[1:]):
        kind = block['type']
        if kind == 'convolutional':
            if int(block.get('groups', 1)) != 1:
                raise ValueError('Grouped convolutions are not supported')
            channels = int(tensor.shape[1])
            filters, size, stride = [int(block[k]) for k in ('filters', 'size', 'stride')]
            bias = take(filters)
            if int(block.get('batch_normalize', 0)):
                gamma, mean, variance = take(filters), take(filters), take(filters)
                # Match OpenCV's Darknet importer epsilon, not a training default.
                scale = gamma / np.sqrt(variance + 1e-6)
            else:
                scale = None
            kernel = take(filters * channels * size * size).reshape(filters, channels, size, size)
            if scale is not None:
                kernel = np.ascontiguousarray(kernel * scale[:, None, None, None])
                bias = np.ascontiguousarray(bias - mean * scale)
                retained.extend((kernel, bias))
            layer = network.add_convolution_nd(tensor, filters, (size, size), kernel, bias)
            layer.stride_nd = (stride, stride)
            padding = size // 2 if int(block.get('pad', 0)) else int(block.get('padding', 0))
            layer.padding_nd = (padding, padding)
            tensor = layer.get_output(0)
            activation = block.get('activation', 'linear')
            if activation == 'leaky':
                layer = network.add_activation(tensor, trt.ActivationType.LEAKY_RELU)
                layer.alpha = .1
                tensor = layer.get_output(0)
            elif activation != 'linear':
                raise ValueError('Unsupported activation: ' + activation)
        elif kind == 'maxpool':
            size, stride = int(block['size']), int(block['stride'])
            layer = network.add_pooling_nd(tensor, trt.PoolingType.MAX, (size, size))
            layer.stride_nd = (stride, stride)
            layer.padding_mode = trt.PaddingMode.SAME_UPPER
            tensor = layer.get_output(0)
        elif kind == 'route':
            ids = [int(v) for v in block['layers'].split(',')]
            sources = [outputs[index + v if v < 0 else v] for v in ids]
            if len(sources) == 1:
                tensor = sources[0]
            else:
                layer = network.add_concatenation(sources)
                layer.axis = 1
                tensor = layer.get_output(0)
        elif kind == 'upsample':
            layer = network.add_resize(tensor)
            layer.resize_mode = trt.ResizeMode.NEAREST
            stride = int(block['stride'])
            layer.scales = (1., 1., float(stride), float(stride))
            tensor = layer.get_output(0)
        elif kind == 'yolo':
            if int(block['classes']) != 80 or len(heads) >= 2:
                raise ValueError('Expected two 80-class YOLOv3-tiny heads')
            mask = [int(v) for v in block['mask'].split(',')]
            anchors = np.array([float(v) for v in block['anchors'].split(',')]).reshape(-1, 2)
            name = 'yolo_head_' + str(len(heads))
            tensor.name = name
            network.mark_output(tensor)
            heads.append(dict(name=name, anchors=anchors[mask].tolist(), classes=80,
                              threshold=float(block.get('thresh', .2)),
                              nms=float(block.get('nms_threshold', .4))))
        else:
            raise ValueError('Unsupported Darknet layer: ' + kind)
        outputs.append(tensor)
    if len(heads) != 2 or offset[0] != weights.size:
        raise ValueError('Unexpected graph or unconsumed Darknet weights')
    print('Building TensorRT {} engine at 320x320 on this GPU...'.format(precision), flush=True)
    engine = builder.build_engine(network, config)
    if engine is None:
        raise RuntimeError('TensorRT engine construction failed')
    serialized = bytes(engine.serialize())
    metadata = fingerprint(directory, trt.__version__)
    metadata.update(precision=precision, heads=heads,
                    engine_sha256=hashlib.sha256(serialized).hexdigest())
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    temporary.write_bytes(serialized)
    temporary.replace(destination)
    destination.with_suffix(destination.suffix + '.json').write_text(json.dumps(metadata, indent=2))
    print('Built ' + str(destination), flush=True)
    return metadata


def decode_head(raw, head):
    """Return Region rows, including its per-head, per-class floating-box NMS."""
    anchors = np.asarray(head['anchors'], dtype=np.float32)
    _, channels, height, width = raw.shape
    count, classes = len(anchors), head['classes']
    if channels != count * (classes + 5):
        raise ValueError('Unexpected YOLO head shape')
    cells = raw.reshape(count, classes + 5, height, width).transpose(2, 3, 0, 1)
    rows = np.empty_like(cells)
    sigmoid = lambda x: 1. / (1. + np.exp(-np.clip(x, -80, 80)))
    grid_y, grid_x = np.mgrid[:height, :width]
    rows[..., 0] = (sigmoid(cells[..., 0]) + grid_x[..., None]) / width
    rows[..., 1] = (sigmoid(cells[..., 1]) + grid_y[..., None]) / height
    rows[..., 2:4] = np.exp(np.clip(cells[..., 2:4], -80, 80)) * anchors / SIZE
    rows[..., 4] = sigmoid(cells[..., 4])
    scores = sigmoid(cells[..., 5:]) * rows[..., 4:5]
    rows[..., 5:] = np.where(scores > head['threshold'], scores, 0)
    rows = rows.reshape(-1, classes + 5)
    if head['nms'] > 0:
        xy = rows[:, :2] - rows[:, 2:4] / 2
        end = xy + rows[:, 2:4]
        area = rows[:, 2] * rows[:, 3]
        for cls in range(classes):
            score = rows[:, 5 + cls]
            ids = np.flatnonzero(score > head['threshold'])
            ids = ids[np.argsort(-score[ids], kind='mergesort')]
            while ids.size:
                current, rest = ids[0], ids[1:]
                overlap = np.maximum(0, np.minimum(end[current], end[rest]) - np.maximum(xy[current], xy[rest]))
                intersection = overlap[:, 0] * overlap[:, 1]
                iou = intersection / np.maximum(area[current] + area[rest] - intersection, 1e-12)
                score[rest[iou > head['nms']]] = 0
                ids = rest[iou <= head['nms']]
    return rows


class Cuda:
    """Minimal checked CUDA runtime binding; avoids PyCUDA ABI dependencies."""
    def __init__(self):
        self.lib = ct.CDLL(ctypes.util.find_library('cudart') or '/usr/local/cuda/lib64/libcudart.so')
        signatures = dict(cudaSetDevice=[ct.c_int], cudaMalloc=[ct.POINTER(ct.c_void_p), ct.c_size_t],
                          cudaFree=[ct.c_void_p], cudaMemcpy=[ct.c_void_p, ct.c_void_p, ct.c_size_t, ct.c_int],
                          cudaStreamCreate=[ct.POINTER(ct.c_void_p)], cudaStreamDestroy=[ct.c_void_p],
                          cudaStreamSynchronize=[ct.c_void_p])
        for name, arguments in signatures.items():
            function = getattr(self.lib, name)
            function.argtypes, function.restype = arguments, ct.c_int
        self.lib.cudaGetErrorString.argtypes = [ct.c_int]
        self.lib.cudaGetErrorString.restype = ct.c_char_p
        self.check(self.lib.cudaSetDevice(0))

    def check(self, code):
        if code:
            raise RuntimeError('CUDA: ' + self.lib.cudaGetErrorString(code).decode())


class TensorRTDetector:
    backend = 'tensorrt'

    def __init__(self, directory, engine_path, model_id='yolov3-tiny'):
        import tensorrt as trt
        from vision_dashboard_common import postprocess
        self.postprocess = postprocess
        self.allocations, self.stream = [], None
        self.engine = self.context = self.runtime = None
        self.cuda = None
        self.model_id=model_id
        self.names = (Path(directory) / 'coco.names').read_text().splitlines()
        engine_path = Path(engine_path)
        metadata_path = engine_path.with_suffix(engine_path.suffix + '.json')
        if not engine_path.exists() or not metadata_path.exists():
            raise RuntimeError('TensorRT engine missing; run scripts/prepare_vision_tensorrt.py first, '
                               'or explicitly select --backend opencv')
        self.metadata = json.loads(metadata_path.read_text())
        expected = fingerprint(directory, trt.__version__, model_id)
        if any(self.metadata.get(key) != value for key, value in expected.items()):
            raise RuntimeError('TensorRT model/runtime fingerprint changed; rebuild the engine on this Nano')
        if sha256(engine_path) != self.metadata['engine_sha256']:
            raise RuntimeError('TensorRT engine SHA256 mismatch; rebuild it')
        self.model = ('YOLOv8n COCO' if model_id=='yolov8n' else 'YOLOv3-tiny')+' / Nano TensorRT GPU ' + self.metadata['precision'].upper()
        cv2.setNumThreads(2)
        try:
            self.cuda = Cuda()
            self.logger = trt.Logger(trt.Logger.WARNING)
            self.runtime = trt.Runtime(self.logger)
            self.engine = self.runtime.deserialize_cuda_engine(engine_path.read_bytes())
            if self.engine is None:
                raise RuntimeError('Cannot deserialize TensorRT engine')
            self.context = self.engine.create_execution_context()
            if self.context is None:
                raise RuntimeError('Cannot create TensorRT execution context')
            self.stream = ct.c_void_p()
            self.cuda.check(self.cuda.lib.cudaStreamCreate(ct.byref(self.stream)))
            self.bindings, self.host, self.indices = [], {}, {}
            for index in range(self.engine.num_bindings):
                name = self.engine.get_binding_name(index)
                shape = tuple(self.engine.get_binding_shape(index))
                if any(dim <= 0 for dim in shape):
                    raise ValueError('Only static TensorRT bindings are supported')
                self.host[name] = np.empty(shape, dtype=trt.nptype(self.engine.get_binding_dtype(index)))
                pointer = ct.c_void_p()
                self.cuda.check(self.cuda.lib.cudaMalloc(ct.byref(pointer), self.host[name].nbytes))
                self.allocations.append(pointer)
                self.bindings.append(pointer.value)
                self.indices[name] = index
            self.input_name=self.metadata.get('input_name','images')
            expected_names = ({self.input_name,self.metadata['output_name']} if model_id=='yolov8n' else {'images'} | {head['name'] for head in self.metadata['heads']})
            if set(self.host) != expected_names or self.host[self.input_name].shape != (1, 3, SIZE, SIZE):
                raise ValueError('Unexpected TensorRT engine bindings')
        except Exception:
            self.close()
            raise

    def rows(self, image):
        if image.shape != (SIZE, SIZE, 3) or image.dtype != np.uint8:
            raise ValueError('TensorRT expects a 320x320 uint8 BGR input')
        blob = cv2.dnn.blobFromImage(image, 1 / 255., (SIZE, SIZE), swapRB=True, crop=False)
        np.copyto(self.host[self.input_name], blob)
        source = self.host[self.input_name]
        self.cuda.check(self.cuda.lib.cudaMemcpy(self.bindings[self.indices[self.input_name]], source.ctypes.data, source.nbytes, 1))
        if not self.context.execute_async_v2(self.bindings, self.stream.value):
            raise RuntimeError('TensorRT GPU execution failed')
        self.cuda.check(self.cuda.lib.cudaStreamSynchronize(self.stream))
        if self.model_id=='yolov8n':
            name=self.metadata['output_name'];array=self.host[name]
            self.cuda.check(self.cuda.lib.cudaMemcpy(array.ctypes.data,self.bindings[self.indices[name]],array.nbytes,2))
            return [decode_yolov8(array)]
        result = []
        for head in self.metadata['heads']:
            array = self.host[head['name']]
            self.cuda.check(self.cuda.lib.cudaMemcpy(array.ctypes.data, self.bindings[self.indices[head['name']]], array.nbytes, 2))
            if not np.all(np.isfinite(array)):
                raise RuntimeError('Non-finite TensorRT output')
            result.append(decode_head(array, head))
        return result

    def detect(self, image):
        return self.postprocess(self.rows(image), self.names)

    def close(self):
        if self.cuda is not None:
            if self.stream:
                self.cuda.lib.cudaStreamSynchronize(self.stream)
            # The context retains activation memory; release it before the engine/runtime.
            self.context = self.engine = self.runtime = None
            for pointer in self.allocations:
                self.cuda.lib.cudaFree(pointer)
            self.allocations = []
            if self.stream:
                self.cuda.lib.cudaStreamDestroy(self.stream)
                self.stream = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
