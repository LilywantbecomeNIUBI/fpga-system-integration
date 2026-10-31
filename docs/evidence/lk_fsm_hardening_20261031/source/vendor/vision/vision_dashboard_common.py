"""Shared live capture and selectable YOLO adapters, compatible with Nano Python 3.6."""
import socket
import threading
import time
from pathlib import Path
import cv2
import numpy as np
from camera_protocol import AutoAssembler
from vision_v23_protocol import source_bbox

PROFILE='fpga_yolo_dashboard_v1'


def select_feedback(detections):
    eligible=[d for d in detections if d['class_id']==39 and d['confidence']>=.5 and source_bbox(d['bbox']) is not None]
    return max(eligible,key=lambda d:d['confidence']) if eligible else None


def jpeg(image):
    ok,data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90])
    if not ok:raise RuntimeError('JPEG preview encoding failed')
    return data.tobytes()


class Capture:
    def __init__(self,bind,port=6102,source='192.168.50.30',encode_preview=True):
        self.assembler=AutoAssembler(raw=True);self.lock=threading.Lock();self.stop=threading.Event()
        self.latest=None;self.error=None;self.first=None;self.last=None;self.encode_preview=encode_preview
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
        self.sock.bind((bind,port));self.sock.settimeout(.1);self.source=source
        self.thread=threading.Thread(target=self.receive,daemon=True);self.thread.start()

    def receive(self):
        try:
            while not self.stop.is_set():
                try:packet,peer=self.sock.recvfrom(65535)
                except socket.timeout:continue
                if peer[0]!=self.source:continue
                raw=self.assembler.push(packet)
                if raw is None:continue
                if not isinstance(raw,np.ndarray) or raw.shape!=(320,320):
                    raise RuntimeError('需要 V2/V3 的 320x320 RGB565 位流，请运行对应下载入口')
                image=cv2.cvtColor(raw.view(np.uint8).reshape(320,320,2),cv2.COLOR_BGR5652BGR)
                stamp=time.monotonic();encoded=jpeg(image) if self.encode_preview else None
                if self.first is None:self.first=stamp
                self.last=stamp
                with self.lock:self.latest=(self.assembler.last_completed,image,stamp,encoded)
        except Exception as exc:
            if not self.stop.is_set():self.error=str(exc)

    def frame(self):
        with self.lock:return self.latest

    def status(self):
        now=time.monotonic();a=self.assembler
        return dict(frames=a.frames,packets=a.packets,bad=a.bad,missing_frames=a.missing_frames,
                    fps=(a.frames-1)/(self.last-self.first) if a.frames>1 and self.last>self.first else 0,
                    frame_id=a.last_completed,age_ms=(now-self.last)*1000 if self.last else None,error=self.error)

    def close(self):
        self.stop.set();self.thread.join(2);self.sock.close()


class Detector:
    backend = 'opencv'
    model_id = 'yolov3-tiny'
    model = 'YOLOv3-tiny / Nano OpenCV CPU'

    def __init__(self,directory):
        p=Path(directory);cv2.setNumThreads(2)
        self.names=(p/'coco.names').read_text().splitlines()
        self.net=cv2.dnn.readNetFromDarknet(str(p/'yolov3-tiny.cfg'),str(p/'yolov3-tiny.weights'))
        layers=self.net.getLayerNames()
        self.outputs=[layers[int(i)-1] for i in self.net.getUnconnectedOutLayers().reshape(-1)]

    def detect(self,image):
        self.net.setInput(cv2.dnn.blobFromImage(image,1/255.,(320,320),swapRB=True,crop=False))
        return postprocess(self.net.forward(self.outputs), self.names)

    def close(self):
        pass


def create_detector(directory, backend='tensorrt', engine_path=None, model_id='yolov3-tiny'):
    from vision_models import resolve_model
    model_id=resolve_model(model_id,backend)
    if backend == 'opencv':
        return Detector(directory)
    if backend != 'tensorrt':
        raise ValueError('Unknown YOLO backend: ' + backend)
    from vision_tensorrt import TensorRTDetector
    return TensorRTDetector(directory, engine_path,model_id)


def postprocess(outputs, names):
    boxes,scores,classes=[],[],[]
    for output in outputs:
        # Do not hold the GIL in thousands of tiny argmax calls while the
        # Nano's receiver must consume 9,600 UDP packets per second.
        output=np.asarray(output)
        if not output.size:continue
        labels=np.argmax(output[:,5:],axis=1)
        confidence=output[np.arange(len(output)),5+labels]
        candidates=np.flatnonzero(np.isfinite(confidence)&(confidence>=.05))
        for index in candidates:
            row=output[index];cls=int(labels[index]);score=float(confidence[index])
            # OpenCV Region has already included objectness in row[5:].
            if not np.isfinite(score) or score<.05:continue
            cx,cy,w,h=[float(v)*320 for v in row[:4]]
            x1,y1=max(0,int(cx-w/2)),max(0,int(cy-h/2))
            x2,y2=min(320,int(cx+w/2)),min(320,int(cy+h/2))
            if x1>=x2 or y1>=y2:continue
            boxes.append([x1,y1,x2-x1,y2-y1]);scores.append(score);classes.append(cls)
    result=[]
    for cls in set(classes):
        ids=[i for i,c in enumerate(classes) if c==cls]
        keep=cv2.dnn.NMSBoxes([boxes[i] for i in ids],[scores[i] for i in ids],.05,.45)
        for index in np.asarray(keep).reshape(-1):
            i=ids[int(index)];x,y,w,h=boxes[i]
            result.append(dict(class_id=cls,label=names[cls],confidence=scores[i],bbox=[x,y,x+w,y+h]))
    return sorted(result,key=lambda d:d['confidence'],reverse=True)[:50]
