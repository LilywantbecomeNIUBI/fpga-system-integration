"""FPGA V2/V3 telemetry and exact quarter-scale letterbox mapping, Python 3.6+."""
import math
import struct
from vision_protocol import detection

EVENT_PORT=6202
EVENT=struct.Struct('!4sBBBB12H')

def source_bbox(box):
    """320x320 model xyxy -> original 1280x720 exclusive xyxy; padding-only -> None."""
    if len(box)!=4 or not all(math.isfinite(v) for v in box):
        raise ValueError('bbox must contain four finite coordinates')
    a,b,c,d=box
    if a>=c or b>=d:
        raise ValueError('bbox is empty or reversed')
    x1=max(0,min(1280,int(math.floor(a*4))))
    y1=max(0,min(720,int(math.floor((b-70)*4))))
    x2=max(0,min(1280,int(math.ceil(c*4))))
    y2=max(0,min(720,int(math.ceil((d-70)*4))))
    return None if x1>=x2 or y1>=y2 else (x1,y1,x2,y2)

def model_detection(frame_id,box=None,class_id=39,confidence=0.9,inference_ms=0):
    if not math.isfinite(confidence) or not 0<=confidence<=1:
        raise ValueError('model confidence must be in 0..1')
    original=None if box is None else source_bbox(box)
    return detection(frame_id,original,class_id,int(round(confidence*1000)),inference_ms=inference_ms)

def parse_event(payload):
    if len(payload)!=32:
        raise ValueError('FSTA must contain 32 bytes')
    values=EVENT.unpack(payload)
    magic,version,flags,reason,direction=values[:5]
    if magic!=b'FSTA' or version!=2 or direction>3 or values[-1]!=0:
        raise ValueError('invalid FSTA header/reserved field')
    names=('frame_id','semantic_frame_id','class_id','confidence','x1','y1','x2','y2',
           'red_permille','track_confidence','semantic_age','reserved')
    result=dict(zip(names,values[5:]));result.pop('reserved')
    if result['red_permille']>1000 or result['track_confidence']>1000:
        raise ValueError('invalid normalized quality')
    for bit,name in enumerate(('target_valid','track_valid','semantic_valid','red','force_yolo','history_miss','scene_change','bad_frame')):
        result[name]=bool(flags&(1<<bit))
    result.update(reason=reason,direction=('none','left','center','right')[direction])
    return result
