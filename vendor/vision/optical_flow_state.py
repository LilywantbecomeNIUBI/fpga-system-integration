"""Frame/epoch ownership and replay validation for the optical-flow viewer (stdlib)."""
from collections import OrderedDict, deque
import base64
import json
import math
import time
import uuid
from optical_flow_protocol import PACKET, STATES, parse_packet

PROFILE = 'fpga_optical_flow_dashboard_v1'
POINTS = 190
MAX_RECORD_BYTES = 64*1024*1024
MAX_RECORD_EVENTS = 250000


def newer(a, b):
    return 0 < ((a-b) & 65535) < 32768


def canonical_event(event):
    """Validate decoded monitor/replay records through the same binary protocol."""
    if not isinstance(event, dict):
        raise ValueError('Telemetry must be an object')
    def integer(name, lo, hi):
        v = event[name]
        if isinstance(v, bool) or not isinstance(v, int) or not lo <= v <= hi:
            raise ValueError('Invalid '+name)
        return v
    stage = integer('stage', 0, 2)
    fid = integer('frame_id', 0, 65535)
    cycles = integer('cycles', 0, 4294967295)
    drops = integer('dropped_frames', 0, 4294967295)
    if event.get('kind') == 'flow':
        u, v = event['u'], event['v']
        if any(isinstance(n, bool) or not isinstance(n, (int, float)) or not math.isfinite(n)
               or not -256 <= n < 256 or abs(n*128-round(n*128)) > 1e-6 for n in (u, v)):
            raise ValueError('Invalid Q7 flow')
        if not isinstance(event.get('valid'), bool):
            raise ValueError('Invalid vector validity')
        fields = [fid, integer('x', 0, 159), integer('y', 0, 89),
                  round(u*128) & 65535, round(v*128) & 65535,
                  integer('sequence', 0, 189), integer('width', 160, 160), integer('height', 90, 90)]
        raw = PACKET.pack(b'OFLW', 1, stage, int(event['valid']), 0, *fields, cycles, drops)
    elif event.get('kind') == 'status':
        for key in ('stop_request', 'confident', 'stale'):
            if not isinstance(event.get(key), bool):
                raise ValueError('Invalid '+key)
        counts, risk = event['valid_counts'], event['risk_counts']
        if not isinstance(counts, list) or not isinstance(risk, list) or len(counts) != 3 or len(risk) != 3 or any(type(n) is not int or not 0 <= n <= 65535 for n in counts+risk):
            raise ValueError('Invalid regional counts')
        flags = int(event['stop_request']) | (int(event['confident']) << 1) | (integer('hazard', 0, 7) << 2) | (int(event['stale']) << 5)
        control = (integer('speed_limit', 0, 255) << 8) | (integer('steer', -1, 1) & 255)
        raw = PACKET.pack(b'OFST', 1, stage, flags, STATES.index(event['state']),
                          fid, *counts, *risk, control, cycles, drops)
    else:
        raise ValueError('Unknown telemetry kind')
    return parse_packet(raw)


class FlowState:
    def __init__(self):
        self.reset()

    def reset(self):
        self.epoch = uuid.uuid4().hex[:12]
        self.frames = OrderedDict()
        self.images = OrderedDict()
        self.latest = self.status_key = None
        self.stage = None
        self.last_accepted = None
        self.status_stamp = None
        self.control = None
        self.frames_seen = self.packets = self.duplicates = self.late = self.bad = 0
        self.ignored_source = self.restarts = self.final_missing = 0
        self.rates = deque(maxlen=120)
        self.video_rates = deque(maxlen=120)
        self.video_frames = self.video_bad = self.video_missing = 0
        self.last_video_stamp = None
        self.latest_video = None
        self.risk_history = deque(maxlen=100)

    def _new_epoch(self):
        restarts = self.restarts+1
        self.reset()
        self.restarts = restarts

    def ingest(self, event, now=None):
        now = time.monotonic() if now is None else now
        fid = event['frame_id']
        if event['kind'] == 'flow':
            seq = event['sequence']
            if (event['x'], event['y']) != (8+(seq % 19)*8, 8+(seq // 19)*8):
                self.bad += 1
                return False
        if self.stage is not None and self.stage != event['stage']:
            # A delayed packet from the previous bitstream cannot replace an active session.
            if self.last_accepted is not None and now-self.last_accepted <= .5:
                self.late += 1
                return False
            self._new_epoch()
        if (self.latest is not None and fid != self.latest and not newer(fid, self.latest)
                and fid <= 32 and self.last_accepted is not None and now-self.last_accepted > .5):
            self._new_epoch()
        self.stage = event['stage']
        self.packets += 1
        advancing = self.latest is None or newer(fid, self.latest)
        if advancing:
            self.latest = fid
            self.rates.append(now)
            self.frames_seen += 1
        elif fid not in self.frames:
            self.late += 1
            return False
        if fid not in self.frames:
            self.frames[fid] = dict(frame_id=fid, vectors={}, status=None, first=now)
        frame = self.frames[fid]
        if event['kind'] == 'flow':
            seq = event['sequence']
            if seq in frame['vectors']:
                self.duplicates += 1
                return False
            frame['vectors'][seq] = event
        else:
            old = frame['status']
            if old == event:
                self.duplicates += 1
                return False
            frame['status'] = event
            fresh_id = self.status_key is None or newer(fid, self.status_key)
            # Repeated/reordered FORWARD cannot refresh the local deadline. Same-id STOP is immediate.
            if fresh_id or (fid == self.status_key and event['stop_request']):
                self.control = event
                self.status_key = fid
                if fresh_id:
                    self.status_stamp = now
                    self.risk_history.append(dict(frame_id=fid, ratios=[r/n if n else None for n, r in zip(event['valid_counts'], event['risk_counts'])]))
        self.last_accepted = now
        while len(self.frames) > 16:
            _, expired = self.frames.popitem(last=False)
            self.final_missing += POINTS-len(expired['vectors'])
        return True

    def video(self, fid, jpeg, now=None):
        now = time.monotonic() if now is None else now
        self.images[fid] = bytes(jpeg)
        while len(self.images) > 20:
            self.images.popitem(last=False)
        self.video_frames += 1
        self.latest_video = fid
        self.last_video_stamp = now
        self.video_rates.append(now)

    @staticmethod
    def fps(stamps, now):
        active = [s for s in stamps if 0 <= now-s < 2]
        return (len(active)-1)/(active[-1]-active[0]) if len(active) > 1 and active[-1] > active[0] else 0.

    def snapshot(self, now=None, mode='idle'):
        now = time.monotonic() if now is None else now
        key = self.status_key if self.status_key in self.frames else self.latest
        # Telemetry often finishes before the image UDP segments. Keep the newest
        # exact pair on screen while the next image is still arriving; control
        # freshness continues to use the newest OFST independently of this view.
        for candidate in reversed(self.frames):
            if candidate in self.images and (self.status_key is None or not newer(candidate,self.status_key)):
                key = candidate
                break
        field = self.frames.get(key) or dict(frame_id=None, vectors={}, status=None, first=now)
        vectors = sorted(field['vectors'].values(), key=lambda e: e['sequence'])
        control = self.control
        age = max(0, now-self.status_stamp)*1000 if self.status_stamp is not None else None
        fresh = age is not None and age < 200 and control is not None
        usable = fresh and control['confident'] and not control['stale'] and mode not in ('idle', 'ended')
        displayed = dict(state=control['state'] if usable else 'unknown',
                         stop_request=control['stop_request'] if usable else True,
                         steer=control['steer'] if usable else 0, speed_limit=control['speed_limit'] if usable else 0,
                         hazard=control['hazard'] if usable else 0, fresh=bool(fresh and mode not in ('idle', 'ended')),
                         confident=bool(usable), age_ms=age, frame_id=control['frame_id'] if control else None,
                         valid_counts=control['valid_counts'] if control else [0, 0, 0],
                         risk_counts=control['risk_counts'] if control else [0, 0, 0])
        known_valid = sum(e['valid'] for e in vectors)
        key_string = '%s-%s' % (self.epoch, key) if key in self.images else None
        return dict(epoch=self.epoch, stage=self.stage, frame_id=key, newest_frame_id=self.latest, field_status=field['status'],
                    vectors=vectors, received=len(vectors), valid=known_valid, invalid=len(vectors)-known_valid,
                    missing=POINTS-len(vectors), expected=POINTS, image_key=key_string, control=displayed,
                    cycles=(field['status'] or {}).get('cycles', max([v['cycles'] for v in vectors] or [0])),
                    dropped_frames=(control or {}).get('dropped_frames', 0), fps=self.fps(self.rates, now),
                    packets=self.packets, bad=self.bad, duplicates=self.duplicates, late=self.late,
                    ignored_source=self.ignored_source, restarts=self.restarts,
                    video=dict(frames=self.video_frames, fps=self.fps(self.video_rates, now),
                               frame_id=self.latest_video, age_ms=(now-self.last_video_stamp)*1000 if self.last_video_stamp is not None else None,
                               bad=self.video_bad, missing_frames=self.video_missing),
                    history=list(self.risk_history))


def parse_recording(raw, with_metadata=False):
    """Accept this viewer's recording and the previous optical_flow_monitor JSONL."""
    if len(raw) > MAX_RECORD_BYTES:
        raise ValueError('记录文件超过64MB')
    items, first = [], None
    metadata = dict(source_mode='unknown')
    for row in raw.decode('utf-8-sig').splitlines():
        if not row.strip():
            continue
        item = json.loads(row)
        if not isinstance(item, dict):
            raise ValueError('Recording rows must be objects')
        if item.get('type') == 'header':
            if item.get('schema') != 'optical_flow_recording_v1':
                raise ValueError('不支持的记录版本')
            metadata['source_mode'] = item.get('source_mode') if item.get('source_mode') in ('live','demo') else 'unknown'
            continue
        if item.get('type') in ('telemetry', 'video'):
            stamp = item['at_ms']
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or not 0 <= stamp <= 3600000:
                raise ValueError('无效回放时间')
            if item['type'] == 'telemetry':
                items.append(dict(type='telemetry', at_ms=float(stamp), data=canonical_event(item['data'])))
            else:
                fid = item['frame_id']
                image = base64.b64decode(item['jpeg_b64'], validate=True)
                if type(fid) is not int or not 0 <= fid <= 65535 or not 4 < len(image) <= 2*1024*1024 or image[:2] != b'\xff\xd8' or image[-2:] != b'\xff\xd9':
                    raise ValueError('无效图像记录')
                items.append(dict(type='video', at_ms=float(stamp), frame_id=fid, jpeg=image))
        elif item.get('kind') in ('flow', 'status'):
            stamp = item['received_unix']
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                raise ValueError('无效记录时间')
            if first is None:
                first = stamp
            at = max(0., (stamp-first)*1000)
            if at > 3600000:
                raise ValueError('记录时间超过1小时')
            items.append(dict(type='telemetry', at_ms=at, data=canonical_event(item)))
        else:
            raise ValueError('记录中包含未知数据类型')
        if len(items) > MAX_RECORD_EVENTS:
            raise ValueError('记录事件数超过250000')
    if not any(x['type'] == 'telemetry' for x in items):
        raise ValueError('没有可回放的光流记录')
    items.sort(key=lambda x: x['at_ms'])
    return (items, metadata) if with_metadata else items
