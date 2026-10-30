"""Mixed FPGA event consumer, independent of chassis and transport ownership."""
import time
import struct
from optical_flow_protocol import parse_packet
from optical_flow_state import FlowState
from vision_v23_protocol import parse_event


class PerceptionEvents:
    def __init__(self):
        self.flow = FlowState()
        self.tracking = None
        self.tracking_stamp = None
        self.diagnostic = None
        self.diagnostic_stamp = None

    def ingest(self, payload, now=None):
        now = time.monotonic() if now is None else now
        if payload[:4] == b'OFDG':
            if len(payload)!=32 or payload[4]!=1 or payload[5]>2:
                raise ValueError('Invalid OFDG diagnostic')
            v=struct.unpack('!4s4B3H4B2H2B2I',payload)
            names=('frame_id','point_x','point_y','patch_state','divider_count','patch_flags',
                   'iteration','patch_wait_cycles','grid_wait_cycles','patch_starts','patch_completions','cycles','dropped_frames')
            event=dict(zip(names,v[5:]))
            event.update(stage=v[2],grid_busy=bool(v[3]&1),grid_state=v[4])
            flags=event['patch_flags']
            fault=event['iteration']
            event.update(illegal_state_seen=bool(fault&128),first_illegal_state=(fault>>2)&31,
                         first_fault_state=(fault>>2)&31,fault_idle_busy=bool(v[3]&64),
                         previous_fault_state=(v[3]>>1)&31,iteration=fault&3)
            event.update(divider_busy=bool(flags&1),divider_done=bool(flags&2),divider_start=bool(flags&4),
                         patch_busy=bool(flags&8),patch_done=bool(flags&16))
            self.diagnostic,self.diagnostic_stamp=event,now
            return 'diagnostic',event
        if payload[:4] in (b'OFST', b'OFLW'):
            event = parse_packet(payload)
            self.flow.ingest(event, now)
            return 'flow', event
        event = parse_event(payload)
        self.tracking, self.tracking_stamp = event, now
        return 'tracking', event

    def flow_snapshot(self, now=None, mode='live'):
        result = self.flow.snapshot(now, mode)
        result.update(observe_only=True, automatic_control_enabled=False)
        stamp=time.monotonic() if now is None else now
        result['diagnostic']=dict(self.diagnostic,age_ms=max(0,stamp-self.diagnostic_stamp)*1000) if self.diagnostic else None
        result['health_fault']=bool(self.diagnostic and self.diagnostic.get('illegal_state_seen'))
        if result['health_fault']:
            result['control'].update(fresh=False,confident=False,state='unknown',stop_request=True,steer=0,speed_limit=0)
        return result
