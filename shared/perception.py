"""Mixed FPGA event consumer, independent of chassis and transport ownership."""
import time
from optical_flow_protocol import parse_packet
from optical_flow_state import FlowState
from vision_v23_protocol import parse_event


class PerceptionEvents:
    def __init__(self):
        self.flow = FlowState()
        self.tracking = None
        self.tracking_stamp = None

    def ingest(self, payload, now=None):
        now = time.monotonic() if now is None else now
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
        return result
