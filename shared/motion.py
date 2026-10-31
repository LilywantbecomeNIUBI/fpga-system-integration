"""Chassis-independent motion contracts. No serial, GPIO or sensor dependencies.

Body axes: +v forward, +w left/counterclockwise. Local monotonic milliseconds;
never compare clocks from different computers. A gateway creates its own lease.
"""
import math


class MotionIntent:
    def __init__(self, v_mm_s, w_mrad_s, issued_ms, valid_for_ms, source):
        for name, value in (('v', v_mm_s), ('w', w_mrad_s), ('issued', issued_ms), ('duration', valid_for_ms)):
            if type(value) is not int:
                raise ValueError(name+' must be an integer')
        if not 1 <= valid_for_ms <= 200 or issued_ms < 0 or not isinstance(source, str) or not source:
            raise ValueError('Invalid motion lease/source')
        self.v_mm_s, self.w_mrad_s = v_mm_s, w_mrad_s
        self.issued_ms, self.valid_for_ms, self.source = issued_ms, valid_for_ms, source

    def fresh(self, now_ms):
        return self.issued_ms <= now_ms < self.issued_ms+self.valid_for_ms


def constrained(intent, constraint, now_ms):
    """Compute a proposal only. Callers still need ownership, ARM and heartbeat.

    Unknown/expired avoidance constrains a future automatic policy to zero.
    The current manual commissioning path does not call this function.
    """
    age = constraint.get('age_ms')
    usable = bool(constraint.get('fresh') and constraint.get('confident') and
                  isinstance(age, (int, float)) and math.isfinite(age) and 0 <= age < 200)
    scale = constraint.get('speed_limit', 0)
    if type(scale) is not int or not 0 <= scale <= 255:
        raise ValueError('speed_limit is normalized 0..255, never PWM')
    if not intent.fresh(now_ms) or not usable or constraint.get('stop_request', True):
        return 0, 0
    return round(intent.v_mm_s*scale/255), round(intent.w_mrad_s*scale/255)


class DifferentialDrive:
    """Replaceable kinematics adapter. Geometry/calibration must be explicit.

    encoder_sign is measured feedback sign for physical forward, independent
    of H-bridge reversal flags. Current chassis signs are (-1, +1).
    """
    def __init__(self, wheel_diameter_mm, track_width_mm, counts_per_rev,
                 encoder_sign, max_counts, control_ms=10):
        values = (wheel_diameter_mm, track_width_mm, counts_per_rev, max_counts, control_ms)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('Explicit positive chassis calibration required')
        if tuple(encoder_sign) not in ((-1, 1), (1, -1), (1, 1), (-1, -1)):
            raise ValueError('encoder_sign must contain two polarities')
        self.track_width_mm, self.sign, self.max_counts = track_width_mm, tuple(encoder_sign), max_counts
        self.factor = counts_per_rev*control_ms/(1000*math.pi*wheel_diameter_mm)

    def wheel_targets(self, intent, now_ms):
        if not intent.fresh(now_ms):
            return 0, 0
        turn = intent.w_mrad_s*self.track_width_mm/2000
        targets = [(intent.v_mm_s-turn)*self.factor, (intent.v_mm_s+turn)*self.factor]
        ratio = min(1., self.max_counts/max(max(map(abs, targets)), 1e-12))
        return tuple(round(v*ratio)*sign for v, sign in zip(targets, self.sign))
