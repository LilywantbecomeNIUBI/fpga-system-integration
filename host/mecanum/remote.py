"""Held-button intent. The command worker enforces its own deadman deadline."""
import threading


class RemoteLease:
    TIMEOUT = .30

    def __init__(self):
        self.lock = threading.Lock()
        self.closed = 0
        self.current = None

    def begin(self, gesture, direction, limit, now):
        with self.lock:
            if gesture <= self.closed or (self.current and gesture <= self.current['gesture']):
                raise ValueError('遥控按键已结束，请松开后重新按下')
            self.closed = gesture - 1
            self.current = dict(gesture=gesture, direction=tuple(direction), limit=limit,
                                deadline=now+self.TIMEOUT)

    def pulse(self, gesture, now):
        with self.lock:
            if not self.current or self.current['gesture'] != gesture or now >= self.current['deadline']:
                self.closed = max(self.closed, gesture)
                if self.current and self.current['gesture'] <= gesture:
                    self.current = None
                raise ValueError('遥控按住心跳已过期，请松开后重新按下')
            self.current['deadline'] = now+self.TIMEOUT

    def end(self, gesture):
        with self.lock:
            self.closed = max(self.closed, gesture)
            if self.current and self.current['gesture'] <= gesture:
                self.current = None

    def cancel(self):
        with self.lock:
            if self.current:
                self.closed = max(self.closed, self.current['gesture'])
            self.current = None

    def snapshot(self, now):
        with self.lock:
            if self.current and now < self.current['deadline']:
                return dict(self.current)
            return None


class RemoteSession:
    """Reuse STOP/CLEAR/ARM/goal handshakes, with an independent hold deadline.

    Only a currently held gesture may start/renew a goal. It never chains goals
    when the finite travel limit is reached, nor restarts a failed gesture.
    """
    def __init__(self, intent, lease, send, profile, now, motor, pose, factory, spec_factory):
        self.gesture = intent['gesture']
        self.lease = lease
        x, y, turn = intent['direction']
        # <=2 counts/10ms. Diagonal axes use 1 to bound resultant speed.
        limit = min(intent['limit'], 1 if x and y else 2)
        spec = spec_factory('定角' if turn else '位置', 3000*x, 3000*y, 720*turn,
                            limit, 4, 120, dict(motor=0, encoder=0,
                            kp=profile['initial_kp_q8'], ki=profile['initial_ki_q8']), profile)
        self.inner = factory(send, spec, now, motor, pose, profile['control_shift'])

    def step(self, now, motor, pose):
        intent = self.lease.snapshot(now)
        if not intent or intent['gesture'] != self.gesture:
            return '遥控已松开或按住心跳超时'
        if not pose or now-pose[0] >= .3 or not pose[1].get('healthy') or pose[1].get('imu_stale'):
            raise RuntimeError('遥控姿态反馈过期，已停车')
        return self.inner.step(now, motor, pose)
