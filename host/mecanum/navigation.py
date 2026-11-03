"""Explicit navigation upload/start sessions. No socket ownership or IO on import."""
from shared.navigation_protocol import packet, upload_operations


class NavigationSession:
    def __init__(self, send_motor, write, spec, now, motor, navigation, options):
        if not motor or now-motor[0] > .3 or not navigation or now-navigation[0] > .5:
            raise ValueError('需要新版固件及新鲜的电机/N4导航遥测')
        self.send_motor, self.write = send_motor, write
        self.spec, self.options = spec, options
        self.sequence = navigation[1]['ack_sequence']
        self.phase, self.sent, self.deadline = 'stop', now, now+1
        self.still_since = None
        self.action = spec['action']
        self.operations = upload_operations(spec['calibration'], spec['points'], spec['mission_id']) if self.action == 'upload' else []
        self.index, self.retries, self.expected = 0, 0, None
        self.last_heartbeat = now
        self.send_motor(0, (0, 0, 0), options)

    def send(self, now, operation, **values):
        self.sequence = (self.sequence+1) & 255
        self.pending_packet = packet(operation, self.sequence, **values)
        self.write(self.pending_packet)
        self.expected, self.sent, self.deadline, self.retries = self.sequence, now, now+.8, 0

    def step(self, now, motor, pose, navigation=None):
        if not motor or now-motor[0] > .3 or not navigation or now-navigation[0] > .5:
            raise RuntimeError('导航遥测过期，已请求停止')
        m, n = motor[1], navigation[1]
        if self.phase == 'stop' and (m['armed'] or any(v != 0 for v in m['velocity'])):
            self.still_since = None
        if self.phase == 'stop' and motor[0] > self.sent and not m['armed'] and all(v == 0 for v in m['velocity']):
            if self.still_since is None:
                self.still_since = now
                self.deadline = now+1
            if now-self.still_since < .3:
                return None
            if self.action == 'upload':
                self.phase = 'upload'
                self.send(now, **self.operations[0])
            elif self.action in ('cancel', 'mode'):
                self.phase = 'cancel'
                self.send(now, 'cancel', mission_id=n['mission_id'])
            else:
                if self.action not in ('start', 'resume'):
                    raise ValueError('无效导航动作')
                if not n['calibrated'] or not pose or now-pose[0] > .3 or not pose[1]['healthy'] or pose[1].get('imu_stale'):
                    raise ValueError('标定或IMU新鲜度不满足启动条件')
                self.phase, self.sent, self.deadline = 'clear', now, now+.5
                self.send_motor(3, (0, 0, 0), self.options)
        elif self.phase == 'clear' and motor[0] > self.sent and not m['faults'] and not m['armed']:
            self.phase, self.sent, self.deadline = 'arm', now, now+.5
            self.motor_sequence = self.send_motor(1, (0, 0, 0), self.options)
        elif self.phase == 'arm' and motor[0] > self.sent and m['armed'] and m['seq'] == self.motor_sequence:
            self.phase = 'starting'
            self.send(now, self.action, mission_id=self.spec['mission_id'])
        elif self.phase in ('upload', 'cancel', 'starting'):
            if navigation[0] > self.sent and n['ack_sequence'] == self.expected:
                if n['ack_error']:
                    raise RuntimeError('FPGA拒绝导航命令，错误码 %s' % n['ack_error'])
                if self.phase == 'upload':
                    if self.operations[self.index]['operation'] == 'config' and self.operations[self.index].get('index') == 15 and not n['calibrated']:
                        raise RuntimeError('FPGA几何确认失败')
                    self.index += 1
                    if self.index == len(self.operations):
                        return '路径与标定已提交，尚未启动'
                    self.send(now, **self.operations[self.index])
                elif self.phase == 'cancel':
                    return '任务已取消，电机保持停止，可切换验收模式'
                else:
                    self.phase = 'running'
                    self.last_heartbeat = now-.1
                    self.deadline = float('inf')
            elif now >= self.deadline and self.retries < 2:
                # Only replay the identical idempotent V4 transaction, never ARM.
                self.write(self.pending_packet)
                self.deadline = now+.8
                self.retries += 1
        elif self.phase == 'running':
            if n['status'] == 5:
                return '全部路径点完成，已停止'
            if n['status'] == 4:
                return '任务暂停，原因码 %s；需手动继续' % n['reason']
            if not n['active']:
                raise RuntimeError('任务已取消或坐标基准失效；不能自动继续')
            if m['faults'] or not m['armed']:
                raise RuntimeError('底盘停止或故障，任务需手动继续')
            if now-self.last_heartbeat >= .1:
                self.sequence = (self.sequence+1) & 255
                self.write(packet('heartbeat', self.sequence, mission_id=self.spec['mission_id']))
                self.last_heartbeat = now
        if now >= self.deadline:
            raise RuntimeError('导航握手超时；任务不会自动重启')
        return None
