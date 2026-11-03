"""Independent wheel/IMU teaching frame and versioned local route files.

The P4 position resets at each manual goal. Use raw M4 wheel counts instead.
Never claim nominal motor dimensions are a measured navigation calibration.
"""
import copy
import json
import math
from pathlib import Path
import re
import uuid

from shared.navigation_protocol import validate_calibration, upload_operations


def wrap(degrees):
    return (degrees+180) % 360-180


class Teaching:
    def __init__(self, profile, folder):
        self.profile = profile
        self.folder = Path(folder)
        self.calibration = None
        nominal = math.pi*profile['wheel_diameter_mm']/profile['raw_counts_per_rev']
        self.scales = [nominal, nominal]
        self.valid = False
        self.error = '请先停车并设置示教起点'
        self.points = []
        self.x = self.y = self.heading = 0.
        self.origin_yaw = None
        self.previous = None
        self.last_time = None
        self.still_since = None
        self.last_observed = None
        self.replay_ready = False
        self.name = ''
        self.revision = 0

    def invalidate(self, reason):
        self.valid = False
        self.replay_ready = False
        self.still_since = None
        self.error = reason

    def configure(self, config):
        cfg = validate_calibration(config)
        json.dumps(cfg, allow_nan=False)
        scales = [round(cfg[k]*65536)/65536 for k in
                  ('forward_mm_per_raw_count', 'lateral_mm_per_raw_count')]
        if min(scales) <= 0:
            raise ValueError('运动比例太小，FPGA Q16量化后为零')
        if self.points and (scales != self.scales or cfg != self.calibration):
            raise ValueError('已有示教点的标定不能更换；请新建示教，使用新标定重新记录')
        if scales != self.scales:
            self.invalidate('标定已更新，请重新设置示教起点')
        self.scales = scales
        self.calibration = copy.deepcopy(cfg)
        self.revision += 1

    def observe(self, motor, pose, now, pose_time):
        if self.last_observed is not None and now-self.last_observed >= .35:
            self.still_since = None
        self.last_observed = now
        stationary = (not motor['armed'] and not any(motor['velocity']) and not any(motor['pwm']))
        if not stationary:
            self.still_since = None
            self.replay_ready = False
        elif self.still_since is None:
            self.still_since = now
        if self.origin_yaw is None or not self.valid:
            return
        if (not pose or now-pose_time >= .3 or not pose.get('healthy') or
                pose.get('imu_stale') or not pose.get('calibrated')):
            self.invalidate('姿态过期或IMU未就绪，请重新设置起点；已有点仍可保存')
            return
        if self.last_time is not None and now-self.last_time >= .35:
            self.invalidate('示教遥测中断，不能补猜漏掉的路程；请重新示教')
            return
        positions = list(motor['position'])
        delta = [((a-b+(1 << 31)) % (1 << 32))-(1 << 31)
                 for a, b in zip(positions, self.previous)]
        if max(map(abs, delta)) > 20480:
            self.invalidate('编码器计数跳变，请重新设置起点')
            return
        self.heading = wrap(pose['yaw_deg']-self.origin_yaw)
        # Same raw-count axis convention and Q16 scales as mc_nav_odometry.
        forward = sum(delta)*self.scales[0]/4
        lateral = (-delta[0]+delta[1]+delta[2]-delta[3])*self.scales[1]/4
        cs, sn = math.cos(math.radians(self.heading)), math.sin(math.radians(self.heading))
        self.x += forward*cs-lateral*sn
        self.y += forward*sn+lateral*cs
        self.previous, self.last_time = positions, now
        if abs(self.x) > 4000 or abs(self.y) > 4000:
            self.invalidate('超出当前导航坐标范围 ±4000mm')
        if math.hypot(self.x, self.y) > 20 or abs(self.heading) > 5:
            self.replay_ready = False

    def require_stationary(self, motor, pose, now):
        if not motor or not pose or now-motor[0] >= .3 or now-pose[0] >= .3:
            raise ValueError('需要新鲜的电机与姿态遥测')
        m, p = motor[1], pose[1]
        if (m['armed'] or any(m['velocity']) or any(m['pwm']) or
                self.still_since is None or now-self.still_since < .3):
            raise ValueError('请先松开遥控，等待四轮未使能且连续静止300ms')
        if not p.get('healthy') or p.get('imu_stale') or not p.get('calibrated'):
            raise ValueError('请先完成IMU静止校零')

    def set_origin(self, motor, pose, now, *, replay=False):
        self.require_stationary(motor, pose, now)
        if replay and not self.points:
            raise ValueError('请先记录或载入路线')
        self.previous = list(motor[1]['position'])
        self.origin_yaw = pose[1]['yaw_deg']
        self.last_time = now
        self.x = self.y = self.heading = 0.
        self.valid, self.error = True, ''
        self.replay_ready = replay
        if not replay:
            self.points, self.name = [], ''
        self.revision += 1

    def record(self, motor, pose, now, speed=30):
        self.require_stationary(motor, pose, now)
        if not self.valid or self.last_time is None or now-self.last_time >= .3:
            raise ValueError(self.error or '示教坐标过期，请重新设置起点')
        if len(self.points) >= 16:
            raise ValueError('最多记录16个路径点')
        cap = min(50, self.calibration['max_speed_mm_s']) if self.calibration else 50
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or not 1 <= speed <= cap:
            raise ValueError('示教路径速度须在1～%s mm/s' % cap)
        last = self.points[-1] if self.points else [0, 0]
        if math.hypot(self.x-last[0], self.y-last[1]) < 150:
            raise ValueError('距上一个点不足150mm，请继续移动后记录')
        self.points.append([round(self.x), round(self.y), round(speed)])
        self.replay_ready = False
        self.revision += 1

    def route(self):
        if not self.points:
            raise ValueError('还没有路径点')
        return dict(format='mecanum-teach-route-v1', name=self.name,
                    coordinate_frame='start_x_forward_y_left_mm', points=copy.deepcopy(self.points),
                    calibration=copy.deepcopy(self.calibration), scales=list(self.scales),
                    measured=self.calibration is not None)

    def save(self, name):
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > 60:
            raise ValueError('路线名称须为1～60个字符')
        self.name = name.strip()
        route = self.route()
        route_id = uuid.uuid4().hex
        self.folder.mkdir(parents=True, exist_ok=True)
        temporary = self.folder/(route_id+'.tmp')
        temporary.write_text(json.dumps(route, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        temporary.replace(self.folder/(route_id+'.json'))
        return route_id

    def routes(self):
        result = []
        for path in sorted(self.folder.glob('*.json'), key=lambda p: p.stat().st_mtime, reverse=True):
            try:
                route = json.loads(path.read_text(encoding='utf-8'))
                result.append(dict(id=path.stem, name=route['name'], count=len(route['points']), measured=route['measured']))
            except (ValueError, KeyError, TypeError):
                continue
        return result

    def load(self, route):
        if not isinstance(route, dict) or route.get('format') != 'mecanum-teach-route-v1' or route.get('coordinate_frame') != 'start_x_forward_y_left_mm':
            raise ValueError('不是有效的麦轮示教路线文件')
        points = route.get('points')
        name = route.get('name', '导入路线')
        if not isinstance(name, str) or len(name) > 60:
            raise ValueError('路线名称无效')
        if not isinstance(points, list) or not 1 <= len(points) <= 16:
            raise ValueError('路线必须包含1～16个点')
        for point in points:
            if (not isinstance(point, list) or len(point) != 3 or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point) or
                    not (-4000 <= point[0] <= 4000 and -4000 <= point[1] <= 4000 and 1 <= point[2] <= 50)):
                raise ValueError('路线点坐标或速度无效')
        cfg = route.get('calibration')
        if cfg is not None:
            cfg = validate_calibration(cfg)
            json.dumps(cfg, allow_nan=False)
            upload_operations(cfg, points, 1)
            scales = [round(cfg[k]*65536)/65536 for k in ('forward_mm_per_raw_count','lateral_mm_per_raw_count')]
            if min(scales) <= 0:
                raise ValueError('路线运动比例量化后为零')
        else:
            scales = route.get('scales')
            if not isinstance(scales, list) or len(scales) != 2 or any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) or not 0 < v <= 1 for v in scales):
                raise ValueError('路线运动比例无效')
        self.calibration = copy.deepcopy(cfg)
        self.scales = list(scales)
        self.points = copy.deepcopy(points)
        self.name = name
        self.origin_yaw = None
        self.invalidate('路线已载入；请把车放回固定起点，对准车头后确认')
        self.revision += 1

    def load_id(self, route_id):
        if not isinstance(route_id, str) or not re.fullmatch('[0-9a-f]{32}', route_id):
            raise ValueError('无效路线ID')
        try:
            self.load(json.loads((self.folder/(route_id+'.json')).read_text(encoding='utf-8')))
        except OSError as exc:
            raise ValueError('路线文件不存在或无法读取') from exc

    def snapshot(self, now=None):
        now = self.last_observed if now is None else now
        stationary_ms = max(0, (now-self.still_since)*1000) if now is not None and self.still_since is not None else 0
        return dict(valid=self.valid, error=self.error, x_mm=round(self.x, 1), y_mm=round(self.y, 1),
                    heading_deg=round(self.heading, 1), points=copy.deepcopy(self.points), name=self.name,
                    measured=self.calibration is not None, calibration=copy.deepcopy(self.calibration),
                    replay_ready=self.replay_ready, revision=self.revision, stationary_ms=stationary_ms)
