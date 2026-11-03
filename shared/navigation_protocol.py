"""Mecanum navigation V4 commands / N4 telemetry. Pure codec, no IO."""
import math
import struct
from host.motor_protocol import crc16

OPS = dict(config=0x40, begin=0x50, point=0x51, commit=0x52, start=0x53,
           pause=0x54, resume=0x55, cancel=0x56, heartbeat=0x57)
REASONS = {0: '无', 1: '人工暂停', 2: '控制租约/使能失效', 4: '传感器过期', 5: '持续受阻'}


def packet(operation, sequence, mission_id=0, index=0, count=0, a=0, b=0, c=0):
    op = OPS[operation] if isinstance(operation, str) else operation
    if op not in OPS.values():
        raise ValueError('Unknown navigation operation')
    body = struct.pack('<2sBBBBHHBBiiH', b'\xa5\x5a', 4, op, sequence & 255, 0, 0,
                       mission_id, index, count, a, b, c & 65535)
    assert len(body) == 22
    return body + struct.pack('<H', crc16(body))


def decode(frame):
    if len(frame) != 48 or frame[:3] != b'N4\x01' or crc16(frame[:46]) != struct.unpack_from('<H', frame, 46)[0]:
        raise ValueError('Invalid N4 navigation telemetry')
    flags = frame[3]
    x, y, heading = struct.unpack_from('<3i', frame, 12)
    gx, gy, vx, vy = struct.unpack_from('<4h', frame, 24)
    return dict(kind='navigation', active=bool(flags & 1), running=bool(flags & 2),
                permit=bool(flags & 4), flow_degraded=bool(flags & 8), map_fresh=bool(flags & 16),
                mission_id=struct.unpack_from('<H', frame, 4)[0], waypoint=frame[6], status=frame[7],
                ack_sequence=frame[8], ack_error=frame[9], reason=frame[10], calibrated=bool(frame[11] & 1),
                x_mm=x/65536., y_mm=y/65536., heading_deg=math.degrees(heading/65536.),
                goal_x_mm=gx, goal_y_mm=gy, vx_mm_s=vx, vy_mm_s=vy,
                device_ms=struct.unpack_from('<I', frame, 32)[0], map_epoch=struct.unpack_from('<H', frame, 36)[0],
                point_overflows=struct.unpack_from('<H', frame, 38)[0], planner_cycles=struct.unpack_from('<I', frame, 40)[0],
                danger_mask=frame[44], unknown_motion=bool(frame[45] & 1), search_limited=bool(frame[45] & 2),
                planner_timeout=bool(frame[45] & 4))


def validate_calibration(config):
    """Only measured, explicitly confirmed values may create an enabled config."""
    if not isinstance(config, dict) or config.get('confirmed') is not True:
        raise ValueError('请先填写并确认实测麦轮/外廓/雷达/停止距离标定')
    required = ('front_mm', 'back_mm', 'half_width_mm', 'radius_mm', 'pad_mm',
                'forward_mm_per_raw_count', 'lateral_mm_per_raw_count', 'rotation_lever_mm',
                'laser_x_mm', 'laser_y_mm', 'laser_yaw_deg', 'max_speed_mm_s', 'accel_mm_s2',
                'measured_stop_mm', 'deceleration_mm_s2')
    for key in required:
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('缺少有效实测参数: ' + key)
    f, b, w, r, pad = (config[k] for k in ('front_mm', 'back_mm', 'half_width_mm', 'radius_mm', 'pad_mm'))
    if not (50 <= f <= 600 and 50 <= b <= 600 and 30 <= w <= 300 and
            math.hypot(max(f, b), w) <= r <= 800 and 56 <= pad <= 400):
        raise ValueError('外廓、外接圆或裕量越界')
    speed, acc, dec, stop = (config[k] for k in ('max_speed_mm_s', 'accel_mm_s2', 'deceleration_mm_s2', 'measured_stop_mm'))
    if not (1 <= speed <= 150 and 20 <= acc <= 1000 and dec > 0 and 0 < stop <= pad-36):
        raise ValueError('速度/实测停止距离与足迹裕量不匹配')
    if speed/dec > 1.8 or speed*speed/(2*dec) > stop:
        raise ValueError('制动尾段必须覆盖在2秒预测及实测停止包络内')
    if not (0 < config['forward_mm_per_raw_count'] <= 1 and 0 < config['lateral_mm_per_raw_count'] <= 1 and
            50 <= config['rotation_lever_mm'] <= 800 and abs(config['laser_x_mm']) <= 500 and abs(config['laser_y_mm']) <= 500 and abs(config['laser_yaw_deg']) <= 180):
        raise ValueError('麦轮比例或雷达外参越界')
    masks = config.get('self_masks', [])
    if not isinstance(masks, list) or len(masks) > 2:
        raise ValueError('最多两个已测量自反射掩码')
    for mask in masks:
        if not isinstance(mask, (list, tuple)) or len(mask) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in mask):
            raise ValueError('自反射掩码格式为[起始度,终止度,最大毫米]')
        if not (0 <= mask[0] < 360 and 0 <= mask[1] < 360 and 0 <= mask[2] <= 500):
            raise ValueError('自反射掩码越界')
    return dict(config)


def upload_operations(config, points, mission_id):
    cfg = validate_calibration(config)
    if not isinstance(points, list) or not 1 <= len(points) <= 16 or not 1 <= mission_id <= 65535:
        raise ValueError('任务需1～16个路径点和有效任务ID')
    ops = []
    def add(operation, **kwargs):
        ops.append(dict(operation=operation, mission_id=mission_id, **kwargs))
    for index, a, b, c in (
        (0, cfg['front_mm'], cfg['back_mm'], cfg['half_width_mm']),
        (1, cfg['radius_mm'], cfg['pad_mm'], 0),
        (2, round(cfg['forward_mm_per_raw_count']*65536), round(cfg['lateral_mm_per_raw_count']*65536), 0),
        (3, cfg['laser_x_mm'], cfg['laser_y_mm'], round(cfg['laser_yaw_deg']*100)),
        (4, cfg['max_speed_mm_s'], cfg['accel_mm_s2'], 0),
        (5, cfg['rotation_lever_mm'], 0, 0)):
        add('config', index=index, a=round(a), b=round(b), c=round(c))
    for index, (start, end, distance) in enumerate(cfg.get('self_masks', []), 6):
        add('config', index=index, a=round(start*100) % 36000, b=round(end*100) % 36000, c=round(distance))
    add('config', index=15, c=0x4e41)
    add('begin', count=len(points))
    table = bytearray()
    for index, point in enumerate(points):
        if not isinstance(point, (list, tuple)) or len(point) not in (2, 3):
            raise ValueError('路径点格式为[x_mm,y_mm,speed_mm_s]')
        x, y = point[:2]; speed = point[2] if len(point) == 3 else cfg['max_speed_mm_s']
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (x, y, speed)) or not (-4000 <= x <= 4000 and -4000 <= y <= 4000 and 1 <= speed <= cfg['max_speed_mm_s']):
            raise ValueError('路径点坐标或速度越界')
        x, y, speed = round(x), round(y), round(speed)
        table.extend(struct.pack('<iiH', x, y, speed))
        add('point', index=index, a=x, b=y, c=speed)
    add('commit', c=crc16(table))
    return ops
