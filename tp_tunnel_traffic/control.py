from __future__ import annotations

import math


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _norm_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _first_order_lpf(prev: float, x: float, alpha: float) -> float:
    a = _clamp(alpha, 0.0, 1.0)
    return (1.0 - a) * prev + a * x


def _pick_lookahead_wp(path_points, nearest_idx: int, lookahead_m: float):
    if not path_points:
        return None
    i = int(_clamp(float(nearest_idx), 0.0, float(len(path_points) - 1)))
    if lookahead_m <= 0.0:
        return path_points[i]
    start = path_points[i].transform.location
    acc = 0.0
    while i < len(path_points) - 1 and acc < lookahead_m:
        a = path_points[i].transform.location
        b = path_points[i + 1].transform.location
        dx = b.x - a.x
        dy = b.y - a.y
        dz = b.z - a.z
        acc += math.sqrt(dx * dx + dy * dy + dz * dz)
        i += 1
    return path_points[i]


def compute_control(
    carla,
    vehicle,
    target_wp,
    target_speed_mps: float,
    follow_distance_m: float,
    front_gap_m: float | None = None,
    *,
    path_points=None,
    nearest_idx: int | None = None,
    lookahead_m: float = 0.0,
    steer_lpf_alpha: float = 0.0,
    steer_max_rate: float = 0.0,
    dt_seconds: float | None = None,
):
    loc = vehicle.get_location()
    rot = vehicle.get_transform().rotation
    vel = vehicle.get_velocity()
    speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)

    # 可选：使用路径前视点（更稳定）替代“最近点”
    if path_points is not None and nearest_idx is not None and lookahead_m > 1e-6:
        wp = _pick_lookahead_wp(path_points, int(nearest_idx), float(lookahead_m))
        target_loc = wp.transform.location if wp is not None else target_wp.transform.location
    else:
        target_loc = target_wp.transform.location
    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y
    desired_yaw = math.degrees(math.atan2(dy, dx))
    yaw_error = _norm_angle_deg(desired_yaw - rot.yaw)

    raw_steer = _clamp(yaw_error / 35.0, -1.0, 1.0)

    # 取上一帧 steer 做平滑/限速（把状态挂在 vehicle 上，避免全局变量）
    prev_steer = float(getattr(vehicle, "_tt_prev_steer", 0.0))
    # 在同步 tick 下，vehicle 侧通常拿不到可靠时间戳；优先用外层传入的 dt
    dt = 0.05
    if dt_seconds is not None and dt_seconds > 1e-4:
        dt = float(dt_seconds)

    steer = raw_steer
    if steer_lpf_alpha > 1e-6:
        steer = _first_order_lpf(prev_steer, steer, float(steer_lpf_alpha))

    if steer_max_rate > 1e-6:
        max_delta = float(steer_max_rate) * float(dt)
        steer = _clamp(steer, prev_steer - max_delta, prev_steer + max_delta)
    steer = _clamp(steer, -1.0, 1.0)

    # 速度控制
    speed_error = target_speed_mps - speed
    throttle = _clamp(speed_error / max(target_speed_mps, 0.1), 0.0, 0.6)
    brake = 0.0

    # 前车太近时减速/刹车
    if front_gap_m is not None and front_gap_m < follow_distance_m:
        throttle = 0.0
        brake = _clamp((follow_distance_m - front_gap_m) / max(follow_distance_m, 0.1), 0.0, 1.0)

    # 角度偏差大时，降低油门，避免冲出车道
    if abs(yaw_error) > 35.0:
        throttle *= 0.5

    # 转向较大时额外降速，减少“打方向还在给油”导致的摆振
    if abs(steer) > 0.55:
        throttle *= 0.6

    control = carla.VehicleControl()
    control.throttle = throttle
    control.brake = brake
    control.steer = steer
    control.hand_brake = False
    control.reverse = False

    setattr(vehicle, "_tt_prev_steer", float(steer))
    return control
