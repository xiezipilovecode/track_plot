"""地图对齐可视化工具 - 用于验证轨迹与CARLA车道的吸附对齐

用法：
    python .\tools\trace_display.py

环境变量：
    TP_XODR_PATH      - OpenDRIVE地图路径
    TP_DATA_FILE_PATH - 轨迹数据文件路径
    TP_TUNNEL_ENTRY_X/Y/Z - 隧道入口锚点

注意：需要 CARLA PythonAPI，请先 activate carla 环境。
"""

import math
import os
import sys
from pathlib import Path
import numpy as np

# 延迟导入CARLA，参考 tp_replay.carla_compat 模式
def _require_carla():
    try:
        import carla
        return carla
    except ModuleNotFoundError:
        print("错误：CARLA PythonAPI 未安装。")
        print("请先运行: conda activate carla")
        sys.exit(1)


# 核心配置 - 支持环境变量覆盖
_XODR_PATH_DEFAULT = r"E:\carla\Unreal\CarlaUE4\Content\Carla\OpenDrive\QingShiLing.xodr"
_DATA_FILE_PATH_DEFAULT = r"E:\code\track_plot\trace_data\test_data\data_6lu1.txt"

XODR_PATH = os.getenv("TP_XODR_PATH", _XODR_PATH_DEFAULT)
DATA_FILE_PATH = os.getenv("TP_DATA_FILE_PATH", _DATA_FILE_PATH_DEFAULT)

# 锚点配置 - 支持环境变量覆盖
_TUNNEL_X_DEFAULT = -213549.140625 / 100.0
_TUNNEL_Y_DEFAULT = 55603.199219 / 100.0
_TUNNEL_Z_DEFAULT = 100.000000 / 100.0

TUNNEL_ENTRY_X = float(os.getenv("TP_TUNNEL_ENTRY_X", str(_TUNNEL_X_DEFAULT)))
TUNNEL_ENTRY_Y = float(os.getenv("TP_TUNNEL_ENTRY_Y", str(_TUNNEL_Y_DEFAULT)))
TUNNEL_ENTRY_Z = float(os.getenv("TP_TUNNEL_ENTRY_Z", str(_TUNNEL_Z_DEFAULT)))

# 变换配置
MANUAL_ROTATION_FIX = float(os.getenv("TP_MANUAL_ROTATION_FIX", "2.5"))
REVERSE_DIRECTION = os.getenv("TP_REVERSE_DIRECTION", "True").lower() == "true"
LATERAL_OFFSET = float(os.getenv("TP_LATERAL_OFFSET", "0.0"))
DATA_SCALE = float(os.getenv("TP_DATA_SCALE", "0.01"))
SCALE_Y = float(os.getenv("TP_SCALE_Y", "1.0"))
SNAP_THRESHOLD = float(os.getenv("TP_SNAP_THRESHOLD", "8.0"))


def read_file(filepath):
    if not os.path.exists(filepath):
        return []
    with open(filepath, 'r', encoding='utf-8') as f:
        tokens = f.read().replace('\n', ' ').split()
    points = []
    for i in range(0, len(tokens), 6):
        if i + 6 > len(tokens):
            break
        try:
            x_str, y_str = tokens[i + 1], tokens[i + 2]
            if 'null' in x_str or 'null' in y_str:
                continue
            x, y = float(x_str), float(y_str)
            if abs(x) > 1.0:
                points.append((x, y))
        except Exception:
            pass
    return points


def get_vector_angle_degrees(dx, dy):
    return math.degrees(math.atan2(dy, dx))


def is_in_front(point_loc, entry_loc, forward_vec):
    p_vec = np.array([point_loc.x, point_loc.y])
    e_vec = np.array([entry_loc.x, entry_loc.y])
    f_vec = np.array([forward_vec.x, forward_vec.y])
    return np.dot(p_vec - e_vec, f_vec) >= -2.0


def get_anchor_lane_info(carla_map, x, y, z):
    """获取指定坐标的路网信息"""
    loc = carla.Location(x, y, z)
    wp = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if not wp:
        return None, None

    anchor_loc = carla.Location(x, y, wp.transform.location.z)
    next_wps = wp.next(20.0)
    if next_wps:
        next_loc = next_wps[0].transform.location
        fwd_x = next_loc.x - anchor_loc.x
        fwd_y = next_loc.y - anchor_loc.y
        length = math.sqrt(fwd_x ** 2 + fwd_y ** 2)
        anchor_fwd = carla.Vector3D(fwd_x / length, fwd_y / length, 0)
    else:
        anchor_fwd = wp.transform.get_forward_vector()

    return anchor_loc, anchor_fwd


def calculate_stable_data_angle(points):
    if len(points) < 20:
        return 0
    p_start = points[0]
    target_idx = 5
    for i in range(5, len(points)):
        dist = math.sqrt((points[i][0] - p_start[0]) ** 2 + (points[i][1] - p_start[1]) ** 2) * DATA_SCALE
        if dist > 30.0:
            target_idx = i
            break

    p_end = points[target_idx]
    vec_dx = (p_end[0] - p_start[0]) * DATA_SCALE
    vec_dy = (p_end[1] - p_start[1]) * DATA_SCALE * SCALE_Y
    return get_vector_angle_degrees(vec_dx, vec_dy)


def main():
    global carla
    carla = _require_carla()
    client = carla.Client(
        os.getenv("TP_CARLA_HOST", "localhost"),
        int(os.getenv("TP_CARLA_PORT", "2000"))
    )
    client.set_timeout(10.0)
    world = client.get_world()
    debug = world.debug

    print(f"加载地图: {XODR_PATH}")
    with open(XODR_PATH, 'r') as f:
        xodr_data = f.read()
    carla_map = carla.Map('TunnelMap', xodr_data)

    print(f"锁定锚点: ({TUNNEL_ENTRY_X:.2f}, {TUNNEL_ENTRY_Y:.2f}) ...")
    entry_loc, entry_fwd = get_anchor_lane_info(carla_map, TUNNEL_ENTRY_X, TUNNEL_ENTRY_Y, TUNNEL_ENTRY_Z)

    if not entry_loc:
        print("错误：锚点不在路网上")
        return

    map_angle = get_vector_angle_degrees(entry_fwd.x, entry_fwd.y)
    print(f"地图长基线朝向: {map_angle:.2f} 度")

    raw_points = read_file(DATA_FILE_PATH)
    if len(raw_points) < 20:
        return

    data_angle = calculate_stable_data_angle(raw_points)
    print(f"数据长基线朝向: {data_angle:.2f} 度")

    rotation_diff = map_angle - data_angle
    if REVERSE_DIRECTION:
        rotation_diff += 180.0
        entry_fwd.x = -entry_fwd.x
        entry_fwd.y = -entry_fwd.y
        right_vec = carla.Vector3D(entry_fwd.y, -entry_fwd.x, 0)
    else:
        right_vec = carla.Vector3D(entry_fwd.y, -entry_fwd.x, 0)

    rotation_diff += MANUAL_ROTATION_FIX
    print(f"最终应用旋转: {rotation_diff:.2f} 度 (含手动修正 {MANUAL_ROTATION_FIX})")

    debug.draw_point(entry_loc, size=0.6, color=carla.Color(255, 255, 0), life_time=100.0)
    debug.draw_arrow(entry_loc, entry_loc + entry_fwd * 20.0, thickness=0.5, arrow_size=0.6, color=carla.Color(255, 0, 0), life_time=100.0)

    print("开始绘制...")
    data_start = raw_points[0]
    rot_rad = math.radians(rotation_diff)
    cos_a = math.cos(rot_rad)
    sin_a = math.sin(rot_rad)

    offset_x = right_vec.x * LATERAL_OFFSET
    offset_y = right_vec.y * LATERAL_OFFSET

    valid_count = 0
    last_wp = None
    prev_rough_loc = None
    prev_blue = None
    prev_green = None

    for i, (dx, dy) in enumerate(raw_points):
        rx = (dx - data_start[0]) * DATA_SCALE
        ry = (dy - data_start[1]) * DATA_SCALE * SCALE_Y

        tx = rx * cos_a - ry * sin_a + entry_loc.x + offset_x
        ty = rx * sin_a + ry * cos_a + entry_loc.y + offset_y

        rough_loc = carla.Location(tx, ty, entry_loc.z)

        if not is_in_front(rough_loc, entry_loc, entry_fwd):
            continue

        blue_loc = carla.Location(rough_loc.x, rough_loc.y, rough_loc.z + 0.5)
        debug.draw_point(blue_loc, size=0.08, color=carla.Color(0, 0, 255), life_time=100.0)
        if prev_blue:
            debug.draw_line(prev_blue, blue_loc, thickness=0.02, color=carla.Color(80, 80, 255), life_time=100.0)
        prev_blue = blue_loc

        chosen_wp = None
        if last_wp and prev_rough_loc:
            step = math.hypot(rough_loc.x - prev_rough_loc.x, rough_loc.y - prev_rough_loc.y)
            step = max(0.5, min(step, 30.0))
            cands = last_wp.previous(step) if REVERSE_DIRECTION else last_wp.next(step)
            if cands:
                chosen_wp = min(cands, key=lambda w: rough_loc.distance(w.transform.location))
                if rough_loc.distance(chosen_wp.transform.location) > SNAP_THRESHOLD:
                    chosen_wp = None
        if not chosen_wp:
            wp = carla_map.get_waypoint(rough_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp and rough_loc.distance(wp.transform.location) <= SNAP_THRESHOLD:
                chosen_wp = wp
        if chosen_wp:
            final_loc = chosen_wp.transform.location
            debug.draw_line(blue_loc, final_loc, thickness=0.02, color=carla.Color(200, 200, 200), life_time=100.0)
            draw_loc = carla.Location(final_loc.x, final_loc.y, final_loc.z + 0.6)
            debug.draw_point(draw_loc, size=0.12, color=carla.Color(0, 255, 0), life_time=100.0)
            if prev_green:
                debug.draw_line(prev_green, draw_loc, thickness=0.03, color=carla.Color(0, 220, 0), life_time=100.0)
            prev_green = draw_loc
            valid_count += 1
            last_wp = chosen_wp
            prev_rough_loc = rough_loc

        if i == 0:
            cam = carla.Location(entry_loc.x - entry_fwd.x * 40, entry_loc.y - entry_fwd.y * 40, entry_loc.z + 20)
            world.get_spectator().set_transform(carla.Transform(cam, carla.Rotation(pitch=-20, yaw=math.degrees(math.atan2(entry_fwd.y, entry_fwd.x)))))

    print(f"绘制完成。")
    print("-" * 40)
    print("【调试指南 - 解决车道漂移】")
    print("1. 观察【蓝色点】(原始) 和【绿色点】(车道) 之间的【白线】。")
    print("2. 如果越往远处走，白线越长，说明角度有偏差。")
    print("3. 如果蓝点逐渐偏向车流的【左边】，请修改 MANUAL_ROTATION_FIX 为负数 (如 -1.0)。")
    print("4. 如果蓝点逐渐偏向车流的【右边】，请修改 MANUAL_ROTATION_FIX 为正数 (如 1.0)。")
    print("-" * 40)


if __name__ == '__main__':
    main()
