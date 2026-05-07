"""线宽调试工具 - 用于调试轨迹与车道的缩放比例"""

import math
import os
import sys

# 延迟导入CARLA
def _require_carla():
    try:
        import carla
        return carla
    except ModuleNotFoundError:
        print("错误：CARLA PythonAPI 未安装。请先运行: conda activate carla")
        sys.exit(1)


# 配置 - 支持环境变量覆盖
XODR_PATH = os.getenv("TP_XODR_PATH", r"E:\carla\Unreal\CarlaUE4\Content\Carla\OpenDrive\QingShiLing.xodr")
DATA_FILE_PATH = os.getenv("TP_DATA_FILE_PATH", r"E:\code\track_plot\data_12lu.txt")

CARLA_START_X = float(os.getenv("TP_CARLA_START_X", "1245.36304688"))
CARLA_START_Y = float(os.getenv("TP_CARLA_START_Y", "-562.05199219"))
CARLA_START_Z = float(os.getenv("TP_CARLA_START_Z", "0.5"))

DATA_SCALE = float(os.getenv("TP_DATA_SCALE", "0.01"))
ROTATION_OFFSET = float(os.getenv("TP_ROTATION_OFFSET", "90"))
SCALE_Y = float(os.getenv("TP_SCALE_Y", "1.0"))


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


def main():
    global carla
    carla = _require_carla()
    client = carla.Client(
        os.getenv("TP_CARLA_HOST", "localhost"),
        int(os.getenv("TP_CARLA_PORT", "2000"))
    )
    world = client.get_world()
    debug = world.debug

    print("正在绘制地图参考线 (白色)...")
    with open(XODR_PATH, 'r') as f:
        xodr_data = f.read()
    carla_map = carla.Map('Map', xodr_data)

    map_waypoints = carla_map.generate_waypoints(distance=2.0)
    for wp in map_waypoints:
        if wp.lane_type == carla.LaneType.Driving:
            debug.draw_point(wp.transform.location, size=0.1, color=carla.Color(200, 200, 200), life_time=100.0)

    raw_points = read_file(DATA_FILE_PATH)
    if not raw_points:
        print("未读取到数据")
        return

    data_start_x = raw_points[0][0]
    data_start_y = raw_points[0][1]

    print(f"当前缩放因子: {DATA_SCALE}")
    print("正在绘制缩放后的轨迹 (红色)...")

    rot_rad = math.radians(ROTATION_OFFSET)

    for i, (dx, dy) in enumerate(raw_points):
        rel_x = dx - data_start_x
        rel_y = dy - data_start_y
        rel_x *= DATA_SCALE
        rel_y *= DATA_SCALE
        rel_y *= SCALE_Y
        rot_x = rel_x * math.cos(rot_rad) - rel_y * math.sin(rot_rad)
        rot_y = rel_x * math.sin(rot_rad) + rel_y * math.cos(rot_rad)
        final_x = rot_x + CARLA_START_X
        final_y = rot_y + CARLA_START_Y

        loc = carla.Location(final_x, final_y, CARLA_START_Z)
        debug.draw_point(loc, size=0.15, color=carla.Color(255, 0, 0), life_time=100.0)

        if i == 0:
            world.get_spectator().set_transform(carla.Transform(
                carla.Location(final_x, final_y, CARLA_START_Z + 50),
                carla.Rotation(pitch=-90)
            ))

    print("\n【缩放调试指南】")
    print("1. 如果红线还是很宽（比白线宽）：把 DATA_SCALE 减小（例如 0.1 -> 0.01）")
    print("2. 如果红线缩成了一团（比白线窄）：把 DATA_SCALE 增大（例如 0.1 -> 1.0）")
    print("3. 当红线的宽度和白线车道宽度一致时，即可固定该参数。")


if __name__ == '__main__':
    main()
