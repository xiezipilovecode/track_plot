"""地图起点获取工具 - 用于确定隧道入口位置"""

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

    print(f"正在读取地图: {XODR_PATH}")
    if not os.path.exists(XODR_PATH):
        print("错误：文件不存在！")
        return

    with open(XODR_PATH, 'r', encoding='utf-8') as f:
        xodr_data = f.read()
    carla_map = carla.Map('TunnelMap', xodr_data)

    print("正在分析道路拓扑...")
    topology = carla_map.get_topology()
    waypoints = carla_map.generate_waypoints(distance=1.0)

    if not waypoints:
        print("地图中未找到车道！")
        return

    filtered_wps = [wp for wp in waypoints if wp.lane_type == carla.LaneType.Driving]
    sorted_by_x = sorted(filtered_wps, key=lambda wp: wp.transform.location.x)
    end_a = sorted_by_x[0]
    end_b = sorted_by_x[-1]

    print("正在绘制标记...")

    def draw_marker(wp, label, color):
        loc = wp.transform.location
        debug.draw_point(loc, size=0.5, color=color, life_time=120.0)
        debug.draw_string(carla.Location(loc.x, loc.y, loc.z + 2.0),
                          label, draw_shadow=True, color=color, life_time=120.0)
        fwd = wp.transform.get_forward_vector()
        arrow_end = loc + fwd * 5.0
        debug.draw_arrow(loc, arrow_end, thickness=0.2, arrow_size=0.5, color=color, life_time=120.0)
        print(f"发现端点 [{label}]:")
        print(f"  坐标: X={loc.x:.2f}, Y={loc.y:.2f}, Z={loc.z:.2f}")
        print(f"  朝向: Yaw={wp.transform.rotation.yaw:.2f}")
        print("-" * 30)

    draw_marker(end_a, "POSSIBLE START A", carla.Color(0, 255, 0))
    draw_marker(end_b, "POSSIBLE START B", carla.Color(255, 0, 0))

    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        carla.Location(end_a.transform.location.x, end_a.transform.location.y, end_a.transform.location.z + 20),
        carla.Rotation(pitch=-90)
    ))
    print("视角已移动到端点 A。请在 CARLA 中查看绿色和红色标记。")
    print("箭头指向隧道内部的那一端，就是你要找的起始点。")


if __name__ == '__main__':
    main()
