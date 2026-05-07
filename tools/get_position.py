"""坐标采集工具 - 实时显示CARLA观察者视角坐标"""

import os
import sys
import time

# 延迟导入CARLA
def _require_carla():
    try:
        import carla
        return carla
    except ModuleNotFoundError:
        print("错误：CARLA PythonAPI 未安装。请先运行: conda activate carla")
        sys.exit(1)


def main():
    global carla
    carla = _require_carla()
    client = carla.Client(
        os.getenv("TP_CARLA_HOST", "localhost"),
        int(os.getenv("TP_CARLA_PORT", "2000"))
    )
    world = client.get_world()
    spectator = world.get_spectator()

    print("=== CARLA 坐标采集工具 ===")
    print("请移动视角(Spectator)到你想要的位置...")

    try:
        while True:
            t = spectator.get_transform()
            loc = t.location
            yaw = t.rotation.yaw
            print(f"当前坐标: x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f} | Yaw={yaw:.2f}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("停止采集")


if __name__ == '__main__':
    main()
