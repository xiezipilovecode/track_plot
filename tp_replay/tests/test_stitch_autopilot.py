"""拼接轨迹隧道测试脚本。

加载拼接 JSON，选择一条高质量轨迹，在 CARLA 隧道中生成车辆，
spectator 跟随该车行驶，用于验证拼接结果与仿真车道的对应情况。

用法：
    set TP_STITCH_JSON_PATH=E:\code\track_plot\trace_data\track_stitch\output\stitched_trajectories.json
    python -m tp_replay.tests.test_stitch_autopilot
"""

from __future__ import annotations

import logging
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tp_replay import config
from tp_replay.carla_compat import require_carla
from tp_replay.engine import ReplayEngine
from tp_replay.stitch_adapter import StitchAdapter
from tp_replay.stitch_autopilot import StitchAutopilot

logger = logging.getLogger(__name__)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    carla = require_carla()
    stitch_json = os.getenv("TP_STITCH_JSON_PATH", "")
    if not stitch_json:
        print("请设置 TP_STITCH_JSON_PATH 环境变量")
        return

    # ── 连接 CARLA ──
    host = os.getenv("TP_CARLA_HOST", "localhost")
    port = int(os.getenv("TP_CARLA_PORT", "2000"))
    client = carla.Client(host, port)
    client.set_timeout(15.0)
    world = client.get_world()

    # ── 同步模式 ──
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # ── 用锚点文件计算坐标变换参数 ──
    anchor_file = os.getenv("TP_DATA_FILE_PATH", config.DATA_FILE_PATH)
    if not anchor_file or not os.path.exists(anchor_file):
        print(f"锚点文件不存在: {anchor_file}")
        return

    engine = ReplayEngine(client, config.XODR_PATH)
    engine.process_data(anchor_file)
    engine.pending_tracks = []

    print(f"地图方向: {engine.map_angle:.2f}°")
    print(f"锚点: ({engine.entry_loc.x:.1f}, {engine.entry_loc.y:.1f}, {engine.entry_loc.z:.1f})")

    # ── 加载拼接轨迹 ──
    adapter = StitchAdapter(engine)
    all_tracks = adapter.load_and_convert(stitch_json, min_quality=0.7, min_cameras=3, max_tracks=50)

    # 只取前 5 分钟内开始的轨迹
    all_tracks = sorted(all_tracks, key=lambda t: t.start_time)
    first_ts = all_tracks[0].start_time if all_tracks else 0
    tracks = [t for t in all_tracks if t.start_time - first_ts < 300.0]  # 5 分钟窗口

    if not tracks:
        print("没有找到符合条件的轨迹")
        return

    print(f"\n加载 {len(tracks)} 条轨迹（前 5 分钟窗口）")

    # 打印首条轨迹详情
    t = tracks[0]
    print(f"\n首条轨迹详情:")
    print(f"  ID: {t.id}")
    print(f"  类型: {t.type_str}")
    print(f"  首帧时间: {t.start_time:.1f}s")
    print(f"  末帧时间: {t.end_time:.1f}s")
    print(f"  时长: {t.end_time - t.start_time:.1f}s")
    print(f"  速度点数: {len(getattr(t, 'speed_profile', []))}")
    if t.frames:
        f = t.frames[0]
        print(f"  Spawn 位置: ({f.loc.x:.1f}, {f.loc.y:.1f}, {f.loc.z:.1f})")
        print(f"  Spawn 速度: {f.v:.1f} m/s ({f.v*3.6:.1f} km/h)")

    # ── TM 初始化 ──
    tm_port = int(os.getenv("TP_STITCH_TM_PORT", "8000"))
    tm = client.get_trafficmanager(tm_port)
    tm.set_synchronous_mode(True)
    tm.set_global_distance_to_leading_vehicle(5.0)

    # ── 生成第一辆车并跟随 ──
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find("vehicle.tesla.model3")
    bp.set_attribute("role_name", "stitch_test")

    t = tracks[0]
    spawn_frame = t.frames[0]
    spawn_loc = spawn_frame.loc
    spawn_rot = spawn_frame.rot

    print(f"\n尝试生成车辆 @ ({spawn_loc.x:.1f}, {spawn_loc.y:.1f}, {spawn_loc.z:.1f})")
    transform = carla.Transform(spawn_loc, spawn_rot)
    vehicle = world.try_spawn_actor(bp, transform)

    if vehicle is None:
        print("生成失败！尝试偏移位置...")
        # 尝试在附近找路点
        wp = engine.map.get_waypoint(spawn_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp:
            spawn_loc = wp.transform.location
            spawn_loc.z += 0.5
            print(f"  吸附到车道: ({spawn_loc.x:.1f}, {spawn_loc.y:.1f}, {spawn_loc.z:.1f})")
            transform = carla.Transform(spawn_loc, wp.transform.rotation)
            vehicle = world.try_spawn_actor(bp, transform)

    if vehicle is None:
        print("仍然失败，退出")
        return

    print(f"车辆已生成: {vehicle.id}")

    # 启用 autopilot
    vehicle.set_autopilot(True, tm_port)
    tm.ignore_lights_percentage(vehicle, 100.0)
    tm.auto_lane_change(vehicle, False)

    # ── Spectator 跟随 ──
    print("\nSpectator 跟随车辆，按 Ctrl+C 退出...")
    spectator = world.get_spectator()

    try:
        tick = 0
        while True:
            world.tick()
            tick += 1

            # Spectator 跟随车辆后方上方
            v_transform = vehicle.get_transform()
            v_forward = v_transform.get_forward_vector()
            cam_loc = carla.Location(
                v_transform.location.x - v_forward.x * 8,
                v_transform.location.y - v_forward.y * 8,
                v_transform.location.z + 5,
            )
            spectator.set_transform(carla.Transform(cam_loc, v_transform.rotation))

            if tick % 100 == 0:
                v = vehicle.get_velocity()
                speed = math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6
                print(f"  tick={tick} speed={speed:.1f} km/h "
                      f"pos=({v_transform.location.x:.0f},{v_transform.location.y:.0f})")

    except KeyboardInterrupt:
        print("\n退出")
    finally:
        vehicle.destroy()
        world.apply_settings(world.get_settings())


if __name__ == "__main__":
    main()
