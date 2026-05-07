from __future__ import annotations

"""隧道三车道车道线显示测试入口。

用途：
- 只提取并显示三车道车道线
- 不启用自动驾驶，方便单独检查当前三车道是否完整、是否有重合或缺失

运行：
    python -m tp_tunnel_traffic.tests.test_tunnel_lanes
"""

import math
from pathlib import Path
import importlib
import sys
import os

if __package__ is None or __package__ == "":
    sys.path.append(str(Path(__file__).resolve().parents[2]))


def run_tunnel_lane_view() -> None:
    module = importlib.import_module("tp_tunnel_traffic.main")
    carla = module.require_carla()
    TunnelTrafficConfig = importlib.import_module("tp_tunnel_traffic.config").TunnelTrafficConfig
    build_three_lane_paths = importlib.import_module("tp_tunnel_traffic.lane_sampling").build_three_lane_paths

    config = TunnelTrafficConfig()
    client = carla.Client(config.carla_host, config.carla_port)
    client.set_timeout(config.carla_timeout)
    world = client.get_world()

    original_settings = None
    try:
        original_settings = world.get_settings()
        if config.sync_mode:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(config.fixed_delta_seconds)
            world.apply_settings(settings)

        lanes = build_three_lane_paths(carla, xodr_path=config.xodr_path, step_m=config.step_m, spawn_z=1.0)
        print(f"左车道采样点数量: {len(lanes['-1'])}")
        print(f"中间车道采样点数量: {len(lanes['-2'])}")
        print(f"右车道采样点数量: {len(lanes['-3'])}")

        min_count = min(len(lanes["-1"]), len(lanes["-2"]), len(lanes["-3"]))
        assert min_count > 0, "三车道采样为空"

        expected_lane_width_m = float(os.getenv("TT_LANE_WIDTH_M", "3.5"))
        width_tol_m = float(os.getenv("TT_LANE_WIDTH_TOL_M", "0.20"))
        spacing_tol_m = float(os.getenv("TT_LANE_SPACING_TOL_M", "0.05"))

        print(
            "诊断: 每隔 40 个采样点打印一次三车道间距（归一化模型：左右车道由中间车道按常量车道宽度合成）"
        )
        print(
            f"期望车道宽度={expected_lane_width_m:.3f}m 宽度容差=±{width_tol_m:.3f}m 等间距容差=±{spacing_tol_m:.3f}m"
        )
        for i in range(min_count):
            p1 = lanes["-1"][i].transform.location
            p2 = lanes["-2"][i].transform.location
            p3 = lanes["-3"][i].transform.location
            d12 = p1.distance(p2)
            d23 = p2.distance(p3)
            d13 = p1.distance(p3)
            assert d12 > 0.5 and d23 > 0.5 and d13 > 1.0, f"第 {i} 个采样点三车道距离过近: {d12}, {d23}, {d13}"

            # 归一化模型期望：d12≈d23≈lane_width, d13≈2*lane_width
            assert abs(d12 - d23) <= spacing_tol_m, (
                f"第 {i} 个采样点左右间距不相等: d12={d12:.3f} d23={d23:.3f} Δ={abs(d12-d23):.3f}"
            )
            assert abs(d12 - expected_lane_width_m) <= width_tol_m, (
                f"第 {i} 个采样点 d12 偏离期望车道宽度: d12={d12:.3f} expected={expected_lane_width_m:.3f}"
            )
            assert abs(d23 - expected_lane_width_m) <= width_tol_m, (
                f"第 {i} 个采样点 d23 偏离期望车道宽度: d23={d23:.3f} expected={expected_lane_width_m:.3f}"
            )
            assert abs(d13 - expected_lane_width_m * 2.0) <= width_tol_m * 2.0, (
                f"第 {i} 个采样点 d13 偏离 2x 车道宽度: d13={d13:.3f} expected={expected_lane_width_m*2.0:.3f}"
            )
            if i % 40 == 0:
                print(
                    f"diag i={i} d12={d12:.3f} d23={d23:.3f} d13={d13:.3f} "
                    f"Δ={abs(d12-d23):.3f} err12={d12-expected_lane_width_m:+.3f} err23={d23-expected_lane_width_m:+.3f}"
                )

        debug = world.debug
        for lane_id, color in (("-1", carla.Color(0, 255, 0)), ("-2", carla.Color(0, 128, 255)), ("-3", carla.Color(255, 0, 0))):
            pts = lanes[lane_id]
            for i, pt in enumerate(pts):
                loc = pt.transform.location
                debug.draw_point(loc, size=0.12, color=color, life_time=300.0)
                if i > 0:
                    prev = pts[i - 1].transform.location
                    debug.draw_line(prev, loc, thickness=0.04, color=color, life_time=300.0)

        for i in range(0, min_count, 40):
            p1 = lanes["-1"][i].transform.location
            p2 = lanes["-2"][i].transform.location
            p3 = lanes["-3"][i].transform.location
            d12 = p1.distance(p2)
            d23 = p2.distance(p3)
            if i + 1 < min_count:
                n = lanes["-2"][i].transform.location
                n2 = lanes["-2"][i + 1].transform.location
                tx = n2.x - n.x
                ty = n2.y - n.y
                lx = p1.x - p2.x
                ly = p1.y - p2.y
                rx = p3.x - p2.x
                ry = p3.y - p2.y
                tlen = max((tx * tx + ty * ty) ** 0.5, 1e-9)
                llen = max((lx * lx + ly * ly) ** 0.5, 1e-9)
                rlen = max((rx * rx + ry * ry) ** 0.5, 1e-9)
                dot_left = abs((tx / tlen) * (lx / llen) + (ty / tlen) * (ly / llen))
                dot_right = abs((tx / tlen) * (rx / rlen) + (ty / tlen) * (ry / rlen))
            else:
                dot_left = 0.0
                dot_right = 0.0
            label = (
                f"d12={d12:.2f} d23={d23:.2f} Δ={abs(d12-d23):.2f} "
                f"w={expected_lane_width_m:.2f} dot={max(dot_left, dot_right):.2f}"
            )
            debug.draw_string(p2, label, color=carla.Color(255, 255, 255), life_time=300.0, draw_shadow=True)
            debug.draw_line(p1, p2, thickness=0.08, color=carla.Color(255, 255, 255), life_time=300.0)
            debug.draw_line(p2, p3, thickness=0.08, color=carla.Color(255, 255, 255), life_time=300.0)

        # 将视角对准反转后的新起点（中间车道的前两个采样点）
        if len(lanes["-2"]) >= 2:
            start_tf = lanes["-2"][0].transform
            next_tf = lanes["-2"][1].transform
            heading = math.atan2(next_tf.location.y - start_tf.location.y, next_tf.location.x - start_tf.location.x)
            eye = carla.Location(
                start_tf.location.x - math.cos(heading) * 14.0,
                start_tf.location.y - math.sin(heading) * 14.0,
                start_tf.location.z + 8.0,
            )
            world.get_spectator().set_transform(
                carla.Transform(
                    eye,
                    carla.Rotation(pitch=-16.0, yaw=math.degrees(heading), roll=0.0),
                )
            )

        print("三车道车道线已绘制，按 Ctrl+C 结束")
        while True:
            world.tick()

    except KeyboardInterrupt:
        pass
    finally:
        if original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass


if __name__ == "__main__":
    run_tunnel_lane_view()
