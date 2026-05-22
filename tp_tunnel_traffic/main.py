from __future__ import annotations

import math
import msvcrt
import random
import time
from datetime import datetime

import importlib
import json
import os
from pathlib import Path
from typing import Any


def require_carla():
    # 动态导入以避免静态分析环境的路径误报（运行时模块在 repo 内）
    return importlib.import_module("tp_replay.carla_compat").require_carla()

from .config import TunnelTrafficConfig
from .autopilot import configure_autopilot, configure_traffic_manager
from .control import compute_control
from .vehicle_plan import build_vehicle_plan
from .collector import DatasetCollector
from .video_collector import VideoCollector
from .hololens_server import HoloLensServer
from .lane_sampling import build_center_lane_path, build_three_lane_paths
from .spawning import destroy_spawned_actors, try_spawn_vehicle
from .gui import TunnelTrafficGUI


def _write_run_actors_json(output_root: Path, capture_vehicle, capture_proxy_state) -> None:
    if output_root is None:
        return
    try:
        actor_id = int(getattr(capture_vehicle, "id", -1)) if capture_vehicle is not None else -1
        bp_id = ""
        if capture_vehicle is not None:
            try:
                bp_id = str(capture_vehicle.type_id)
            except Exception:
                bp_id = ""

        lane_id = -1
        if capture_proxy_state is not None:
            try:
                lane_id = int(capture_proxy_state.get("lane_id", -1))
            except Exception:
                lane_id = -1

        data = {
            "target": {
                "actor_id": actor_id,
                "blueprint_id": bp_id,
                "lane_id": lane_id,
            }
        }
        with open(Path(output_root) / "actors.json", "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception:
        return


def _follow_driver_view(world, vehicle):
    try:
        carla = require_carla()
        spectator = world.get_spectator()
        tf = vehicle.get_transform()
        forward = tf.get_forward_vector()
        right = tf.get_right_vector()
        cam_forward_m, cam_right_m, cam_up_m, cam_pitch_deg = _load_camera_tune(
            os.getenv("TT_CAMERA_TUNE_JSON", r"tp_tunnel_traffic\camera_offsets.json")
        )
        cam_yaw_offset_deg = float(getattr(_follow_driver_view, "yaw_offset_deg", 0.0))

        target_loc = carla.Location(
            tf.location.x + forward.x * cam_forward_m + right.x * cam_right_m,
            tf.location.y + forward.y * cam_forward_m + right.y * cam_right_m,
            tf.location.z + cam_up_m,
        )
        target_rot = carla.Rotation(pitch=cam_pitch_deg, yaw=tf.rotation.yaw + cam_yaw_offset_deg, roll=0.0)
        spectator.set_transform(carla.Transform(target_loc, target_rot))
    except Exception:
        pass


def _look_at_point_view(world, target_loc):
    try:
        carla = require_carla()
        spectator = world.get_spectator()
        eye = carla.Location(target_loc.x - 8.0, target_loc.y - 3.0, target_loc.z + 4.0)
        yaw = math.degrees(math.atan2(target_loc.y - eye.y, target_loc.x - eye.x))
        pitch = -12.0
        spectator.set_transform(carla.Transform(eye, carla.Rotation(pitch=pitch, yaw=yaw, roll=0.0)))
    except Exception:
        pass


def _look_at_entry_view(world, start_tf, next_tf, yaw_offset_deg: float = 0.0):
    try:
        carla = require_carla()
        spectator = world.get_spectator()
        heading = math.atan2(next_tf.location.y - start_tf.location.y, next_tf.location.x - start_tf.location.x)
        eye = carla.Location(
            start_tf.location.x - math.cos(heading) * 16.0,
            start_tf.location.y - math.sin(heading) * 16.0,
            start_tf.location.z + 10.0,
        )
        yaw = math.degrees(heading) + float(yaw_offset_deg)
        pitch = -18.0
        spectator.set_transform(carla.Transform(eye, carla.Rotation(pitch=pitch, yaw=yaw, roll=0.0)))
    except Exception:
        pass


def _load_camera_tune(camera_tune_json_path: str):
    cam_forward_m = 0.65
    cam_right_m = -0.18
    cam_up_m = 1.22
    cam_pitch_deg = -7.5
    try:
        if camera_tune_json_path and os.path.exists(camera_tune_json_path):
            with open(camera_tune_json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            cam_forward_m = float(data.get("cam_forward_m", cam_forward_m))
            cam_right_m = float(data.get("cam_right_m", cam_right_m))
            cam_up_m = float(data.get("cam_up_m", cam_up_m))
            cam_pitch_deg = float(data.get("cam_pitch_deg", cam_pitch_deg))
    except Exception:
        pass
    return cam_forward_m, cam_right_m, cam_up_m, cam_pitch_deg


def _coerce_transform(obj: Any):
    """Coerce various CARLA-ish objects into a carla.Transform.

    Notes:
    - carla.Transform itself has a method named `transform(...)`, so we must not
      treat the presence of `.transform` as proof that `obj` is a wrapper.
    - PathPoint (our dataclass) exposes `.transform` as the actual transform.
    - Some CARLA objects may expose `transform` as a callable; handle both.
    """
    if obj is None:
        return None
    if hasattr(obj, "location") and hasattr(obj, "rotation"):
        return obj
    if hasattr(obj, "transform"):
        t = getattr(obj, "transform")
        # PathPoint.transform is a value; carla.Transform.transform is a method.
        if not callable(t):
            return t
    if hasattr(obj, "get_transform"):
        try:
            return obj.get_transform()
        except Exception:
            return None
    return None


def _set_overview_view(world, target_loc, yaw: float = 0.0):
    try:
        carla = require_carla()
        spectator = world.get_spectator()
        eye = carla.Location(target_loc.x, target_loc.y, target_loc.z + 22.0)
        spectator.set_transform(carla.Transform(eye, carla.Rotation(pitch=-70.0, yaw=yaw, roll=0.0)))
    except Exception:
        pass


def _compute_orbit_transform(carla, target_loc, yaw_deg: float, pitch_deg: float, distance_m: float, up_m: float = 0.0):
    # Orbit camera around a target location. yaw rotates around +Z, pitch tilts up/down.
    # CARLA pitch: positive looks up; we keep typical orbit pitch in [-80, -5].
    yaw_rad = math.radians(float(yaw_deg))
    pitch_rad = math.radians(float(pitch_deg))
    d = max(1.0, float(distance_m))
    # Local forward from camera to target
    fx = math.cos(pitch_rad) * math.cos(yaw_rad)
    fy = math.cos(pitch_rad) * math.sin(yaw_rad)
    fz = math.sin(pitch_rad)
    eye = carla.Location(
        target_loc.x - fx * d,
        target_loc.y - fy * d,
        target_loc.z - fz * d + float(up_m),
    )
    rot = carla.Rotation(pitch=float(pitch_deg), yaw=float(yaw_deg), roll=0.0)
    return carla.Transform(eye, rot)


def _update_camera_yaw_hotkeys() -> bool:
    changed = False
    preset = getattr(_update_camera_yaw_hotkeys, "preset", 0)
    yaw = float(getattr(_follow_driver_view, "yaw_offset_deg", 0.0))
    view_mode = getattr(_update_camera_yaw_hotkeys, "view_mode", "ego")
    presets = [-90.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 90.0]

    while msvcrt.kbhit():
        try:
            key = msvcrt.getch().decode("utf-8", errors="ignore")
        except Exception:
            continue

        if key == "[":
            preset = (preset - 1) % len(presets)
            yaw = presets[preset]
            changed = True
        elif key == "]":
            preset = (preset + 1) % len(presets)
            yaw = presets[preset]
            changed = True
        elif key == "0":
            preset = presets.index(0.0)
            yaw = 0.0
            changed = True
        elif key == ",":
            yaw -= 1.0
            changed = True
        elif key == ".":
            yaw += 1.0
            changed = True
        elif key.lower() == "v":
            view_mode = "overview" if view_mode == "ego" else "ego"
            changed = True

    setattr(_update_camera_yaw_hotkeys, "preset", preset)
    setattr(_update_camera_yaw_hotkeys, "view_mode", view_mode)
    setattr(_follow_driver_view, "yaw_offset_deg", yaw)
    return changed


def _nearest_idx_with_window(lane_points, loc, nearest_idx: int, window: int = 80):
    lo = max(0, nearest_idx - 10)
    hi = min(len(lane_points) - 1, nearest_idx + window)
    best_i = nearest_idx
    best_d2 = None
    for i in range(lo, hi + 1):
        wp = lane_points[i]
        dx = wp.transform.location.x - loc.x
        dy = wp.transform.location.y - loc.y
        dz = wp.transform.location.z - loc.z
        d2 = dx * dx + dy * dy + dz * dz
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def _build_proxy_state(world, vehicle, lane_id: int, waypoint_index: int, lane_points, target_speed_mps: float):
    return {
        "vehicle": vehicle,
        "actor_id": int(vehicle.id),
        "display_name": f"proxy#{int(vehicle.id)}",
        "lane_id": lane_id,
        "waypoint_index": waypoint_index,
        "lane_points": lane_points,
        "nearest_idx": waypoint_index,
        "target_speed_mps": target_speed_mps,
        "follow_distance_m": 10.0,
        "last_control": None,
        "active": True,
    }


def _lane_counts_by_index(plans):
    counts = {}
    for plan in plans:
        counts[plan.lane_index] = counts.get(plan.lane_index, 0) + 1
    return counts


def _resolve_capture_target(proxy_states, ego_vehicle, selected_proxy_actor_id):
    if selected_proxy_actor_id is None:
        return ego_vehicle, None
    for p in proxy_states:
        vehicle = p.get("vehicle")
        if vehicle is not None and vehicle.is_alive and p.get("actor_id") == selected_proxy_actor_id:
            return vehicle, p
    return ego_vehicle, None


def _build_capture_state_provider(
    config,
    proxy_states,
    selected_proxy_actor_id,
    view_mode,
    yaw_offset,
    collect_proxy_enabled: bool,
    video_recording: bool,
    hololens_active: bool,
    overview_state,
):
    return {
        "view_mode": view_mode,
        "yaw_offset_deg": float(yaw_offset),
        "selected_proxy_actor_id": selected_proxy_actor_id,
        "proxies": _collect_proxy_state_snapshot(proxy_states),
        "collect_output_by_target": bool(getattr(config, "collect_output_by_target", True)),
        "collect_proxy_enabled": bool(collect_proxy_enabled),
        "video_recording": bool(video_recording),
        "hololens_active": bool(hololens_active),
        "overview_state": dict(overview_state or {}),
    }


def _target_output_path(config, target_actor_id, target_kind: str) -> Path:
    base = Path(config.collect_output_dir)
    if not getattr(config, "collect_output_by_target", True):
        return base
    if target_kind == "ego":
        return base / "ego"
    if target_kind == "proxy":
        if target_actor_id is None:
            return base / "proxy_unselected"
        return base / f"proxy_{int(target_actor_id)}"
    suffix = f"{target_kind}_{int(target_actor_id)}" if target_actor_id is not None else f"{target_kind}"
    return base / suffix


def _collect_proxy_state_snapshot(proxy_states):
    out = []
    for p in proxy_states:
        vehicle = p.get("vehicle")
        if vehicle is None:
            continue
        out.append({
            "actor_id": int(p.get("actor_id", getattr(vehicle, "id", -1))),
            "display_name": str(p.get("display_name", f"proxy#{getattr(vehicle, 'id', -1)}")),
            "lane_id": int(p.get("lane_id", -1)),
            "waypoint_index": int(p.get("waypoint_index", 0)),
            "alive": bool(vehicle.is_alive),
        })
    return out


def _is_near_entry(loc, entry_loc, radius_m: float) -> bool:
    dx = loc.x - entry_loc.x
    dy = loc.y - entry_loc.y
    dz = loc.z - entry_loc.z
    return dx * dx + dy * dy + dz * dz <= float(radius_m) * float(radius_m)


def _draw_debug_points(world, lane_points, limit: int | None = None):
    try:
        debug = world.debug
        carla = require_carla()
        pts = lane_points if limit is None or limit <= 0 else lane_points[:limit]
        for i, pt in enumerate(pts):
            loc = pt.transform.location
            color = carla.Color(0, 255, 0) if i == 0 else carla.Color(255, 0, 0) if i == len(pts) - 1 else carla.Color(0, 128, 255)
            debug.draw_point(loc, size=0.12, color=color, life_time=300.0)
            if i > 0:
                prev = pts[i - 1].transform.location
                debug.draw_line(prev, loc, thickness=0.04, color=carla.Color(255, 255, 0), life_time=300.0)
    except Exception:
        pass


def main():
    carla = require_carla()
    config = TunnelTrafficConfig()
    rng = random.Random(config.seed)
    setattr(_update_camera_yaw_hotkeys, "view_mode", getattr(config, "camera_mode", "ego"))

    client = carla.Client(config.carla_host, config.carla_port)
    client.set_timeout(config.carla_timeout)
    world = client.get_world()

    gui = None
    original_settings = None
    spawned = []
    collector = None
    video_collector = None
    hololens_server = None
    try:
        original_settings = world.get_settings()
        if config.sync_mode:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(config.fixed_delta_seconds)
            world.apply_settings(settings)

        if config.clear_existing_vehicles:
            for actor in world.get_actors().filter("vehicle.*"):
                try:
                    actor.destroy()
                except Exception:
                    pass

        lanes = build_three_lane_paths(carla, xodr_path=config.xodr_path, step_m=config.step_m, spawn_z=1.0)
        lane_points = lanes["-2"]

        print(f"车道采样: 左={len(lanes['-1'])}  中={len(lane_points)}  右={len(lanes['-3'])}")

        if False and config.debug_draw:
            _draw_debug_points(world, lanes["-1"])
            _draw_debug_points(world, lanes["-2"])
            _draw_debug_points(world, lanes["-3"])

        bp_lib = world.get_blueprint_library()
        blueprints = list(bp_lib.filter(config.blueprint_filter))
        if config.blueprints_deny:
            deny = {x.strip() for x in config.blueprints_deny.split(",") if x.strip()}
            blueprints = [bp for bp in blueprints if bp.id not in deny]
        if not blueprints:
            raise RuntimeError("没有可用的车辆蓝图")

        blueprint = rng.choice(blueprints)
        if blueprint.has_attribute("color"):
            try:
                colors = list(blueprint.get_attribute("color").recommended_values)
                if colors:
                    blueprint.set_attribute("color", rng.choice(colors))
            except Exception:
                pass

        target_speed = max(6.0, float(getattr(config, "proxy_base_speed_mps", 12.0)))
        print(f"代理车基准速度: {target_speed:.2f}m/s ({target_speed * 3.6:.0f}km/h)")
        reserved_entry_m = 0.0
        proxy_states = []
        tm = None
        lane_lists = [lanes["-1"], lanes["-2"], lanes["-3"]]
        if config.proxy_enable:
            if config.proxy_use_tm:
                try:
                    tm = configure_traffic_manager(client, config)
                except Exception:
                    tm = None

            # 预留 ego 入口区域（米）：至少主车 clearance + follow_distance + 车长缓冲
            reserved_entry_m = max(
                0.0,
                float(config.spawn_clearance_m) + float(config.proxy_follow_distance_m) + 6.0,
            )
            # 让 build_vehicle_plan 使用该预留值（保持 helper 小且局部，不扩散到 config）
            try:
                setattr(config, "proxy_reserved_entry_m", float(reserved_entry_m))
            except Exception:
                pass

            # 先用 vehicle_plan 生成代理车计划，再按计划生成代理车
            plans = build_vehicle_plan(world, lane_lists, config, rng)
            lane_counts = _lane_counts_by_index(plans)
            print(
                f"代理车计划: {len(plans)} 辆, "
                f"左/中/右 = {lane_counts.get(0, 0)}/{lane_counts.get(1, 0)}/{lane_counts.get(2, 0)}"
            )

            spawn_ok = 0
            spawn_fail = 0
            spawn_skip = 0
            for plan in plans:
                lane_index = plan.lane_index
                lane_wps = lane_lists[lane_index]
                if not lane_wps:
                    spawn_skip += 1
                    continue
                wp_idx = min(max(0, plan.waypoint_index), len(lane_wps) - 1)
                start_wp = lane_wps[wp_idx].transform
                next_idx = min(wp_idx + 1, len(lane_wps) - 1)
                next_wp = lane_wps[next_idx].transform
                spawn_yaw = math.degrees(math.atan2(
                    next_wp.location.y - start_wp.location.y,
                    next_wp.location.x - start_wp.location.x,
                ))
                proxy_bp = bp_lib.find(plan.blueprint_id)
                if plan.color and proxy_bp.has_attribute("color"):
                    try:
                        proxy_bp.set_attribute("color", plan.color)
                    except Exception:
                        pass
                proxy_tf = carla.Transform(
                    carla.Location(start_wp.location.x, start_wp.location.y, 1.2),
                    carla.Rotation(pitch=0.0, yaw=spawn_yaw, roll=0.0),
                )

                # 额外入口净空：避免 warmup 后主车生成点附近被占
                if _is_near_entry(proxy_tf.location, lane_lists[1][0].transform.location, reserved_entry_m):
                    spawn_skip += 1
                    continue

                proxy_vehicle = try_spawn_vehicle(
                    world,
                    proxy_bp,
                    proxy_tf,
                    config.proxy_spawn_clearance_m,
                    spawned,
                    lane_aware_clearance=bool(getattr(config, "proxy_spawn_lane_aware", False)),
                    same_lane_lateral_m=float(getattr(config, "proxy_spawn_same_lane_lateral_m", 1.6)),
                    other_lane_clearance_m=float(getattr(config, "proxy_spawn_other_lane_clearance_m", 2.5)),
                    other_lane_lateral_m=float(getattr(config, "proxy_spawn_other_lane_lateral_m", 6.0)),
                )
                if proxy_vehicle is None:
                    spawn_fail += 1
                    continue
                spawned.append(proxy_vehicle)
                spawn_ok += 1
                proxy_vehicle.set_simulate_physics(False)
                world.tick()
                proxy_vehicle.set_simulate_physics(True)
                if tm is not None:
                    configure_autopilot(proxy_vehicle, tm, config)
                    # 代理车流优先使用 TT_PROXY_* 密度/跟车参数
                    try:
                        tm.distance_to_leading_vehicle(proxy_vehicle, float(config.proxy_follow_distance_m))
                        tm.vehicle_percentage_speed_difference(proxy_vehicle, float(config.proxy_speed_diff_percent))
                    except Exception:
                        pass
                proxy_states.append(
                    _build_proxy_state(
                        world,
                        proxy_vehicle,
                        lane_index,
                        wp_idx,
                        lane_wps,
                        target_speed * (1.0 - float(plan.speed_diff_percent) / 100.0),
                    )
                )

            # 入口区域附近如果仍有代理车生成成功（比如已有世界车辆或 clearance 不足），先清理掉
            if proxy_states:
                kept = []
                for p in proxy_states:
                    try:
                        if not p["vehicle"].is_alive:
                            p["active"] = False
                            continue
                        if _is_near_entry(p["vehicle"].get_location(), lane_lists[1][0].transform.location, reserved_entry_m):
                            p["vehicle"].destroy()
                            p["active"] = False
                            continue
                    except Exception:
                        # 出错就保守地丢弃
                        try:
                            if p.get("vehicle") is not None and p["vehicle"].is_alive:
                                p["vehicle"].destroy()
                        except Exception:
                            pass
                        p["active"] = False
                        continue
                    kept.append(p)
                proxy_states = kept
                print(f"代理车: spawn {spawn_ok} OK, {spawn_skip} skip, {spawn_fail} fail  (计划{len(plans)}辆)")

        # 先让代理车预热一段时间，营造更稳定的交通流环境
        warmup_deadline = time.time() + max(0.0, float(config.proxy_warmup_seconds))
        while config.proxy_enable and time.time() < warmup_deadline:
            if tm is None and proxy_states:
                # warmup 也按车道分组 + 前车距离约束，避免扎堆
                lane_groups = {}
                for p in proxy_states:
                    if not p.get("active") or p.get("vehicle") is None or not p["vehicle"].is_alive:
                        p["active"] = False
                        continue
                    lane_groups.setdefault(p["lane_id"], []).append(p)

                # 先更新 nearest_idx，再排序，保证 leader/spacing 稳定
                for group in lane_groups.values():
                    for p in group:
                        try:
                            p_loc = p["vehicle"].get_location()
                            p["nearest_idx"] = _nearest_idx_with_window(p["lane_points"], p_loc, p["nearest_idx"], window=60)
                        except Exception:
                            continue
                    group.sort(key=lambda x: x["nearest_idx"])

                for group in lane_groups.values():
                    for i, p in enumerate(group):
                        pv = p["vehicle"]
                        try:
                            p_loc = pv.get_location()
                        except Exception:
                            continue

                        front_gap_m = None
                        front_speed_mps = None
                        if i + 1 < len(group):
                            leader = group[i + 1]["vehicle"]
                            try:
                                front_gap_m = p_loc.distance(leader.get_location())
                                vel = leader.get_velocity()
                                front_speed_mps = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
                            except Exception:
                                front_gap_m = None

                        p_control = compute_control(
                            carla,
                            pv,
                            p["lane_points"][p["nearest_idx"]],
                            target_speed_mps=float(p["target_speed_mps"]),
                            follow_distance_m=float(config.proxy_follow_distance_m),
                            front_gap_m=front_gap_m,
                            front_speed_mps=front_speed_mps,
                            path_points=p["lane_points"],
                            nearest_idx=p["nearest_idx"],
                            lookahead_m=float(config.lookahead_m),
                            steer_lpf_alpha=float(config.steer_lpf_alpha),
                            steer_max_rate=float(config.steer_max_rate),
                            dt_seconds=float(config.fixed_delta_seconds) if config.sync_mode else None,
                            idm_a=float(config.idm_max_accel),
                            idm_b=float(config.idm_comfort_decel),
                            idm_s0=float(config.idm_min_gap),
                            idm_T=float(config.idm_time_headway),
                            idm_delta=float(config.idm_delta),
                        )
                        pv.apply_control(p_control)
                        p["last_control"] = p_control
            else:
                # Traffic Manager 模式下：autopilot 已接管，只需要 tick
                pass
            world.tick()

        # No ego capture vehicle anymore. Proxy vehicles are the only scene targets.
        selected_proxy_actor_id = None
        capture_vehicle = None
        capture_proxy_state = None
        collect_proxy_enabled = False
        video_recording = False
        hololens_active = False
        proxy_first_person = True
        start_tf = lane_points[0].transform
        next_tf = lane_points[1].transform
        overview_target_loc = start_tf.location

        # Overview free-fly camera state (interactive)
        overview_pos = [start_tf.location.x - 22.0, start_tf.location.y, start_tf.location.z + 12.0]
        overview_yaw_deg = 180.0
        overview_pitch_deg = -18.0

        def _reset_overview_camera() -> None:
            nonlocal overview_pos, overview_yaw_deg, overview_pitch_deg
            overview_pos = [start_tf.location.x - 22.0, start_tf.location.y, start_tf.location.z + 12.0]
            try:
                heading = math.degrees(math.atan2(
                    next_tf.location.y - start_tf.location.y,
                    next_tf.location.x - start_tf.location.x,
                ))
            except Exception:
                heading = 180.0
            overview_yaw_deg = heading
            overview_pitch_deg = -18.0
        gui = TunnelTrafficGUI(world, None, config)
        if gui is not None and gui.enabled:
            gui.update_state_provider(lambda: _build_capture_state_provider(
                config,
                proxy_states,
                selected_proxy_actor_id,
                getattr(_update_camera_yaw_hotkeys, "view_mode", getattr(config, "camera_mode", "ego")),
                float(getattr(_follow_driver_view, "yaw_offset_deg", 0.0)),
                bool(collect_proxy_enabled),
                bool(video_recording),
                bool(hololens_active),
                {
                    "yaw_deg": overview_yaw_deg,
                    "pitch_deg": overview_pitch_deg,
                    "pos": tuple(overview_pos),
                    "mode": "freefly",
                },
            ))

        _look_at_entry_view(world, start_tf, next_tf)
        _reset_overview_camera()

        collector = DatasetCollector(carla, world, None, config, lane_points)
        # Start disabled; GUI button arms collection for the selected proxy.
        try:
            collector.set_enabled(False)
            collector.set_vehicle(None, lane_points=lane_points, output_root=_target_output_path(config, None, "proxy"))
        except Exception:
            collector.spawn_sensors()
        print(
            f"采集初始化: enabled=OFF, output_base={Path(config.collect_output_dir)}, "
            f"by_target={bool(getattr(config, 'collect_output_by_target', True))}"
        )

        # Video collector (independent from dataset collector)
        video_collector = VideoCollector(carla, world, config, lane_points) if config.video_enable else None
        if video_collector is not None:
            print(f"视频录制初始化: enabled=OFF, output_base={Path(config.video_output_dir)}")

        # HoloLens server (WebRTC streaming)
        hololens_server = HoloLensServer(carla, world, config) if config.hololens_enable else None
        if hololens_server is not None:
            print(f"HoloLens 推流初始化: port={config.hololens_port}")

        def _spawn_proxy_from_plan(plan) -> bool:
            lane_index = int(plan.lane_index)
            if lane_index < 0 or lane_index >= len(lane_lists):
                return False
            lane_wps = lane_lists[lane_index]
            if not lane_wps:
                return False
            wp_idx = min(max(0, int(plan.waypoint_index)), len(lane_wps) - 1)
            start_wp = lane_wps[wp_idx].transform
            next_idx = min(wp_idx + 1, len(lane_wps) - 1)
            next_wp = lane_wps[next_idx].transform
            spawn_yaw = math.degrees(math.atan2(
                next_wp.location.y - start_wp.location.y,
                next_wp.location.x - start_wp.location.x,
            ))
            proxy_bp = bp_lib.find(plan.blueprint_id)
            if plan.color and proxy_bp.has_attribute("color"):
                try:
                    proxy_bp.set_attribute("color", plan.color)
                except Exception:
                    pass
            proxy_tf = carla.Transform(
                carla.Location(start_wp.location.x, start_wp.location.y, 1.2),
                carla.Rotation(pitch=0.0, yaw=spawn_yaw, roll=0.0),
            )
            if _is_near_entry(proxy_tf.location, lane_lists[1][0].transform.location, reserved_entry_m):
                return False
            proxy_vehicle = try_spawn_vehicle(
                world,
                proxy_bp,
                proxy_tf,
                config.proxy_spawn_clearance_m,
                spawned,
                lane_aware_clearance=bool(getattr(config, "proxy_spawn_lane_aware", False)),
                same_lane_clearance_m=float(config.proxy_spawn_clearance_m),
                same_lane_lateral_m=float(getattr(config, "proxy_spawn_same_lane_lateral_m", 1.6)),
                other_lane_clearance_m=float(getattr(config, "proxy_spawn_other_lane_clearance_m", 2.5)),
                other_lane_lateral_m=float(getattr(config, "proxy_spawn_other_lane_lateral_m", 6.0)),
            )
            if proxy_vehicle is None:
                return False
            spawned.append(proxy_vehicle)
            proxy_vehicle.set_simulate_physics(False)
            world.tick()
            proxy_vehicle.set_simulate_physics(True)
            if tm is not None:
                configure_autopilot(proxy_vehicle, tm, config)
                try:
                    tm.distance_to_leading_vehicle(proxy_vehicle, float(config.proxy_follow_distance_m))
                    tm.vehicle_percentage_speed_difference(proxy_vehicle, float(config.proxy_speed_diff_percent))
                except Exception:
                    pass
            proxy_states.append(
                _build_proxy_state(
                    world,
                    proxy_vehicle,
                    lane_index,
                    wp_idx,
                    lane_wps,
                    target_speed * (1.0 - float(plan.speed_diff_percent) / 100.0),
                )
            )
            print(
                f"代理车生成成功: lane={lane_index} wp={wp_idx} blueprint={plan.blueprint_id} "
                f"color={plan.color or 'default'} speed_diff={plan.speed_diff_percent:.1f}%"
            )
            return True

        dt_seconds = float(config.fixed_delta_seconds) if config.sync_mode else None
        debug_ticks = 0
        collect_tick_idx = 0
        target_lane_count = int(getattr(config, "proxy_target_per_lane", 0))
        if target_lane_count <= 0:
            target_lane_count = max(5, int(config.proxy_min_per_lane) + 1)
        next_replenish_time = time.time() + 2.0
        next_status_log_time = time.time() + 1.0
        next_detail_log_time = time.time() + max(1.0, float(getattr(config, "proxy_detail_log_interval_s", 5.0)))
        while True:

            # 代理车辆：同车道前车约束前进
            if proxy_states:
                lane_groups = {}
                for p in proxy_states:
                    if not p["active"] or not p["vehicle"].is_alive:
                        p["active"] = False
                        continue
                    lane_groups.setdefault(p["lane_id"], []).append(p)

                for lane_id, group in lane_groups.items():
                    for p in group:
                        try:
                            pv = p["vehicle"]
                            p_loc = pv.get_location()
                            p["nearest_idx"] = _nearest_idx_with_window(p["lane_points"], p_loc, p["nearest_idx"], window=60)
                        except Exception:
                            continue

                    group.sort(key=lambda x: x["nearest_idx"])
                    for i, p in enumerate(group):
                        pv = p["vehicle"]
                        p_loc = pv.get_location()

                        front_gap_m = None
                        if i + 1 < len(group):
                            leader = group[i + 1]["vehicle"]
                            try:
                                front_gap_m = p_loc.distance(leader.get_location())
                            except Exception:
                                front_gap_m = None

                        p_control = compute_control(
                            carla,
                            pv,
                            p["lane_points"][p["nearest_idx"]],
                            target_speed_mps=float(p["target_speed_mps"]),
                            follow_distance_m=float(config.proxy_follow_distance_m),
                            front_gap_m=front_gap_m,
                            path_points=p["lane_points"],
                            nearest_idx=p["nearest_idx"],
                            lookahead_m=float(config.lookahead_m),
                            steer_lpf_alpha=float(config.steer_lpf_alpha),
                            steer_max_rate=float(config.steer_max_rate),
                            dt_seconds=dt_seconds,
                        )
                        pv.apply_control(p_control)
                # 清理已驶出隧道末端的车辆，保持车流持续补充
                kept_states = []
                for p in proxy_states:
                    try:
                        if not p.get("active") or p.get("vehicle") is None or not p["vehicle"].is_alive:
                            p["active"] = False
                            continue
                        if int(p.get("nearest_idx", 0)) >= len(p.get("lane_points", lane_points)) - 5:
                            try:
                                p["vehicle"].destroy()
                            except Exception:
                                pass
                            p["active"] = False
                            continue
                        kept_states.append(p)
                    except Exception:
                        continue
                proxy_states = kept_states

            _update_camera_yaw_hotkeys()

            now = time.time()
            if now >= next_status_log_time:
                lane_counts = {}
                for p in proxy_states:
                    if p.get("active") and p.get("vehicle") is not None and p["vehicle"].is_alive:
                        lane_counts[p["lane_id"]] = lane_counts.get(p["lane_id"], 0) + 1
                total_active = sum(lane_counts.values())
                # In-place status bar (overwrite same line with \r)
                print(
                    f"\r代理车流: {total_active} 辆  L={lane_counts.get(0, 0)}  M={lane_counts.get(1, 0)}  R={lane_counts.get(2, 0)}  "
                    f"collect={'ON' if collect_proxy_enabled else 'OFF'}  ",
                    end="", flush=True,
                )
                next_status_log_time = now + 2.0

            if now >= next_detail_log_time and config.proxy_detail_log_interval_s > 0:
                active_states = [
                    p for p in proxy_states
                    if p.get("active") and p.get("vehicle") is not None and p["vehicle"].is_alive
                ]
                if active_states:
                    details = []
                    for p in sorted(active_states, key=lambda x: (int(x.get("lane_id", 99)), int(x.get("nearest_idx", 0)))):
                        v = p.get("vehicle")
                        if v is None:
                            continue
                        try:
                            vel = v.get_velocity()
                            speed_kmh = (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z) ** 0.5 * 3.6
                        except Exception:
                            speed_kmh = 0.0
                        details.append(
                            f"#{int(p.get('actor_id', -1))}@L{int(p.get('lane_id', -1))}"
                            f" idx={int(p.get('nearest_idx', 0))}"
                            f" v={speed_kmh:.1f}km/h"
                        )
                    print("代理车详情: " + " | ".join(details))
                next_detail_log_time = now + max(1.0, float(getattr(config, "proxy_detail_log_interval_s", 5.0)))

            if now >= next_replenish_time and config.proxy_enable:
                lane_counts = {}
                for p in proxy_states:
                    if p.get("active") and p.get("vehicle") is not None and p["vehicle"].is_alive:
                        lane_counts[p["lane_id"]] = lane_counts.get(p["lane_id"], 0) + 1
                # 临时放大计划数量，快速补缺口车道
                saved_min = getattr(config, "proxy_min_per_lane")
                saved_max = getattr(config, "proxy_max_per_lane")
                saved_start_ratio = getattr(config, "proxy_spawn_start_ratio")
                try:
                    setattr(config, "proxy_min_per_lane", 3)
                    setattr(config, "proxy_max_per_lane", 3)
                    setattr(config, "proxy_spawn_start_ratio", min(float(saved_start_ratio), 0.08))
                    replenish_plans = build_vehicle_plan(world, lane_lists, config, rng)
                    for plan in replenish_plans:
                        lane_idx = int(plan.lane_index)
                        if lane_counts.get(lane_idx, 0) >= target_lane_count:
                            continue
                        if _spawn_proxy_from_plan(plan):
                            lane_counts[lane_idx] = lane_counts.get(lane_idx, 0) + 1
                finally:
                    setattr(config, "proxy_min_per_lane", saved_min)
                    setattr(config, "proxy_max_per_lane", saved_max)
                    setattr(config, "proxy_spawn_start_ratio", saved_start_ratio)
                next_replenish_time = now + 1.0

            if gui is not None and gui.enabled:
                gui_action = gui.tick()
                if gui_action == "quit":
                    print("GUI closed, ending simulation")
                    break
                elif gui_action:
                    # Sync with hotkey logic
                    view_mode = getattr(_update_camera_yaw_hotkeys, "view_mode", "ego")
                    yaw = float(getattr(_follow_driver_view, "yaw_offset_deg", 0.0))
                    
                    if gui_action == "ego":
                        view_mode = "ego"
                    elif gui_action == "overview":
                        view_mode = "overview"
                    elif gui_action == "yaw_left":
                        yaw -= 1.0
                    elif gui_action == "yaw_right":
                        yaw += 1.0
                    elif gui_action == "yaw_reset":
                        yaw = 0.0
                    elif gui_action == "toggle_collect_target":
                        if selected_proxy_actor_id is not None:
                            # 互斥：开启采集时关闭视频录制
                            if collect_proxy_enabled and video_recording and video_collector is not None:
                                video_collector.stop()
                                video_recording = False
                                print("Record Video: OFF（因开启数据集采集自动停止）")
                            collect_proxy_enabled = not bool(collect_proxy_enabled)
                            target_dir = _target_output_path(config, selected_proxy_actor_id, "proxy")
                            run_dir = target_dir / datetime.now().strftime("run_%Y%m%d_%H%M%S") if collect_proxy_enabled else target_dir
                            print(
                                f"Collect Selected: {'ON' if collect_proxy_enabled else 'OFF'} "
                                f"target_proxy={selected_proxy_actor_id}, output_dir={run_dir if collect_proxy_enabled else target_dir}"
                            )
                            if collect_proxy_enabled and collector is not None and capture_vehicle is not None:
                                try:
                                    collector.set_vehicle(
                                        capture_vehicle,
                                        lane_points=(capture_proxy_state.get("lane_points", lane_points) if capture_proxy_state else lane_points),
                                        output_root=run_dir,
                                    )
                                    _write_run_actors_json(run_dir, capture_vehicle, capture_proxy_state)
                                    print(f"采集目标已绑定: actor_id={selected_proxy_actor_id}")
                                    print(
                                        f"采集状态: 持续采集已启动 stride={int(config.collect_frame_stride)} "
                                        f"target_actor={selected_proxy_actor_id}"
                                    )
                                except Exception:
                                    pass
                            if collector is not None:
                                try:
                                    collector.set_enabled(bool(collect_proxy_enabled) and capture_proxy_state is not None)
                                except Exception:
                                    pass
                        else:
                            print("Collect Selected: 忽略（当前未选中代理车）")
                    elif gui_action == "toggle_video_record":
                        if video_collector is None:
                            print("Record Video: 未启用（TT_VIDEO_ENABLE=0）")
                        elif selected_proxy_actor_id is not None and capture_vehicle is not None:
                            # 互斥：开启视频录制时关闭采集
                            if not video_recording and collect_proxy_enabled and collector is not None:
                                collector.set_enabled(False)
                                collect_proxy_enabled = False
                                print("Collect Selected: OFF（因开启视频录制自动停止）")
                            video_recording = not video_recording
                            if video_recording:
                                video_collector.set_vehicle(
                                    capture_vehicle,
                                    lane_points=(capture_proxy_state.get("lane_points", lane_points) if capture_proxy_state else lane_points),
                                )
                                video_collector.start()
                                print(f"Record Video: ON, target_proxy={selected_proxy_actor_id}")
                            else:
                                video_collector.stop()
                                print(f"Record Video: OFF")
                        else:
                            print("Record Video: 忽略（当前未选中代理车）")
                    elif gui_action == "toggle_hololens":
                        if hololens_server is None:
                            print("HoloLens Stream: 未启用（TT_HOLOLENS_ENABLE=0）")
                        else:
                            hololens_active = not hololens_active
                            if hololens_active:
                                hololens_server.start(capture_vehicle)
                                if capture_vehicle is not None:
                                    print(f"HoloLens Stream: ON, target={selected_proxy_actor_id}, port={config.hololens_port}")
                                else:
                                    print(f"HoloLens Stream: ON, no target (spectator view), port={config.hololens_port}")
                            else:
                                hololens_server.stop()
                                print("HoloLens Stream: OFF")
                    elif gui_action == "overview_reset":
                        _reset_overview_camera()
                    elif "overview_" in gui_action:
                        # GUI may batch multiple overview actions with ';'
                        for sub_action in str(gui_action).split(";"):
                            if sub_action == "overview_reset":
                                _reset_overview_camera()
                            elif sub_action.startswith("overview_move:"):
                                try:
                                    payload = sub_action.split(":", 1)[1]
                                    dx_s, dy_s, dz_s = payload.split(",", 2)
                                    dx = float(dx_s)
                                    dy = float(dy_s)
                                    dz = float(dz_s)
                                    # Move in camera-local coordinates: W/S forward, A/D strafe, Q/E vertical
                                    yaw_rad = math.radians(float(overview_yaw_deg))
                                    forward_x = math.cos(yaw_rad)
                                    forward_y = math.sin(yaw_rad)
                                    right_x = math.cos(yaw_rad + math.pi / 2.0)
                                    right_y = math.sin(yaw_rad + math.pi / 2.0)
                                    move_scale = 1.5
                                    overview_pos[0] += (forward_x * dy + right_x * dx) * move_scale
                                    overview_pos[1] += (forward_y * dy + right_y * dx) * move_scale
                                    overview_pos[2] += dz * move_scale
                                except Exception:
                                    pass
                            elif sub_action.startswith("overview_look:"):
                                try:
                                    payload = sub_action.split(":", 1)[1]
                                    dx_s, dy_s = payload.split(",", 1)
                                    dx = float(dx_s)
                                    dy = float(dy_s)
                                    sens = 0.18
                                    overview_yaw_deg += dx * sens
                                    overview_pitch_deg -= dy * sens
                                    overview_pitch_deg = max(-80.0, min(20.0, float(overview_pitch_deg)))
                                except Exception:
                                    pass
                            elif sub_action.startswith("overview_zoom:"):
                                try:
                                    dz = float(sub_action.split(":", 1)[1])
                                    # Keep wheel as additional distance adjustment.
                                    overview_pos[0] -= math.cos(math.radians(overview_yaw_deg)) * dz * 2.0
                                    overview_pos[1] -= math.sin(math.radians(overview_yaw_deg)) * dz * 2.0
                                except Exception:
                                    pass
                    elif gui_action.startswith("select_proxy:"):
                        try:
                            selected_proxy_actor_id = int(gui_action.split(":", 1)[1])
                        except Exception:
                            selected_proxy_actor_id = None
                        if selected_proxy_actor_id is not None:
                            resolved_vehicle, resolved_state = _resolve_capture_target(proxy_states, None, selected_proxy_actor_id)
                            if resolved_state is None:
                                selected_proxy_actor_id = None
                                view_mode = "ego"
                                capture_vehicle = None
                                capture_proxy_state = None
                                collect_proxy_enabled = False
                            else:
                                view_mode = "ego"
                                capture_vehicle = resolved_vehicle
                                capture_proxy_state = resolved_state
                                proxy_first_person = True
                                if collect_proxy_enabled:
                                    collect_proxy_enabled = False
                                    if collector is not None:
                                        try:
                                            collector.set_enabled(False)
                                        except Exception:
                                            pass
                                    print("Collect Selected: OFF（采集中切换目标，为避免串数据已自动停止）")
                                # 视频录制跟随新目标
                                if video_recording and video_collector is not None:
                                    video_collector.set_vehicle(
                                        capture_vehicle,
                                        lane_points=(capture_proxy_state.get("lane_points", lane_points) if capture_proxy_state else lane_points),
                                    )
                                # HoloLens 推流跟随新目标
                                if hololens_active and hololens_server is not None:
                                    hololens_server.set_vehicle(capture_vehicle)
                                if collector is not None:
                                    try:
                                        target_dir = _target_output_path(config, selected_proxy_actor_id, "proxy")
                                        print(
                                            f"代理车已选中: actor_id={selected_proxy_actor_id}, "
                                            f"collect_output={target_dir}"
                                        )
                                    except Exception:
                                        pass
                        else:
                            capture_vehicle = None
                            capture_proxy_state = None
                            collect_proxy_enabled = False
                            if collector is not None:
                                try:
                                    collector.set_vehicle(
                                        capture_vehicle,
                                        lane_points=lane_points,
                                    )
                                    print("代理车已取消选择: collect_target=none")
                                except Exception:
                                    pass

                        # Update collector enabled state for target separation/arming
                        if collector is not None:
                            try:
                                collector.set_enabled(bool(collect_proxy_enabled) and capture_proxy_state is not None)
                            except Exception:
                                pass
                    
                    setattr(_update_camera_yaw_hotkeys, "view_mode", view_mode)
                    setattr(_follow_driver_view, "yaw_offset_deg", yaw)

            if debug_ticks < 5:
                debug_ticks += 1

            # HoloLens camera: position BEFORE tick for zero-lag
            if hololens_active and hololens_server is not None:
                hololens_server.pre_tick()

            world.tick()
            collect_tick_idx += 1

            view_mode = getattr(_update_camera_yaw_hotkeys, "view_mode", getattr(config, "camera_mode", "ego"))
            current_tf = None
            if capture_vehicle is not None and not capture_vehicle.is_alive:
                capture_vehicle = None
                capture_proxy_state = None
                selected_proxy_actor_id = None
                if collect_proxy_enabled:
                    collect_proxy_enabled = False
                    if collector is not None:
                        try:
                            collector.set_enabled(False)
                        except Exception:
                            pass
                if video_recording and video_collector is not None:
                    video_collector.stop()
                    video_recording = False
                if hololens_active and hololens_server is not None:
                    # Vehicle died — let server handle it gracefully
                    # (old camera already destroyed by CARLA, new one spawned above)
                    pass
                print("Collect Selected: OFF（目标代理车已失效）")

            if view_mode == "overview":
                # Free-fly overview view: WASD/QE + mouse look.
                yaw_rad = math.radians(float(overview_yaw_deg))
                pitch_rad = math.radians(float(overview_pitch_deg))
                forward_x = math.cos(yaw_rad) * math.cos(pitch_rad)
                forward_y = math.sin(yaw_rad) * math.cos(pitch_rad)
                forward_z = math.sin(pitch_rad)
                current_tf = carla.Transform(
                    carla.Location(float(overview_pos[0]), float(overview_pos[1]), float(overview_pos[2])),
                    carla.Rotation(pitch=float(overview_pitch_deg), yaw=float(overview_yaw_deg), roll=0.0),
                )
                try:
                    world.get_spectator().set_transform(current_tf)
                except Exception:
                    pass
            else:
                if capture_vehicle is not None:
                    _follow_driver_view(world, capture_vehicle)
                    # GUI ego camera is handled by default transform attachments or can be explicitly updated
                    tf = capture_vehicle.get_transform()
                else:
                    tf = _coerce_transform(start_tf)

                if tf is None:
                    tf = world.get_spectator().get_transform()

                forward = tf.get_forward_vector()
                right = tf.get_right_vector()
                cam_yaw_offset_deg = float(getattr(_follow_driver_view, "yaw_offset_deg", 0.0))
                cam_forward_m, cam_right_m, cam_up_m, cam_pitch_deg = _load_camera_tune(
                    getattr(config, "camera_tune_json", r"tp_tunnel_traffic\camera_offsets.json")
                )
                target_loc = carla.Location(
                    tf.location.x + forward.x * cam_forward_m + right.x * cam_right_m,
                    tf.location.y + forward.y * cam_forward_m + right.y * cam_right_m,
                    tf.location.z + cam_up_m,
                )
                target_rot = carla.Rotation(pitch=cam_pitch_deg, yaw=tf.rotation.yaw + cam_yaw_offset_deg, roll=0.0)
                current_tf = carla.Transform(target_loc, target_rot)

            if gui is not None and gui.enabled and current_tf is not None:
                gui.update_transform(current_tf)

            if gui is not None and gui.enabled:
                gui.update_state_provider(lambda: _build_capture_state_provider(
                    config,
                    proxy_states,
                    selected_proxy_actor_id,
                    getattr(_update_camera_yaw_hotkeys, "view_mode", getattr(config, "camera_mode", "overview")),
                    float(getattr(_follow_driver_view, "yaw_offset_deg", 0.0)),
                    bool(collect_proxy_enabled),
                    bool(video_recording),
                    bool(hololens_active),
                    {
                        "yaw_deg": overview_yaw_deg,
                        "pitch_deg": overview_pitch_deg,
                        "pos": tuple(overview_pos),
                    },
                ))

            if collector is not None:
                collector.set_enabled(bool(collect_proxy_enabled) and capture_proxy_state is not None)
                if capture_proxy_state is not None and capture_vehicle is not None:
                    collector_nearest_idx = int(capture_proxy_state.get("nearest_idx", 0))
                    collector_control = capture_proxy_state.get("last_control")
                    if collector_control is None:
                        try:
                            collector_control = capture_vehicle.get_control()
                        except Exception:
                            collector_control = None
                    if collector_control is not None:
                        try:
                            world_frame = int(world.get_snapshot().frame)
                        except Exception:
                            world_frame = int(collect_tick_idx)
                        try:
                            collector.on_tick(world_frame, collector_nearest_idx, collector_control)
                        except Exception:
                            collect_proxy_enabled = False
                            try:
                                collector.set_enabled(False)
                            except Exception:
                                pass
                            print("Collect Selected: OFF（采集写盘异常，已自动停采）")

            if video_collector is not None and video_recording:
                try:
                    world_frame = int(world.get_snapshot().frame)
                except Exception:
                    world_frame = int(collect_tick_idx)
                try:
                    video_collector.on_tick(world_frame)
                except Exception:
                    video_collector.stop()
                    video_recording = False
                    print("Record Video: OFF（写盘异常，已自动停止）")

    except KeyboardInterrupt:
        pass
    finally:
        if gui is not None:
            gui.destroy()
        if collector is not None:
            collector.destroy()
        if video_collector is not None:
            video_collector.destroy()
        if hololens_server is not None:
            hololens_server.stop()
        destroy_spawned_actors(spawned)
        if original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass


if __name__ == "__main__":
    main()
