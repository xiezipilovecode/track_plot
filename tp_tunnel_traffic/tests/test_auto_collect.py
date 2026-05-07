from __future__ import annotations

"""Fully automated batch dataset collection for tunnel traffic.

Usage (CMD, CARLA server must be running):
    set TT_PROXY_ENABLE=1
    set TT_AUTO_COLLECT_ENABLE=1
    set TT_AUTO_COLLECT_SECONDS_PER_RUN=60
    set TT_AUTO_COLLECT_MAX_RUNS=20
    set TT_AUTO_COLLECT_MODE=balanced
    set TT_AUTO_COLLECT_TARGET_IMAGES_PER_CAMERA=0
    set TT_COLLECT_OUTPUT_DIR=dataset
    set TT_COLLECT_FRAME_STRIDE=10
    set TT_COLLECT_RECORD_LABELS=1
    set TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1
    set TT_COLLECT_WRITE_COCO=1
    set TT_COLLECT_COCO_MIN_AREA_PX2=200
    set TT_COLLECT_COCO_MAX_HEIGHT_RATIO=0.9
    python -m tp_tunnel_traffic.tests.test_auto_collect
"""

import json
import math
import random
import sys
import time
from datetime import datetime
from pathlib import Path

import importlib

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def _require_carla():
    return importlib.import_module("tp_replay.carla_compat").require_carla()


def _import_mod(name: str):
    return importlib.import_module(name)

# Lazy imports to keep LSP clean (resolved at runtime via -m)
_tunnel_cfg = _import_mod("tp_tunnel_traffic.config")
TunnelTrafficConfig = _tunnel_cfg.TunnelTrafficConfig
_lane = _import_mod("tp_tunnel_traffic.lane_sampling")
build_three_lane_paths = _lane.build_three_lane_paths
_vplan = _import_mod("tp_tunnel_traffic.vehicle_plan")
build_vehicle_plan = _vplan.build_vehicle_plan
_spawn = _import_mod("tp_tunnel_traffic.spawning")
destroy_spawned_actors = _spawn.destroy_spawned_actors
try_spawn_vehicle = _spawn.try_spawn_vehicle
_ctrl = _import_mod("tp_tunnel_traffic.control")
compute_control = _ctrl.compute_control
_auto = _import_mod("tp_tunnel_traffic.autopilot")
configure_autopilot = _auto.configure_autopilot
configure_traffic_manager = _auto.configure_traffic_manager
_coll = _import_mod("tp_tunnel_traffic.collector")
DatasetCollector = _coll.DatasetCollector


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


def _is_near_entry(loc, entry_loc, radius_m: float) -> bool:
    dx = loc.x - entry_loc.x
    dy = loc.y - entry_loc.y
    dz = loc.z - entry_loc.z
    return (dx * dx + dy * dy + dz * dz) <= (float(radius_m) * float(radius_m))


def _lane_counts_by_index(plans):
    counts: dict[int, int] = {}
    for plan in plans:
        counts[plan.lane_index] = counts.get(plan.lane_index, 0) + 1
    return counts


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
    suffix = f"{target_kind}_{int(target_actor_id)}" if target_actor_id is not None else target_kind
    return base / suffix


def _load_batch_state(batch_dir: Path) -> dict:
    state_path = batch_dir / "auto_collect_config.json"
    if state_path.exists():
        with open(state_path, "r", encoding="utf-8") as f:
            return json.load(f)
    return {
        "runs_completed": 0,
        "current_per_camera": {},
        "target_collections": {},
        "completed_runs": [],
    }


def _save_batch_state(batch_dir: Path, state: dict) -> None:
    state_path = batch_dir / "auto_collect_config.json"
    with open(state_path, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)


def _save_batch_summary(batch_dir: Path, state: dict, started_at: str) -> None:
    summary = {
        "started_at": started_at,
        "target_images_per_camera": state.get("target_images_per_camera", 0),
        "current_per_camera": state.get("current_per_camera", {}),
        "total_runs": state.get("runs_completed", 0),
        "target_collections": state.get("target_collections", {}),
        "completed_runs": state.get("completed_runs", []),
    }
    summary_path = batch_dir / "batch_summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(f"Batch summary written: {summary_path}")


def _count_images_in_run(run_dir: Path, cam_names: list[str]) -> dict[str, int]:
    counts: dict[str, int] = {}
    images_root = run_dir / "images"
    if not images_root.exists():
        return counts
    for cam in cam_names:
        cam_dir = images_root / cam
        if cam_dir.is_dir():
            counts[cam] = len(list(cam_dir.glob("*.png")))
    return counts


def _write_run_actors_json(output_root: Path, vehicle, proxy_state) -> None:
    if output_root is None:
        return
    try:
        actor_id = int(getattr(vehicle, "id", -1)) if vehicle is not None else -1
        bp_id = ""
        if vehicle is not None:
            try:
                bp_id = str(vehicle.type_id)
            except Exception:
                bp_id = ""
        lane_id = -1
        if proxy_state is not None:
            try:
                lane_id = int(proxy_state.get("lane_id", -1))
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


def _pick_next_target(
    proxy_states: list[dict],
    config,
    state: dict,
) -> tuple:
    """Select the next proxy vehicle to collect from."""
    mode = str(getattr(config, "auto_collect_mode", "balanced"))
    min_speed = float(getattr(config, "auto_collect_min_speed_mps", 5.0))
    target_collections = state.get("target_collections", {})

    # Filter to alive, fast-enough proxies
    alive = []
    for p in proxy_states:
        if not p.get("active"):
            continue
        v = p.get("vehicle")
        if v is None or not v.is_alive:
            continue
        try:
            vel = v.get_velocity()
            speed = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
        except Exception:
            speed = 0.0
        if speed < min_speed:
            continue
        alive.append((int(v.id), speed, p))

    if not alive:
        return None, None

    if mode == "random":
        _, _, chosen = random.choice(alive)
        return chosen, chosen["vehicle"]
    elif mode == "cycle":
        alive.sort(key=lambda x: x[0])
        _, _, chosen = alive[0]
        return chosen, chosen["vehicle"]
    else:  # balanced
        # Pick the one with the fewest completed runs
        alive.sort(key=lambda x: target_collections.get(str(x[0]), 0))
        _, _, chosen = alive[0]
        return chosen, chosen["vehicle"]


def main():
    carla = _require_carla()
    config = TunnelTrafficConfig()
    rng = random.Random(config.seed)

    if not bool(getattr(config, "auto_collect_enable", False)):
        print("TT_AUTO_COLLECT_ENABLE is not set. Exiting.")
        return

    batch_dir = Path(getattr(config, "auto_collect_output_batch_dir", "dataset"))
    batch_dir.mkdir(parents=True, exist_ok=True)
    started_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    state = _load_batch_state(batch_dir)
    if bool(getattr(config, "auto_collect_resume", True)):
        print(f"Resuming from previous state: {state.get('runs_completed', 0)} runs completed")
    else:
        state = {
            "runs_completed": 0,
            "current_per_camera": {},
            "target_collections": {},
            "completed_runs": [],
            "target_images_per_camera": int(getattr(config, "auto_collect_target_images_per_camera", 0)),
        }

    max_runs = int(getattr(config, "auto_collect_max_runs", 20))
    seconds_per_run = float(getattr(config, "auto_collect_seconds_per_run", 60.0))
    cooldown_s = float(getattr(config, "auto_collect_cooldown_s", 5.0))
    target_images = state.get("target_images_per_camera", 0)

    client = carla.Client(config.carla_host, config.carla_port)
    client.set_timeout(config.carla_timeout)
    world = client.get_world()

    gui = None  # no GUI in auto mode
    original_settings = None
    spawned = []
    collector = None

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
        print(f"中间车道采样点数量: {len(lane_points)}")

        bp_lib = world.get_blueprint_library()
        blueprints = list(bp_lib.filter(config.blueprint_filter))
        if config.blueprints_deny:
            deny = {x.strip() for x in config.blueprints_deny.split(",") if x.strip()}
            blueprints = [bp for bp in blueprints if bp.id not in deny]
        if not blueprints:
            raise RuntimeError("没有可用的车辆蓝图")

        target_speed = max(6.0, float(getattr(config, "proxy_base_speed_mps", 12.0)))
        print(
            f"代理车基准速度: {target_speed:.2f} m/s ({target_speed * 3.6:.1f} km/h), "
            f"follow_distance={float(config.proxy_follow_distance_m):.1f}m"
        )

        proxy_states: list[dict] = []
        tm = None
        lane_lists = [lanes["-1"], lanes["-2"], lanes["-3"]]

        if config.proxy_enable:
            if config.proxy_use_tm:
                try:
                    tm = configure_traffic_manager(client, config)
                except Exception:
                    tm = None

            reserved_entry_m = max(
                0.0,
                float(config.spawn_clearance_m) + float(config.proxy_follow_distance_m) + 6.0,
            )
            try:
                setattr(config, "proxy_reserved_entry_m", float(reserved_entry_m))
            except Exception:
                pass

            plans = build_vehicle_plan(world, lane_lists, config, rng)
            lane_counts = _lane_counts_by_index(plans)
            print(
                f"代理车计划总数: {len(plans)}, 左/中/右 = "
                f"{lane_counts.get(0, 0)}/{lane_counts.get(1, 0)}/{lane_counts.get(2, 0)}"
            )

            for plan in plans:
                lane_index = plan.lane_index
                lane_wps = lane_lists[lane_index]
                if not lane_wps:
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
                if _is_near_entry(proxy_tf.location, lane_lists[1][0].transform.location, reserved_entry_m):
                    continue
                proxy_vehicle = try_spawn_vehicle(
                    world, proxy_bp, proxy_tf, config.proxy_spawn_clearance_m, spawned,
                    lane_aware_clearance=bool(getattr(config, "proxy_spawn_lane_aware", False)),
                    same_lane_lateral_m=float(getattr(config, "proxy_spawn_same_lane_lateral_m", 1.6)),
                    other_lane_clearance_m=float(getattr(config, "proxy_spawn_other_lane_clearance_m", 2.5)),
                    other_lane_lateral_m=float(getattr(config, "proxy_spawn_other_lane_lateral_m", 6.0)),
                )
                if proxy_vehicle is None:
                    continue
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
                proxy_states.append(_build_proxy_state(
                    world, proxy_vehicle, lane_index, wp_idx, lane_wps,
                    target_speed * (1.0 - float(plan.speed_diff_percent) / 100.0),
                ))

            print(f"代理车生成完成: {len(proxy_states)}")

        # ---- Warmup ----
        warmup_deadline = time.time() + max(0.0, float(config.proxy_warmup_seconds))
        dt_seconds = float(config.fixed_delta_seconds) if config.sync_mode else None
        while config.proxy_enable and time.time() < warmup_deadline:
            if tm is None:
                _drive_proxies(proxy_states, config, dt_seconds, carla)
            world.tick()
        print("Warmup complete")

        # ---- Collector setup ----
        collector = DatasetCollector(carla, world, None, config, lane_points)
        collector.set_enabled(False)

        # ---- Main auto-collect loop ----
        target_lane_count = int(getattr(config, "proxy_target_per_lane", 0))
        if target_lane_count <= 0:
            target_lane_count = max(5, int(config.proxy_min_per_lane) + 1)

        next_replenish_time = time.time() + 2.0
        next_status_log_time = time.time() + 1.0

        auto_state = "IDLE"
        auto_state_since = time.time()
        current_capture_vehicle = None
        current_capture_state = None
        current_run_dir: Path | None = None
        current_actor_id: int = 0
        start_tick: int = 0

        while state["runs_completed"] < max_runs:
            # Check if target images reached
            if target_images > 0:
                cam_images = state.get("current_per_camera", {})
                min_so_far = min(cam_images.values()) if cam_images else 0
                if min_so_far >= target_images:
                    print(f"Target images per camera ({target_images}) reached. Stopping.")
                    break

            # Drive proxies
            if proxy_states and tm is None:
                _drive_proxies(proxy_states, config, dt_seconds, carla)

            # Clean up dead/end-of-tunnel proxies
            _cleanup_proxies(proxy_states, lane_points)

            # Replenish proxies
            now = time.time()
            if now >= next_replenish_time and config.proxy_enable:
                _replenish_proxies(world, proxy_states, config, rng, lane_lists, bp_lib,
                                   tm, target_speed, spawned, target_lane_count)
                next_replenish_time = now + 1.5

            # Status log
            if now >= next_status_log_time:
                active = sum(1 for p in proxy_states if p.get("active") and p.get("vehicle") is not None)
                print(f"代理车: {active}/{len(proxy_states)}, "
                      f"runs={state['runs_completed']}/{max_runs}, state={auto_state}")
                next_status_log_time = now + 5.0

            world.tick()

            # ---- Auto state machine ----
            if auto_state == "IDLE":
                chosen_state, chosen_vehicle = _pick_next_target(proxy_states, config, state)
                if chosen_state is None or chosen_vehicle is None:
                    print("No suitable proxy vehicle found. Waiting...")
                    time.sleep(2.0)
                    continue
                current_capture_vehicle = chosen_vehicle
                current_capture_state = chosen_state
                current_actor_id = int(current_vehicle_id(chosen_vehicle))
                target_dir = _target_output_path(config, current_actor_id, "proxy")
                current_run_dir = target_dir / datetime.now().strftime("run_%Y%m%d_%H%M%S")
                print(f"Starting collection: actor_id={current_actor_id}, "
                      f"run_dir={current_run_dir}")
                collector.set_vehicle(
                    current_capture_vehicle,
                    lane_points=chosen_state.get("lane_points", lane_points),
                    output_root=current_run_dir,
                )
                _write_run_actors_json(current_run_dir, current_capture_vehicle, chosen_state)
                collector.set_enabled(True)
                auto_state = "COLLECTING"
                auto_state_since = time.time()
                start_tick = 0

            elif auto_state == "COLLECTING":
                if current_capture_state is not None and current_capture_vehicle is not None:
                    # Check if vehicle still alive
                    if not current_capture_vehicle.is_alive:
                        print(f"Target vehicle {current_actor_id} died during collection")
                        collector.set_enabled(False)
                        auto_state = "COOLDOWN"
                        auto_state_since = time.time()
                        continue

                    # Feed collector with control
                    nearest_idx = int(current_capture_state.get("nearest_idx", 0))
                    control = current_capture_state.get("last_control")
                    if control is None:
                        try:
                            control = current_capture_vehicle.get_control()
                        except Exception:
                            control = None
                    if control is not None:
                        try:
                            world_frame = int(world.get_snapshot().frame)
                        except Exception:
                            world_frame = start_tick
                        try:
                            collector.on_tick(world_frame, nearest_idx, control)
                        except Exception:
                            print("Collect write error, stopping")
                            collector.set_enabled(False)
                            auto_state = "COOLDOWN"
                            auto_state_since = time.time()
                            continue

                # Check time elapsed
                elapsed = time.time() - auto_state_since
                if elapsed >= seconds_per_run:
                    print(f"Run {state['runs_completed'] + 1} complete: {elapsed:.1f}s")
                    collector.set_enabled(False)

                    # Count images collected in this run
                    cam_names = [spec.name for spec in getattr(collector, "_active_cameras", []) if not spec.name.endswith(
                        str(getattr(config, "collect_instance_suffix", "_instance")))]
                    run_images = _count_images_in_run(current_run_dir, cam_names) if current_run_dir else {}
                    current_per = state.get("current_per_camera", {})
                    for cam, cnt in run_images.items():
                        current_per[cam] = current_per.get(cam, 0) + cnt
                    state["current_per_camera"] = current_per

                    # Update target collections count
                    target_collections = state.get("target_collections", {})
                    target_collections[str(current_actor_id)] = target_collections.get(str(current_actor_id), 0) + 1
                    state["target_collections"] = target_collections

                    # Record completed run
                    completed = state.get("completed_runs", [])
                    completed.append({
                        "proxy": f"proxy_{current_actor_id}",
                        "run_dir": str(current_run_dir) if current_run_dir else "",
                        "seconds": round(elapsed, 1),
                        "images_per_camera": run_images,
                    })
                    state["completed_runs"] = completed

                    state["runs_completed"] = state["runs_completed"] + 1
                    _save_batch_state(batch_dir, state)
                    _save_batch_summary(batch_dir, state, started_at)

                    print(f"Progress: {state['runs_completed']}/{max_runs} runs, "
                          f"cameras: {dict(list(current_per.items())[:3])}...")

                    auto_state = "COOLDOWN"
                    auto_state_since = time.time()

            elif auto_state == "COOLDOWN":
                if time.time() - auto_state_since >= cooldown_s:
                    # Unbind collector from previous vehicle
                    current_capture_vehicle = None
                    current_capture_state = None
                    current_run_dir = None
                    auto_state = "IDLE"

    except KeyboardInterrupt:
        print("Interrupted. Saving progress...")
        _save_batch_state(batch_dir, state)
        _save_batch_summary(batch_dir, state, started_at)
    finally:
        if collector is not None:
            collector.destroy()
        destroy_spawned_actors(spawned)
        if original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        _save_batch_state(batch_dir, state)
        _save_batch_summary(batch_dir, state, started_at)
        print("Auto collect finished.")


def current_vehicle_id(vehicle) -> int:
    return int(getattr(vehicle, "id", -1))


def _drive_proxies(proxy_states: list[dict], config, dt_seconds, carla):
    """Drive all active proxy vehicles for one tick."""
    lane_groups: dict[int, list[dict]] = {}
    for p in proxy_states:
        if not p.get("active") or p.get("vehicle") is None or not p["vehicle"].is_alive:
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
                try:
                    front_gap_m = p_loc.distance(group[i + 1]["vehicle"].get_location())
                except Exception:
                    front_gap_m = None
            p_control = compute_control(
                carla, pv, p["lane_points"][p["nearest_idx"]],
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
            p["last_control"] = p_control


def _cleanup_proxies(proxy_states: list[dict], lane_points):
    """Remove vehicles that have driven past the end of the tunnel."""
    kept = []
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
            kept.append(p)
        except Exception:
            continue
    proxy_states[:] = kept


def _replenish_proxies(world, proxy_states, config, rng, lane_lists, bp_lib, tm, target_speed, spawned, target_lane_count):
    """Spawn new proxy vehicles to maintain target density."""
    lane_counts: dict[int, int] = {}
    for p in proxy_states:
        if p.get("active") and p.get("vehicle") is not None and p["vehicle"].is_alive:
            lane_counts[p["lane_id"]] = lane_counts.get(p["lane_id"], 0) + 1

    saved_min = getattr(config, "proxy_min_per_lane")
    saved_max = getattr(config, "proxy_max_per_lane")
    saved_start_ratio = getattr(config, "proxy_spawn_start_ratio")
    try:
        setattr(config, "proxy_min_per_lane", 1)
        setattr(config, "proxy_max_per_lane", 1)
        setattr(config, "proxy_spawn_start_ratio", min(float(saved_start_ratio), 0.08))
        plans = build_vehicle_plan(world, lane_lists, config, rng)
        for plan in plans:
            lane_idx = int(plan.lane_index)
            if lane_counts.get(lane_idx, 0) >= target_lane_count:
                continue
            if _spawn_one_proxy(world, plan, lane_lists, config, bp_lib, tm, target_speed, spawned, proxy_states):
                lane_counts[lane_idx] = lane_counts.get(lane_idx, 0) + 1
    finally:
        setattr(config, "proxy_min_per_lane", saved_min)
        setattr(config, "proxy_max_per_lane", saved_max)
        setattr(config, "proxy_spawn_start_ratio", saved_start_ratio)


def _spawn_one_proxy(world, plan, lane_lists, config, bp_lib, tm, target_speed, spawned, proxy_states) -> bool:
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
    carla = _require_carla()
    proxy_tf = carla.Transform(
        carla.Location(start_wp.location.x, start_wp.location.y, 1.2),
        carla.Rotation(pitch=0.0, yaw=spawn_yaw, roll=0.0),
    )
    proxy_vehicle = try_spawn_vehicle(
        world, proxy_bp, proxy_tf, config.proxy_spawn_clearance_m, spawned,
        lane_aware_clearance=bool(getattr(config, "proxy_spawn_lane_aware", False)),
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
    proxy_states.append(_build_proxy_state(
        world, proxy_vehicle, lane_index, wp_idx, lane_wps,
        target_speed * (1.0 - float(plan.speed_diff_percent) / 100.0),
    ))
    return True


if __name__ == "__main__":
    main()
