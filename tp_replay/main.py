"""tp_replay 主入口（从历史 replay_main.py 迁移）。

设计目标：
- 入口尽量“薄”：只做 env 覆盖、日志初始化、CARLA world 设置保护，以及调用 ReplayEngine。
- 保持 import 时无副作用：auto_control_main.py 会延迟导入本模块。
"""

from __future__ import annotations

import io
import json
import logging
import math
import os
import sys
from typing import Optional
from pathlib import Path

from . import config
from .carla_compat import require_carla
from .engine import ReplayEngine, _info
from .env_utils import (
    _ensure_logging_to_file,
    _get_bool_from_env,
    _get_float_from_env,
    _get_float_list_from_env,
    _get_int_from_env,
    _get_speed_factor_from_env,
    _get_str_from_env,
)
from .world_utils import _maybe_generate_opendrive_world


logger = logging.getLogger(__name__)


def _run_stitch_simple(world, client, engine, settings) -> None:
    """拼接轨迹回放——简单直接：逐 tick 插值 drive，与单车测试逻辑一致。"""
    carla = require_carla()
    stitch_json = os.getenv("TP_STITCH_JSON_PATH") or config.STITCH_JSON_PATH
    if not stitch_json:
        _info("错误：需要设置 TP_STITCH_JSON_PATH")
        return

    anchor_file = config.DATA_FILE_PATH
    if not anchor_file or not os.path.exists(anchor_file):
        _info(f"警告：锚点文件不存在 {anchor_file}")
    else:
        engine.process_data(anchor_file)
        engine.pending_tracks = []

    ca = engine._stitch_cos_a; sa = engine._stitch_sin_a
    ox = engine._stitch_off_x; oy = engine._stitch_off_y
    ds = engine._data_start_point
    sc = float(config.DATA_SCALE); sy = float(config.SCALE_Y)

    # ── 加载 + 过滤 ──
    import json
    with open(stitch_json, "r", encoding="utf-8") as f:
        data = json.load(f)
    all_trajs = data.get("trajectories", data if isinstance(data, list) else [])
    min_q = _get_float_from_env("TP_STITCH_MIN_QUALITY", 0.5)
    min_c = _get_int_from_env("TP_STITCH_MIN_CAMERAS", 2)
    max_t = _get_int_from_env("TP_TRACK_LIMIT", 0) or None

    def _mono(t):
        nodes = t.get("nodes", [])
        return sum(1 for i in range(len(nodes)-1) if nodes[i+1]["y"] < nodes[i]["y"]) == 0

    candidates = [t for t in sorted(all_trajs, key=lambda x: x["quality_score"], reverse=True)
                  if t["quality_score"] >= min_q and t["camera_count"] >= min_c and _mono(t)]
    if max_t: candidates = candidates[:max_t]
    if not candidates: _info("无轨迹"); return

    # 时间窗口过滤
    max_start_s = _get_float_from_env("TP_STITCH_MAX_START_S", 600.0)
    candidates = sorted(candidates, key=lambda t: t["nodes"][0]["timestamp"])
    gts = candidates[0]["nodes"][0]["timestamp"]
    candidates = [t for t in candidates if (t["nodes"][0]["timestamp"] - gts) / 1000.0 < max_start_s]
    _info(f"加载 {len(candidates)} 条轨迹")

    # ── 全部转为世界坐标路点 ──
    all_wpts = {}  # traj_id -> [(loc, spd_kmh, cam)]
    for st in candidates:
        wpts = []
        for n in st["nodes"]:
            y = n["y"]; x = n.get("x") or ds[0]
            rx = (x - ds[0]) * sc; ry = (y - ds[1]) * sc * sy
            loc = carla.Location(rx*ca - ry*sa + engine.entry_loc.x + ox,
                                  rx*sa + ry*ca + engine.entry_loc.y + oy,
                                  engine.entry_loc.z)
            try:
                wp = engine.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                if wp: loc = wp.transform.location
            except: pass
            wpts.append((loc, n.get("speed") or 0, n["timestamp"]))
        all_wpts[id(st)] = wpts

    # ── Spawn 调度 ──
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find("vehicle.tesla.model3")
    if bp is None: bp = bp_lib.filter("vehicle.*")[0]
    bp.set_attribute("role_name", "stitch")

    pending = list(candidates)  # 待生成
    active = {}     # actor -> (traj, wpts, idx, sim_dist, smooth_yaw, segs)
    max_active = _get_int_from_env("TP_STITCH_TM_MAX_ACTIVE", 25)

    _try_set_spectator_view(world, engine)
    fixed_dt = float(settings.fixed_delta_seconds)
    _info(f"回放中...")
    tick_idx = 0

    try:
        while pending or active:
            world.tick()
            tick_idx += 1

            # Spawn 到期车辆
            sim_time = tick_idx * fixed_dt * float(config.PLAYBACK_SPEED)
            spawn_this_tick = []
            for st in list(pending):
                first_ts = st["nodes"][0]["timestamp"]
                if (first_ts - gts) / 1000.0 <= sim_time and len(active) < max_active:
                    pending.remove(st)
                    spawn_this_tick.append(st)
                else:
                    break  # pending 已按时间排序，后面的更晚

            for st in spawn_this_tick:
                wpts = all_wpts[id(st)]
                p0, p1 = wpts[0][0], wpts[1][0]
                yaw0 = math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))
                # 往前走 1 个路点避开入口边缘
                try:
                    wp = engine.map.get_waypoint(p0, project_to_road=True, lane_type=carla.LaneType.Driving)
                    if wp:
                        nxt = list(wp.previous(2.0))
                        if nxt:
                            p0 = carla.Location(nxt[0].transform.location.x, nxt[0].transform.location.y, nxt[0].transform.location.z)
                            yaw0 = nxt[0].transform.rotation.yaw
                except: pass
                actor = world.try_spawn_actor(bp, carla.Transform(p0, carla.Rotation(yaw=yaw0)))
                if actor is None:
                    for off in [5, 10, 20]:
                        pp = carla.Location(p0.x, p0.y + off, p0.z)
                        actor = world.try_spawn_actor(bp, carla.Transform(pp, carla.Rotation(yaw=yaw0)))
                        if actor: break
                if actor is None: continue
                # 预计算段
                segs = []
                for i in range(len(wpts) - 1):
                    d = wpts[i][0].distance(wpts[i + 1][0])
                    spd = max(0.1, (wpts[i][1] + wpts[i + 1][1]) / 2) / 3.6
                    segs.append((d, spd))
                active[actor.id] = (st, wpts, 0, 0.0, yaw0, segs)

            # 更新活跃车辆
            to_remove = []
            for aid, (st, wpts, idx, sim_dist, smooth_yaw, segs) in list(active.items()):
                actor = world.get_actor(aid) if hasattr(world, 'get_actor') else None
                if actor is None or not actor.is_alive:
                    to_remove.append(aid); continue

                if idx >= len(segs):
                    to_remove.append(aid); continue

                seg_d, seg_spd = segs[idx]
                step = seg_spd * fixed_dt * float(config.PLAYBACK_SPEED)
                sim_dist += step

                while idx < len(segs) and sim_dist > seg_d:
                    sim_dist -= seg_d; idx += 1
                    if idx < len(segs): seg_d, seg_spd = segs[idx]

                if idx >= len(segs):
                    # 终点：保持最后一帧位置
                    last_pt = wpts[-1][0]
                    actor.set_transform(carla.Transform(last_pt, carla.Rotation(yaw=smooth_yaw)))
                    to_remove.append(aid); continue

                alpha = max(0, min(1, sim_dist / max(seg_d, 0.01)))
                p0, p1 = wpts[idx][0], wpts[min(idx + 1, len(wpts) - 1)][0]
                ix = p0.x + (p1.x - p0.x) * alpha
                iy = p0.y + (p1.y - p0.y) * alpha
                raw_yaw = math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))
                diff = (raw_yaw - smooth_yaw + 180) % 360 - 180
                smooth_yaw += diff * 0.3

                actor.set_transform(carla.Transform(carla.Location(ix, iy, p0.z), carla.Rotation(yaw=smooth_yaw)))
                active[aid] = (st, wpts, idx, sim_dist, smooth_yaw, segs)

            for aid in to_remove: del active[aid]

            if tick_idx % 20 == 0:
                print(f"\rt={tick_idx*fixed_dt:.0f}s active={len(active)} pending={len(pending)}", end="")

        _info("\n回放完成")
    except KeyboardInterrupt:
        _info("\n退出")
    finally:
        for aid in list(active.keys()):
            try:
                a = world.get_actor(aid)
                if a and a.is_alive: a.destroy()
            except: pass


def _run_stitch_kinematic(world, client, engine, settings) -> None:
    """拼接轨迹 kinematic 回放模式。

    用引擎原生的 kinematic tick 驱动全帧 VehicleTrack，
    引擎内置的路点吸附、插值、yaw gating 保证平滑行驶。
    """
    from .stitch_adapter import StitchAdapter

    stitch_json = os.getenv("TP_STITCH_JSON_PATH") or config.STITCH_JSON_PATH
    if not stitch_json:
        _info("错误：stitch_kinematic 模式需要设置 TP_STITCH_JSON_PATH")
        return

    # Phase 1: 锚点数据 → 计算坐标变换参数
    anchor_file = config.DATA_FILE_PATH
    if not anchor_file or not os.path.exists(anchor_file):
        _info(f"警告：锚点文件不存在 {anchor_file}，使用默认变换")
    else:
        engine.process_data(anchor_file)
        engine.pending_tracks = []

    # Phase 2: 加载拼接轨迹（全帧模式——每节点一个 TrackFrame）
    adapter = StitchAdapter(engine)
    stitch_tracks = adapter.load_for_kinematic(
        stitch_json,
        min_quality=_get_float_from_env("TP_STITCH_MIN_QUALITY", float(config.STITCH_MIN_QUALITY)),
        min_cameras=_get_int_from_env("TP_STITCH_MIN_CAMERAS", int(config.STITCH_MIN_CAMERAS)),
        max_tracks=_get_int_from_env("TP_TRACK_LIMIT", 0) or None,
    )
    if not stitch_tracks:
        _info("错误：未加载到有效的拼接轨迹")
        return

    # 时间窗口过滤
    max_start_s = _get_float_from_env("TP_STITCH_MAX_START_S", 600.0)
    stitch_tracks = sorted(stitch_tracks, key=lambda t: t.start_time)
    earliest = stitch_tracks[0].start_time if stitch_tracks else 0.0
    stitch_tracks = [t for t in stitch_tracks if t.start_time - earliest < max_start_s]
    _info(f"Time-filtered: {len(stitch_tracks)} tracks in first {max_start_s}s")

    # 设置 spawn 时间（引擎的 spawn 调度依赖此字段）
    for t in stitch_tracks:
        t.next_spawn_time = t.start_time

    engine.pending_tracks = stitch_tracks
    _try_set_spectator_view(world, engine)

    _info(f"Stitch-Kinematic: {len(stitch_tracks)} trajectories | "
          f"max_active={config.MAX_ACTIVE_VEHICLES}")

    # Phase 3: 引擎原生主循环
    fixed_dt = float(settings.fixed_delta_seconds)
    tick_idx = 0

    while True:
        world.tick()
        active_cnt = engine.tick(fixed_dt)
        tick_idx += 1

        if tick_idx % max(1, int(config.PRINT_EVERY_N_TICKS)) == 0:
            print(
                f"\rTrajTime: {engine.current_traj_time:.2f}s | Active: {active_cnt} "
                f"| SpawnOK: {engine.stats.get('spawn_success',0)} "
                f"| SpawnFail: {engine.stats['spawn_failures']} "
                f"| Finished: {engine.stats.get('finished_tracks',0)} "
                f"| Pending: {len(engine.pending_tracks)}   ",
                end="",
            )

        if tick_idx % max(1, int(config.STATUS_LOG_EVERY_N_TICKS)) == 0:
            logger.info(
                "Status: time=%.2fs active=%d pending=%d spawned=%d finished=%d",
                engine.current_traj_time, active_cnt,
                len(engine.pending_tracks),
                engine.stats.get("spawn_success", 0),
                engine.stats.get("finished_tracks", 0),
            )

        if not engine.pending_tracks and active_cnt == 0:
            if config.MAX_TRAJ_TIME_SECONDS and engine.current_traj_time < float(config.MAX_TRAJ_TIME_SECONDS):
                continue
            _info("\nStitch-Kinematic playback finished.")
            break

        if config.MAX_TRAJ_TIME_SECONDS and engine.current_traj_time >= float(config.MAX_TRAJ_TIME_SECONDS):
            _info(f"\nStop: reached TP_MAX_TRAJ_TIME")
            break


def _run_stitch_autopilot(world, client, engine, settings) -> None:
    """TM 自动驾驶模式：拼接轨迹 → TM 自动驾驶回放。"""
    from .stitch_adapter import StitchAdapter
    from .stitch_autopilot import StitchAutopilot

    stitch_json = os.getenv("TP_STITCH_JSON_PATH") or config.STITCH_JSON_PATH
    if not stitch_json:
        _info("错误：stitch_autopilot 模式需要设置 TP_STITCH_JSON_PATH")
        return

    # Phase 1: 运行 process_data 获取坐标变换参数
    anchor_file = config.DATA_FILE_PATH
    if not anchor_file or not os.path.exists(anchor_file):
        _info(f"警告：锚点文件不存在 {anchor_file}，使用默认变换")
    else:
        engine.process_data(anchor_file)
        engine.pending_tracks = []

    # Phase 2: 加载拼接轨迹
    adapter = StitchAdapter(engine)
    stitch_tracks = adapter.load_and_convert(
        stitch_json,
        min_quality=_get_float_from_env("TP_STITCH_MIN_QUALITY", float(config.STITCH_MIN_QUALITY)),
        min_cameras=_get_int_from_env("TP_STITCH_MIN_CAMERAS", int(config.STITCH_MIN_CAMERAS)),
        max_tracks=_get_int_from_env("TP_TRACK_LIMIT", 0) or None,
    )
    if not stitch_tracks:
        _info("错误：未加载到有效的拼接轨迹")
        return

    # 过滤：只保留前 N 秒内开始的轨迹（避免 11 天跨度导致车辆永不生成）
    max_start_s = _get_float_from_env("TP_STITCH_MAX_START_S", 600.0)
    stitch_tracks = sorted(stitch_tracks, key=lambda t: t.start_time)
    earliest = stitch_tracks[0].start_time if stitch_tracks else 0.0
    stitch_tracks = [t for t in stitch_tracks if t.start_time - earliest < max_start_s]
    _info(f"Time-filtered to {len(stitch_tracks)} tracks (first {max_start_s}s window)")

    # Phase 3: 创建 autopilot 编排器
    autopilot = StitchAutopilot(world, client, engine)
    autopilot.load(stitch_tracks)

    _try_set_spectator_view(world, engine)
    _info(f"Stitch-Autopilot: {len(stitch_tracks)} trajectories | "
          f"TM active={autopilot._spawned_total} max={config.STITCH_TM_MAX_ACTIVE}")

    # Phase 4: 主循环
    fixed_dt = float(settings.fixed_delta_seconds)
    tick_idx = 0

    while True:
        world.tick()
        active_cnt = autopilot.tick(fixed_dt)
        tick_idx += 1
        s = autopilot.stats  # 确保 s 在每次迭代都定义（避免条件块内赋值导致未定义）

        if tick_idx % max(1, int(config.PRINT_EVERY_N_TICKS)) == 0:
            print(
                f"\rTime: {s['current_time']:.1f}s | Active: {s['active']} "
                f"| Spawned: {s['spawned']} | Finished: {s['finished']} "
                f"| Pending: {s['pending']} | Fail: {s['failures']}   ",
                end="",
            )

        if tick_idx % max(1, int(config.STATUS_LOG_EVERY_N_TICKS)) == 0:
            logger.info(
                "Status: time=%.1fs active=%d pending=%d spawned=%d finished=%d failures=%d",
                s["current_time"], s["active"], s["pending"],
                s["spawned"], s["finished"], s["failures"],
            )

        if s["pending"] == 0 and s["active"] == 0:
            if config.MAX_TRAJ_TIME_SECONDS and s["current_time"] < float(config.MAX_TRAJ_TIME_SECONDS):
                continue
            _info("\nStitch-Autopilot playback finished.")
            s_final = autopilot.stats
            logger.info(
                "Final stats: spawned=%d finished=%d failures=%d",
                s_final["spawned"], s_final["finished"], s_final["failures"],
            )
            break

        if config.MAX_TRAJ_TIME_SECONDS and s["current_time"] >= float(config.MAX_TRAJ_TIME_SECONDS):
            _info(f"\nStop: reached TP_MAX_TRAJ_TIME={float(config.MAX_TRAJ_TIME_SECONDS):.2f}s")
            break

    autopilot.cleanup()


def _apply_env_overrides_to_config() -> None:
    """Mutate tp_replay.config module-level defaults using env vars.

    This mirrors the legacy behavior in replay_main.main() where globals were
    overridden in-place.
    """

    # Track limit (allow -1 to mean None)
    track_limit = _get_int_from_env(
        "TP_TRACK_LIMIT",
        config.TRACK_LIMIT if config.TRACK_LIMIT is not None else -1,
    )
    if track_limit == -1:
        config.TRACK_LIMIT = None
    else:
        config.TRACK_LIMIT = track_limit

    config.PRINT_EVERY_N_TICKS = _get_int_from_env(
        "TP_PRINT_EVERY_N_TICKS",
        int(config.PRINT_EVERY_N_TICKS),
    )
    config.STATUS_LOG_EVERY_N_TICKS = _get_int_from_env(
        "TP_STATUS_LOG_EVERY_N_TICKS",
        int(config.STATUS_LOG_EVERY_N_TICKS),
    )
    config.MAX_TRAJ_TIME_SECONDS = _get_float_from_env(
        "TP_MAX_TRAJ_TIME",
        float(config.MAX_TRAJ_TIME_SECONDS),
    )

    config.FINISH_BEHAVIOR = _get_str_from_env("TP_FINISH_BEHAVIOR", config.FINISH_BEHAVIOR)
    config.FINISH_TELEPORT_Z = _get_float_from_env(
        "TP_FINISH_TELEPORT_Z",
        float(config.FINISH_TELEPORT_Z),
    )

    config.SPAWN_RETRY_DELAY_SECONDS = _get_float_from_env(
        "TP_SPAWN_RETRY_DELAY_SECONDS",
        float(config.SPAWN_RETRY_DELAY_SECONDS),
    )
    config.SPAWN_MAX_ATTEMPTS = _get_int_from_env(
        "TP_SPAWN_MAX_ATTEMPTS",
        int(config.SPAWN_MAX_ATTEMPTS),
    )
    config.SPAWN_OVERLAP_BLOCK_DIST_M = _get_float_from_env(
        "TP_SPAWN_OVERLAP_BLOCK_DIST_M",
        float(config.SPAWN_OVERLAP_BLOCK_DIST_M),
    )
    config.SPAWN_CANDIDATE_OFFSETS_M = _get_float_list_from_env(
        "TP_SPAWN_CANDIDATE_OFFSETS_M",
        list(config.SPAWN_CANDIDATE_OFFSETS_M),
    )
    config.SPAWN_TRY_ADJACENT_LANES = _get_bool_from_env(
        "TP_SPAWN_TRY_ADJACENT_LANES",
        bool(config.SPAWN_TRY_ADJACENT_LANES),
    )
    config.SPAWN_RETRY_MIN_REMAINING_SECONDS = _get_float_from_env(
        "TP_SPAWN_RETRY_MIN_REMAINING_SECONDS",
        float(config.SPAWN_RETRY_MIN_REMAINING_SECONDS),
    )
    config.SPAWN_RETRY_MIN_DELAY_SECONDS = _get_float_from_env(
        "TP_SPAWN_RETRY_MIN_DELAY_SECONDS",
        float(config.SPAWN_RETRY_MIN_DELAY_SECONDS),
    )
    config.SPAWN_Z_OFFSET_M = _get_float_from_env(
        "TP_SPAWN_Z_OFFSET_M",
        float(config.SPAWN_Z_OFFSET_M),
    )
    config.SPAWN_MAX_CANDIDATES = _get_int_from_env(
        "TP_SPAWN_MAX_CANDIDATES",
        int(config.SPAWN_MAX_CANDIDATES),
    )

    # MAX_ACTIVE_VEHICLES: empty -> keep default; <=0 -> None
    max_active_raw = os.getenv("TP_MAX_ACTIVE_VEHICLES")
    if max_active_raw is not None and str(max_active_raw).strip() != "":
        v = _get_int_from_env("TP_MAX_ACTIVE_VEHICLES", 0)
        config.MAX_ACTIVE_VEHICLES = None if v <= 0 else v

    config.SPAWN_DEFER_SECONDS = _get_float_from_env(
        "TP_SPAWN_DEFER_SECONDS",
        float(config.SPAWN_DEFER_SECONDS),
    )

    config.PLAYBACK_SPEED = _get_float_from_env(
        "TP_PLAYBACK_SPEED",
        float(config.PLAYBACK_SPEED),
    )
    config.ENABLE_PHYSICS = _get_bool_from_env(
        "TP_ENABLE_PHYSICS",
        bool(config.ENABLE_PHYSICS),
    )

    config.DATA_SPEED_IN_KMH = _get_bool_from_env(
        "TP_DATA_SPEED_IN_KMH",
        bool(config.DATA_SPEED_IN_KMH),
    )
    config.SPEED_FACTOR = _get_speed_factor_from_env(float(config.SPEED_FACTOR))

    config.REPLAY_CONTROL_MODE = _get_str_from_env(
        "TP_REPLAY_CONTROL_MODE",
        config.REPLAY_CONTROL_MODE,
    ).strip().lower()
    if config.REPLAY_CONTROL_MODE == "kinematic":
        config.ENABLE_PHYSICS = False

    config.PHYSICS_SYNC_DIST = _get_float_from_env(
        "TP_PHYSICS_SYNC_DIST",
        float(config.PHYSICS_SYNC_DIST),
    )
    config.LOOKAHEAD_TIME = _get_float_from_env(
        "TP_LOOKAHEAD_TIME",
        float(config.LOOKAHEAD_TIME),
    )
    config.LOOKAHEAD_SPEED_GAIN = _get_float_from_env(
        "TP_LOOKAHEAD_SPEED_GAIN",
        float(config.LOOKAHEAD_SPEED_GAIN),
    )
    config.CATCH_UP_GAIN = _get_float_from_env(
        "TP_CATCH_UP_GAIN",
        float(config.CATCH_UP_GAIN),
    )
    config.SNAP_THRESHOLD = _get_float_from_env(
        "TP_SNAP_THRESHOLD",
        float(config.SNAP_THRESHOLD),
    )
    config.SNAP_STRICT_DIST = _get_float_from_env(
        "TP_SNAP_STRICT_DIST",
        float(config.SNAP_STRICT_DIST),
    )
    config.LANE_CHANGE_PENALTY_M = _get_float_from_env(
        "TP_LANE_CHANGE_PENALTY_M",
        float(config.LANE_CHANGE_PENALTY_M),
    )
    config.LANE_SWITCH_MIN_IMPROVEMENT_M = _get_float_from_env(
        "TP_LANE_SWITCH_MIN_IMPROVEMENT_M",
        float(config.LANE_SWITCH_MIN_IMPROVEMENT_M),
    )
    config.ENFORCE_SAME_ROAD_ID = _get_bool_from_env(
        "TP_ENFORCE_SAME_ROAD_ID",
        bool(config.ENFORCE_SAME_ROAD_ID),
    )
    config.ROTATION_BIAS_DEG = _get_float_from_env(
        "TP_ROTATION_BIAS_DEG",
        float(config.ROTATION_BIAS_DEG),
    )
    config.DATA_ANGLE_BASELINE_M = _get_float_from_env(
        "TP_DATA_ANGLE_BASELINE_M",
        float(config.DATA_ANGLE_BASELINE_M),
    )

    config.XODR_PATH = os.getenv("TP_XODR_PATH") or config.XODR_PATH
    config.DATA_FILE_PATH = os.getenv("TP_DATA_FILE_PATH") or config.DATA_FILE_PATH


def _try_set_spectator_view(world, engine) -> None:
    """Apply spectator view from a JSON file if present; fallback to anchor view."""

    carla = require_carla()

    try:
        spectator = world.get_spectator()
    except Exception as e:
        logger.debug("get_spectator failed: %s", e)
        return

    cam_path = _get_str_from_env("TP_SPECTATOR_VIEW_JSON", "")
    repo_root = Path(__file__).resolve().parents[1]
    default_cam = str(repo_root / "camera_view.json")
    candidates = [p for p in [cam_path, default_cam] if p]

    last_error: Optional[Exception] = None
    for p in candidates:
        try:
            if not os.path.exists(p):
                raise FileNotFoundError(p)
            with open(p, "r", encoding="utf-8") as f:
                cam = json.load(f)
            cam_loc = carla.Location(
                cam["location"]["x"],
                cam["location"]["y"],
                cam["location"]["z"],
            )
            cam_rot = carla.Rotation(
                pitch=cam["rotation"]["pitch"],
                yaw=cam["rotation"]["yaw"],
                roll=cam["rotation"]["roll"],
            )
            spectator.set_transform(carla.Transform(cam_loc, cam_rot))
            logger.info("Applied spectator view from %s", p)
            return
        except (OSError, json.JSONDecodeError, KeyError) as e:
            last_error = e

    if last_error is not None:
        logger.info("Use fallback spectator view: %s", last_error)

    try:
        fwd = engine.entry_fwd
        vf = -1.0 if bool(config.REVERSE_DIRECTION) else 1.0
        cam_loc = carla.Location(
            engine.entry_loc.x - fwd.x * 10 * vf,
            engine.entry_loc.y - fwd.y * 10 * vf,
            engine.entry_loc.z + 8,
        )
        yaw = math.degrees(math.atan2(fwd.y * vf, fwd.x * vf))
        spectator.set_transform(
            carla.Transform(cam_loc, carla.Rotation(pitch=-10, yaw=yaw))
        )
    except Exception as e:
        logger.debug("fallback spectator set_transform failed: %s", e)


def _maybe_apply_weather_override(world) -> None:
    """Optionally override weather using env vars.

    Preserves legacy semantics:
    - By default does nothing.
    - If TP_WEATHER_PRESET is set, apply the named carla.WeatherParameters preset.
    - Else if TP_FORCE_WEATHER=1, apply partial TP_WEATHER_* overrides.
    """

    carla = require_carla()

    force_weather = _get_bool_from_env("TP_FORCE_WEATHER", False)
    weather_preset = _get_str_from_env("TP_WEATHER_PRESET", "")
    if not (force_weather or weather_preset):
        return

    try:
        if weather_preset:
            preset_name = weather_preset.strip()
            preset = getattr(carla.WeatherParameters, preset_name, None)
            if preset is None:
                logger.warning(
                    "Unknown TP_WEATHER_PRESET=%r; expected a carla.WeatherParameters preset name (e.g. ClearNoon)",
                    preset_name,
                )
            else:
                world.set_weather(preset)
                logger.info("Applied TP_WEATHER_PRESET=%s", preset_name)
            return

        if force_weather:
            w = world.get_weather()
            cloud = os.getenv("TP_WEATHER_CLOUDINESS")
            if cloud not in (None, ""):
                w.cloudiness = float(cloud)
            rain = os.getenv("TP_WEATHER_PRECIPITATION")
            if rain not in (None, ""):
                w.precipitation = float(rain)
            fog = os.getenv("TP_WEATHER_FOG_DENSITY")
            if fog not in (None, ""):
                w.fog_density = float(fog)
            sun_alt = os.getenv("TP_WEATHER_SUN_ALTITUDE_ANGLE")
            if sun_alt not in (None, ""):
                w.sun_altitude_angle = float(sun_alt)
            sun_az = os.getenv("TP_WEATHER_SUN_AZIMUTH_ANGLE")
            if sun_az not in (None, ""):
                w.sun_azimuth_angle = float(sun_az)
            world.set_weather(w)
            logger.info("Applied TP_FORCE_WEATHER=1 (partial TP_WEATHER_* overrides)")
    except Exception as e:
        logger.warning("Failed to apply weather override: %s", e)


def main() -> None:
    """Run CARLA replay."""

    # Windows 控制台经常是 GBK：尽量让输出用 UTF-8，减少乱码
    engine = None

    try:
        if isinstance(sys.stdout, io.TextIOWrapper):
            sys.stdout.reconfigure(encoding="utf-8")
        if isinstance(sys.stderr, io.TextIOWrapper):
            sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

    log_path = _ensure_logging_to_file()
    if log_path:
        print(f"Logging to: {log_path}")

    _apply_env_overrides_to_config()

    logger.info(
        "Run config: XODR_PATH=%s DATA_FILE_PATH=%s TRACK_LIMIT=%s TP_SELECT_MODE=%s TP_MAX_TRAJ_TIME=%s TP_REBASE_TIME=%s "
        "PLAYBACK_SPEED=%s TP_REPLAY_CONTROL_MODE=%s ENABLE_PHYSICS=%s TP_DATA_SPEED_IN_KMH=%s TP_DATA_SPEED_FACTOR=%s PHYSICS_SYNC_DIST=%s SNAP_THRESHOLD=%s "
        "TP_SPAWN_MAX_ATTEMPTS=%s TP_SPAWN_RETRY_DELAY_SECONDS=%s TP_SPAWN_OVERLAP_BLOCK_DIST_M=%s "
        "TP_SPAWN_CANDIDATE_OFFSETS_M=%s TP_SPAWN_TRY_ADJACENT_LANES=%s TP_SPAWN_MAX_CANDIDATES=%s "
        "TP_SPAWN_RETRY_MIN_REMAINING_SECONDS=%s TP_SPAWN_RETRY_MIN_DELAY_SECONDS=%s TP_SPAWN_Z_OFFSET_M=%s "
        "TP_FINISH_BEHAVIOR=%s TP_FINISH_TELEPORT_Z=%s TP_MAX_ACTIVE_VEHICLES=%s TP_SPAWN_DEFER_SECONDS=%s",
        config.XODR_PATH,
        config.DATA_FILE_PATH,
        config.TRACK_LIMIT,
        (os.getenv("TP_SELECT_MODE") or "none"),
        config.MAX_TRAJ_TIME_SECONDS,
        (os.getenv("TP_REBASE_TIME") or "0"),
        config.PLAYBACK_SPEED,
        (os.getenv("TP_REPLAY_CONTROL_MODE") or config.REPLAY_CONTROL_MODE),
        config.ENABLE_PHYSICS,
        config.DATA_SPEED_IN_KMH,
        config.SPEED_FACTOR,
        config.PHYSICS_SYNC_DIST,
        config.SNAP_THRESHOLD,
        config.SPAWN_MAX_ATTEMPTS,
        config.SPAWN_RETRY_DELAY_SECONDS,
        config.SPAWN_OVERLAP_BLOCK_DIST_M,
        ",".join(str(x) for x in config.SPAWN_CANDIDATE_OFFSETS_M),
        config.SPAWN_TRY_ADJACENT_LANES,
        config.SPAWN_MAX_CANDIDATES,
        config.SPAWN_RETRY_MIN_REMAINING_SECONDS,
        config.SPAWN_RETRY_MIN_DELAY_SECONDS,
        config.SPAWN_Z_OFFSET_M,
        config.FINISH_BEHAVIOR,
        config.FINISH_TELEPORT_Z,
        config.MAX_ACTIVE_VEHICLES,
        config.SPAWN_DEFER_SECONDS,
    )

    if _get_bool_from_env("TP_SPAWN_SCHEDULE_COMPRESS", False):
        logger.info(
            "Spawn schedule config: enabled=1 mode=%s base=%.3f spread=%.3f jitter=%.3f seed=%r preview_n=%d",
            _get_str_from_env("TP_SPAWN_SCHEDULE_MODE", "uniform"),
            float(_get_float_from_env("TP_SPAWN_SCHEDULE_BASE_SECONDS", 0.0)),
            float(_get_float_from_env("TP_SPAWN_SCHEDULE_SPREAD_SECONDS", 10.0)),
            float(_get_float_from_env("TP_SPAWN_SCHEDULE_JITTER_SECONDS", 0.0)),
            os.getenv("TP_SPAWN_SCHEDULE_SEED"),
            int(_get_int_from_env("TP_SPAWN_SCHEDULE_PREVIEW_N", 10)),
        )

    carla = require_carla()

    try:
        host = _get_str_from_env("TP_CARLA_HOST", "localhost")
        port = _get_int_from_env("TP_CARLA_PORT", 2000)
        timeout = _get_float_from_env("TP_CARLA_TIMEOUT_SECONDS", 10.0)
        client = carla.Client(host, int(port))
        client.set_timeout(float(timeout))
        world = _maybe_generate_opendrive_world(client, config.XODR_PATH)
    except Exception as e:
        logger.error(
            "Failed to connect to CARLA server at %s:%s: %s. Is CARLA Server running?",
            os.getenv("TP_CARLA_HOST") or "localhost",
            os.getenv("TP_CARLA_PORT") or "2000",
            e,
        )
        raise

    # Keep a baseline for restoration.
    original_settings = None
    try:
        original_settings = world.get_settings()
    except Exception as e:
        logger.warning("Failed to get world settings: %s", e)

    try:
        # Weather override is optional and should apply to the final world.
        _maybe_apply_weather_override(world)

        # Apply deterministic synchronous settings (legacy behavior)
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = _get_float_from_env(
            "TP_FIXED_DELTA_SECONDS",
            0.05,
        )

        if _get_bool_from_env("TP_ENABLE_SUBSTEPPING", False):
            if hasattr(settings, "substepping"):
                settings.substepping = True
            if hasattr(settings, "max_substep_delta_time"):
                settings.max_substep_delta_time = _get_float_from_env(
                    "TP_MAX_SUBSTEP_DELTA_TIME",
                    0.01,
                )
            if hasattr(settings, "max_substeps"):
                settings.max_substeps = _get_int_from_env("TP_MAX_SUBSTEPS", 2)

        world.apply_settings(settings)

        engine = ReplayEngine(client, config.XODR_PATH)

        # —— 拼接轨迹回放模式 ——
        replay_mode = _get_str_from_env("TP_REPLAY_MODE", "kinematic").strip().lower()
        if replay_mode == "stitch_simple":
            _run_stitch_simple(world, client, engine, settings)
            return
        if replay_mode == "stitch_kinematic":
            _run_stitch_kinematic(world, client, engine, settings)
            return
        if replay_mode == "stitch_autopilot":
            _run_stitch_autopilot(world, client, engine, settings)
            return

        engine.process_data(config.DATA_FILE_PATH)
        if not engine.pending_tracks:
            logger.warning("No tracks loaded; exiting")
            return

        _try_set_spectator_view(world, engine)

        mode_label = os.getenv("TP_REPLAY_CONTROL_MODE") or config.REPLAY_CONTROL_MODE
        _info(f"开始回放 (mode={mode_label})...")

        tick_idx = 0
        fixed_dt = float(settings.fixed_delta_seconds)
        while True:
            world.tick()
            active_cnt = engine.tick(fixed_dt)
            tick_idx += 1

            if tick_idx % max(1, int(config.PRINT_EVERY_N_TICKS)) == 0:
                print(
                    f"\rTrajTime: {engine.current_traj_time:.2f}s | Active: {active_cnt} "
                    f"| Teleports: {engine.stats['teleports']} "
                    f"| SpawnOK: {engine.stats.get('spawn_success', 0)} "
                    f"| SpawnFail: {engine.stats['spawn_failures']} "
                    f"(None: {engine.stats.get('spawn_fail_none', 0)} Exc: {engine.stats.get('spawn_fail_exception', 0)}) "
                    f"| SkippedWP: {engine.stats['skipped_no_waypoint']} "
                    f"| ParsedLaneChg: {engine.stats.get('parsed_lane_changes', 0)} "
                    f"| Finished: {engine.stats.get('finished_tracks', 0)} Destroyed: {engine.stats.get('destroyed_tracks', 0)} "
                    f"| Pending: {len(engine.pending_tracks)}   ",
                    end="",
                )

            if tick_idx % max(1, int(config.STATUS_LOG_EVERY_N_TICKS)) == 0:
                logger.info(
                    "Status: traj_time=%.2fs active=%d pending=%d teleports=%d spawn_ok=%d spawn_fail=%d finished=%d destroyed=%d extended=%d parsed_lane_changes=%d snap_fallback_wp=%d skipped_wp=%d",
                    engine.current_traj_time,
                    active_cnt,
                    len(engine.pending_tracks),
                    engine.stats["teleports"],
                    engine.stats.get("spawn_success", 0),
                    engine.stats["spawn_failures"],
                    engine.stats.get("finished_tracks", 0),
                    engine.stats.get("destroyed_tracks", 0),
                    engine.stats.get("replays_extended", 0),
                    engine.stats.get("parsed_lane_changes", 0),
                    engine.stats.get("snap_fallback_waypoint", 0),
                    engine.stats["skipped_no_waypoint"],
                )

            if not engine.pending_tracks and active_cnt == 0:
                if config.MAX_TRAJ_TIME_SECONDS and engine.current_traj_time < float(
                    config.MAX_TRAJ_TIME_SECONDS
                ):
                    continue

                _info("\nPlayback finished.")
                logger.info(
                    "Final stats: teleports=%d spawn_ok=%d spawn_fail=%d (none=%d exc=%d) blocked_overlap=%d deferred_no_state=%d "
                    "skipped_past_end=%d abandoned_max_attempts=%d abandoned_near_end=%d candidates_tried=%d finished=%d destroyed=%d "
                    "extended=%d parsed_lane_changes=%d snap_fallback_wp=%d skipped_wp=%d parse_errors=%d",
                    engine.stats["teleports"],
                    engine.stats.get("spawn_success", 0),
                    engine.stats["spawn_failures"],
                    engine.stats.get("spawn_fail_none", 0),
                    engine.stats.get("spawn_fail_exception", 0),
                    engine.stats.get("spawn_blocked_overlap", 0),
                    engine.stats.get("spawn_deferred_no_state", 0),
                    engine.stats.get("spawn_skipped_past_end", 0),
                    engine.stats.get("spawn_abandoned_max_attempts", 0),
                    engine.stats.get("spawn_abandoned_near_end", 0),
                    engine.stats.get("spawn_candidates_tried", 0),
                    engine.stats.get("finished_tracks", 0),
                    engine.stats.get("destroyed_tracks", 0),
                    engine.stats.get("replays_extended", 0),
                    engine.stats.get("parsed_lane_changes", 0),
                    engine.stats.get("snap_fallback_waypoint", 0),
                    engine.stats["skipped_no_waypoint"],
                    engine.stats["parse_errors"],
                )
                if float(config.POST_PLAYBACK_HOLD_SECONDS) > 0.0:
                    hold_ticks = int(float(config.POST_PLAYBACK_HOLD_SECONDS) / fixed_dt)
                    for _ in range(max(0, hold_ticks)):
                        world.tick()
                break

            if config.MAX_TRAJ_TIME_SECONDS and engine.current_traj_time >= float(
                config.MAX_TRAJ_TIME_SECONDS
            ):
                print(
                    f"\nStop: reached TP_MAX_TRAJ_TIME={float(config.MAX_TRAJ_TIME_SECONDS):.2f}s"
                )
                logger.info("Stop due to max traj time: %.3fs", engine.current_traj_time)
                break

    except KeyboardInterrupt:
        _info("\nStop")
        if engine is not None:
            logger.info(
                "Interrupted: traj_time=%.3fs active=%d pending=%d teleports=%d spawn_ok=%d spawn_fail=%d (none=%d exc=%d) "
                "blocked_overlap=%d deferred_no_state=%d skipped_past_end=%d abandoned_max_attempts=%d abandoned_near_end=%d "
                "candidates_tried=%d finished=%d destroyed=%d parsed_lane_changes=%d snap_fallback_wp=%d skipped_wp=%d parse_errors=%d",
                engine.current_traj_time,
                len(engine.active_tracks),
                len(engine.pending_tracks),
                engine.stats.get("teleports", 0),
                engine.stats.get("spawn_success", 0),
                engine.stats.get("spawn_failures", 0),
                engine.stats.get("spawn_fail_none", 0),
                engine.stats.get("spawn_fail_exception", 0),
                engine.stats.get("spawn_blocked_overlap", 0),
                engine.stats.get("spawn_deferred_no_state", 0),
                engine.stats.get("spawn_skipped_past_end", 0),
                engine.stats.get("spawn_abandoned_max_attempts", 0),
                engine.stats.get("spawn_abandoned_near_end", 0),
                engine.stats.get("spawn_candidates_tried", 0),
                engine.stats.get("finished_tracks", 0),
                engine.stats.get("destroyed_tracks", 0),
                engine.stats.get("parsed_lane_changes", 0),
                engine.stats.get("snap_fallback_waypoint", 0),
                engine.stats.get("skipped_no_waypoint", 0),
                engine.stats.get("parse_errors", 0),
            )
    except Exception:
        logger.exception("Fatal error")
        raise
    finally:
        _info("Cleaning up...")

        # Best-effort destroy
        if engine is not None:
            try:
                for t in list(engine.active_tracks):
                    if t.actor:
                        t.actor.destroy()
            except Exception as e:
                logger.debug("destroy active actors failed: %s", e)
            try:
                for t in list(engine.pending_tracks):
                    if t.actor:
                        t.actor.destroy()
            except Exception as e:
                logger.debug("destroy pending actors failed: %s", e)

        if original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception as e:
                logger.warning("Failed to restore world settings: %s", e)


if __name__ == "__main__":
    main()
