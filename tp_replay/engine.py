import json
import logging
import math
import os
import random
from typing import Optional

from . import config
from .carla_compat import require_carla
from .env_utils import (
    _get_bool_from_env,
    _get_float_from_env,
    _get_int_from_env,
    _get_str_from_env,
)
from .geometry import (
    _calculate_stable_data_angle,
    _extract_xy_points_from_tokens,
    get_vector_angle_degrees,
    is_in_front,
)
from .models import TrackFrame, VehicleTrack
from .selection import _compress_tracks_spawn_schedule, _rebase_tracks_to_zero, _select_tracks


logger = logging.getLogger(__name__)


def _info(msg: str) -> None:
    print(msg)
    logger.info(msg)


def _lerp(a, b, alpha: float):
    try:
        aa = float(a)
        bb = float(b)
        t = float(alpha)
    except Exception:
        return b
    t = max(0.0, min(1.0, t))
    return aa * (1.0 - t) + bb * t


class ReplayEngine:
    def __init__(self, client, xodr_path):
        carla = require_carla()

        self.world = client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        xodr_path = (str(xodr_path).strip() if xodr_path is not None else "")

        map_obj = None
        xodr_error = None
        if xodr_path:
            try:
                _info(f"加载地图(XODR): {xodr_path}")
                with open(xodr_path, "r", encoding="utf-8") as f:
                    xodr_content = f.read()
                map_obj = carla.Map("Map", xodr_content)
            except (OSError, RuntimeError, ValueError, TypeError) as e:
                xodr_error = e
                logger.warning(
                    "Failed to load XODR_PATH=%s: %s; falling back to world.get_map()",
                    xodr_path,
                    e,
                )

        if map_obj is None:
            try:
                map_obj = self.world.get_map()
                _info("加载地图(world.get_map): ok")
            except Exception as e:
                if xodr_path:
                    raise RuntimeError(
                        f"Failed to load CARLA map from XODR_PATH={xodr_path!r} ({xodr_error}) and world.get_map() also failed ({e})."
                    ) from e
                raise RuntimeError(
                    f"Failed to load CARLA map from world.get_map() ({e}) and no TP_XODR_PATH was provided."
                ) from e

        self.map = map_obj

        self.entry_loc = carla.Location(
            float(config.TUNNEL_ENTRY_X),
            float(config.TUNNEL_ENTRY_Y),
            float(config.TUNNEL_ENTRY_Z),
        )
        entry_wp = self.map.get_waypoint(
            self.entry_loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if entry_wp is None:
            raise RuntimeError(
                f"Entry waypoint not found for entry_loc={self.entry_loc} "
                f"(lane_type=Driving). Check anchor coordinates / XODR map."
            )

        self.entry_loc.z = entry_wp.transform.location.z

        # 优化：寻找更可靠的道路方向
        next_wps = entry_wp.next(10.0)
        if next_wps:
            n_loc = next_wps[0].transform.location
            dx = n_loc.x - self.entry_loc.x
            dy = n_loc.y - self.entry_loc.y
            self.map_angle = get_vector_angle_degrees(dx, dy)
            l = math.hypot(dx, dy)
            if l > 1e-6:
                self.entry_fwd = carla.Vector3D(dx / l, dy / l, 0)
            else:
                self.entry_fwd = entry_wp.transform.get_forward_vector()
        else:
            self.entry_fwd = entry_wp.transform.get_forward_vector()
            self.map_angle = entry_wp.transform.rotation.yaw

        _info(f"锚点锁定. 地图方向: {self.map_angle:.2f}")

        self.pending_tracks = []
        self.active_tracks = []
        self.global_start_time_raw = float("inf")
        self.current_traj_time = 0.0
        self.stats = {
            "parse_errors": 0,
            "spawn_failures": 0,
            "spawn_success": 0,
            "spawn_fail_none": 0,
            "spawn_fail_exception": 0,
            "spawn_blocked_overlap": 0,
            "spawn_deferred_no_state": 0,
            "spawn_skipped_past_end": 0,
            "spawn_abandoned_max_attempts": 0,
            "spawn_abandoned_near_end": 0,
            "spawn_candidates_tried": 0,
            "teleports": 0,
            "finished_tracks": 0,
            "destroyed_tracks": 0,
            "parsed_lane_changes": 0,
            "snap_fallback_waypoint": 0,
            "skipped_no_waypoint": 0,
            "replays_extended": 0,
        }

        # Optional track replay extension (controlled via env)
        self.extend_replay = _get_bool_from_env("TP_EXTEND_REPLAY", False)
        self.extend_until_s = _get_float_from_env(
            "TP_EXTEND_UNTIL_SECONDS",
            float(config.MAX_TRAJ_TIME_SECONDS)
            if float(config.MAX_TRAJ_TIME_SECONDS or 0.0) > 0.0
            else 0.0,
        )
        self.extend_max_loops = _get_int_from_env("TP_EXTEND_MAX_LOOPS", 2)

    def _log_parse_error(self, msg, exc=None):
        self.stats["parse_errors"] += 1
        if self.stats["parse_errors"] <= 5:
            if exc is None:
                logger.warning(msg)
            else:
                logger.warning("%s: %s", msg, exc)

    def process_data(self, file_path):
        carla = require_carla()

        _info("解析数据...")
        with open(file_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        data_start_point = None
        data_angle = 0.0

        # 1) 选择一个更可靠的 data_start_point / data_angle（用于对齐与旋转）
        auto_anchor = _get_bool_from_env("TP_AUTO_ANCHOR", True)
        if auto_anchor:
            max_candidates = _get_int_from_env("TP_AUTO_ANCHOR_MAX_CANDIDATES", 120)
            sample_points = _get_int_from_env("TP_AUTO_ANCHOR_SAMPLE_POINTS", 80)
            min_keep = _get_int_from_env("TP_AUTO_ANCHOR_MIN_KEEP", 20)

            best = None
            best_keep = -1
            best_mean = float("inf")
            best_angle = 0.0
            best_start = None
            checked = 0

            for line_idx, line in enumerate(lines[: max(0, max_candidates)]):
                tokens = line.replace("\n", " ").split()
                if len(tokens) < 12:
                    continue
                pts = _extract_xy_points_from_tokens(tokens, max_points=sample_points)
                if len(pts) < 6:
                    continue

                cand_start = pts[0]
                cand_angle = _calculate_stable_data_angle(pts)
                rotation_diff = float(self.map_angle) - float(cand_angle)
                fwd_vec_calc = carla.Vector3D(self.entry_fwd.x, self.entry_fwd.y, 0)

                reverse = bool(config.REVERSE_DIRECTION)
                if reverse:
                    rotation_diff += 180.0
                    fwd_vec_calc.x = -fwd_vec_calc.x
                    fwd_vec_calc.y = -fwd_vec_calc.y
                right_vec = carla.Vector3D(fwd_vec_calc.y, -fwd_vec_calc.x, 0)

                rotation_diff += float(config.ROTATION_BIAS_DEG)
                rot_rad = math.radians(rotation_diff)
                cos_a = math.cos(rot_rad)
                sin_a = math.sin(rot_rad)
                off_x = right_vec.x * float(config.LATERAL_OFFSET)
                off_y = right_vec.y * float(config.LATERAL_OFFSET)

                keep = 0
                sum_dist = 0.0
                for (x, y) in pts:
                    rx = (x - cand_start[0]) * float(config.DATA_SCALE)
                    ry = (y - cand_start[1]) * float(config.DATA_SCALE) * float(config.SCALE_Y)
                    rot_x = rx * cos_a - ry * sin_a
                    rot_y = rx * sin_a + ry * cos_a
                    final_x = rot_x + self.entry_loc.x + off_x
                    final_y = rot_y + self.entry_loc.y + off_y
                    rough_loc = carla.Location(final_x, final_y, self.entry_loc.z)

                    try:
                        wp = self.map.get_waypoint(
                            rough_loc,
                            project_to_road=True,
                            lane_type=carla.LaneType.Driving,
                        )
                    except Exception:
                        wp = None
                    if not wp:
                        continue
                    try:
                        d = rough_loc.distance(wp.transform.location)
                    except Exception:
                        continue

                    if d <= max(8.0, float(config.SNAP_THRESHOLD) * 1.5):
                        keep += 1
                        sum_dist += float(d)

                checked += 1
                if keep <= 0:
                    continue
                mean = sum_dist / keep
                if keep > best_keep or (keep == best_keep and mean < best_mean):
                    best_keep = keep
                    best_mean = mean
                    best = line_idx
                    best_angle = cand_angle
                    best_start = cand_start

            if best is not None and best_start is not None and best_keep > 0:
                data_start_point = best_start
                data_angle = best_angle
                lvl = logging.INFO if best_keep >= min_keep else logging.WARNING
                logger.log(
                    lvl,
                    "Auto anchor selected: line=%d checked=%d keep=%d (min_keep=%d) mean_dist=%.3f data_angle=%.3f",
                    best,
                    checked,
                    best_keep,
                    min_keep,
                    best_mean,
                    data_angle,
                )
            else:
                logger.warning(
                    "Auto anchor failed (best_keep=%s checked=%d); falling back to first usable track",
                    best_keep,
                    checked,
                )

        # Fallback：取第一条可用轨迹的首点作为 data_start_point / data_angle
        if not data_start_point:
            for line in lines:
                tokens = line.replace("\n", " ").split()
                if len(tokens) < 12:
                    continue
                pts = _extract_xy_points_from_tokens(tokens)
                if len(pts) >= 2:
                    data_start_point = pts[0]
                    data_angle = _calculate_stable_data_angle(pts)
                    break

        if not data_start_point:
            logger.warning("No usable start point found in data; abort processing")
            return

        rotation_diff = float(self.map_angle) - float(data_angle)
        fwd_vec_calc = carla.Vector3D(self.entry_fwd.x, self.entry_fwd.y, 0)

        reverse = bool(config.REVERSE_DIRECTION)
        if reverse:
            rotation_diff += 180.0
            fwd_vec_calc.x = -fwd_vec_calc.x
            fwd_vec_calc.y = -fwd_vec_calc.y
        right_vec = carla.Vector3D(fwd_vec_calc.y, -fwd_vec_calc.x, 0)

        rotation_diff += float(config.ROTATION_BIAS_DEG)
        rot_rad = math.radians(rotation_diff)
        cos_a = math.cos(rot_rad)
        sin_a = math.sin(rot_rad)
        off_x = right_vec.x * float(config.LATERAL_OFFSET)
        off_y = right_vec.y * float(config.LATERAL_OFFSET)

        # 2) 找最小时间戳
        min_ts = float("inf")
        for line in lines:
            tokens = line.replace("\n", " ").split()
            for i in range(0, len(tokens), 6):
                try:
                    ts = float(tokens[i])
                    if ts < min_ts:
                        min_ts = ts
                except (ValueError, IndexError):
                    continue
        self.global_start_time_raw = min_ts
        logger.info("Data global min timestamp (raw) = %.3f", self.global_start_time_raw)

        # 3) 转换
        temp_tracks = []
        for line_idx, line in enumerate(lines):
            tokens = line.replace("\n", " ").split()
            if len(tokens) < 6:
                continue
            v_type = tokens[5] if len(tokens) >= 6 else "car"

            track = VehicleTrack(vehicle_id=line_idx, type_str=v_type)
            last_valid_v_mps = 0.0
            last_wp = None
            prev_rough_loc = None
            prev_valid_yaw = None
            offset_count = 0
            last_lane_id = None
            used_fallback_wp = 0
            lane_changes = 0

            for i in range(0, len(tokens), 6):
                try:
                    ts_raw = float(tokens[i])
                    x = float(tokens[i + 1])
                    y = float(tokens[i + 2])
                except (ValueError, IndexError) as e:
                    self._log_parse_error(f"轨迹点解析失败 line={line_idx} i={i}", e)
                    continue

                # 速度（统一转为 m/s）
                v_mps = last_valid_v_mps
                if i + 3 < len(tokens) and str(tokens[i + 3]).strip().lower() != "null":
                    try:
                        raw_v = float(tokens[i + 3])
                        v_mps = float(raw_v) * float(config.SPEED_FACTOR)
                        last_valid_v_mps = v_mps
                    except ValueError as e:
                        self._log_parse_error(
                            f"速度解析失败 line={line_idx} i={i} v={tokens[i+3]!r}",
                            e,
                        )
                        v_mps = last_valid_v_mps

                if i + 5 < len(tokens):
                    t = tokens[i + 5]
                    if t and t != v_type:
                        logger.debug(
                            "Track type changed within a line: line=%d base=%r at_i=%d type=%r",
                            line_idx,
                            v_type,
                            i,
                            t,
                        )

                if abs(x) < 1.0:
                    continue

                rel_time = (ts_raw - self.global_start_time_raw) / 1000.0

                rx = (x - data_start_point[0]) * float(config.DATA_SCALE)
                ry = (y - data_start_point[1]) * float(config.DATA_SCALE) * float(config.SCALE_Y)
                rot_x = rx * cos_a - ry * sin_a
                rot_y = rx * sin_a + ry * cos_a
                final_x = rot_x + self.entry_loc.x + off_x
                final_y = rot_y + self.entry_loc.y + off_y
                rough_loc = carla.Location(final_x, final_y, self.entry_loc.z)

                if not is_in_front(rough_loc, self.entry_loc, fwd_vec_calc):
                    continue

                chosen_wp = None
                if last_wp and prev_rough_loc:
                    step = math.hypot(
                        rough_loc.x - prev_rough_loc.x,
                        rough_loc.y - prev_rough_loc.y,
                    )
                    step = max(0.5, min(step, 30.0))
                    cands = last_wp.previous(step) if reverse else last_wp.next(step)
                    if cands:
                        chosen_wp = min(
                            cands,
                            key=lambda w: rough_loc.distance(w.transform.location),
                        )
                        if rough_loc.distance(chosen_wp.transform.location) > float(
                            config.SNAP_THRESHOLD
                        ):
                            chosen_wp = None

                if not chosen_wp:
                    used_fallback_wp += 1
                    wp = self.map.get_waypoint(
                        rough_loc,
                        project_to_road=True,
                        lane_type=carla.LaneType.Driving,
                    )
                    if not wp:
                        self.stats["skipped_no_waypoint"] += 1
                        prev_rough_loc = rough_loc
                        continue
                    if rough_loc.distance(wp.transform.location) > float(config.SNAP_THRESHOLD):
                        prev_rough_loc = rough_loc
                        continue
                    chosen_wp = wp

                if chosen_wp and last_lane_id is not None:
                    try:
                        dist = rough_loc.distance(chosen_wp.transform.location)
                        if dist <= float(config.SNAP_STRICT_DIST) and chosen_wp.lane_id != last_lane_id:
                            lane_candidate = None
                            if chosen_wp.lane_id < last_lane_id:
                                lane_candidate = chosen_wp.get_left_lane()
                            elif chosen_wp.lane_id > last_lane_id:
                                lane_candidate = chosen_wp.get_right_lane()
                            if lane_candidate and lane_candidate.lane_type == carla.LaneType.Driving:
                                if rough_loc.distance(lane_candidate.transform.location) <= dist + 0.5:
                                    chosen_wp = lane_candidate
                    except Exception as e:
                        logger.debug("lane_id keep failed track=%s: %s", track.id, e)

                if chosen_wp and last_wp and bool(config.ENFORCE_SAME_ROAD_ID):
                    try:
                        if chosen_wp.road_id != last_wp.road_id:
                            if rough_loc.distance(chosen_wp.transform.location) > float(
                                config.SNAP_STRICT_DIST
                            ):
                                chosen_wp = None
                    except Exception as e:
                        logger.debug("road_id check failed track=%s: %s", track.id, e)

                if chosen_wp is None:
                    self.stats["skipped_no_waypoint"] += 1
                    prev_rough_loc = rough_loc
                    continue

                if chosen_wp and last_wp:
                    base_offset = rough_loc.distance(chosen_wp.transform.location)
                    offset_count = (
                        offset_count + 1
                        if base_offset >= float(config.SNAP_THRESHOLD)
                        else max(0, offset_count - 1)
                    )
                    if offset_count >= 5:
                        if prev_rough_loc is None:
                            prev_rough_loc = rough_loc
                            continue

                        step2 = math.hypot(
                            rough_loc.x - prev_rough_loc.x,
                            rough_loc.y - prev_rough_loc.y,
                        )
                        step2 = max(0.5, min(step2, 30.0))

                        neighs = []
                        try:
                            left_lane = last_wp.get_left_lane()
                        except Exception as e:
                            logger.debug("get_left_lane failed track=%s: %s", track.id, e)
                            left_lane = None
                        try:
                            right_lane = last_wp.get_right_lane()
                        except Exception as e:
                            logger.debug("get_right_lane failed track=%s: %s", track.id, e)
                            right_lane = None

                        if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                            lcands = left_lane.previous(step2) if reverse else left_lane.next(step2)
                            neighs.append(
                                min(lcands, key=lambda w: rough_loc.distance(w.transform.location))
                                if lcands
                                else left_lane
                            )
                        if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                            rcands = right_lane.previous(step2) if reverse else right_lane.next(step2)
                            neighs.append(
                                min(rcands, key=lambda w: rough_loc.distance(w.transform.location))
                                if rcands
                                else right_lane
                            )

                        if neighs:
                            best_wp = chosen_wp
                            best_cost = base_offset
                            for cand in neighs:
                                cand_dist = rough_loc.distance(cand.transform.location)
                                cost = float(cand_dist) + float(config.LANE_CHANGE_PENALTY_M)
                                if best_wp is None or cost < best_cost:
                                    best_wp = cand
                                    best_cost = cost

                            if best_wp is not None and best_wp is not chosen_wp:
                                if rough_loc.distance(best_wp.transform.location) <= base_offset - float(
                                    config.LANE_SWITCH_MIN_IMPROVEMENT_M
                                ):
                                    chosen_wp = best_wp
                                    offset_count = 0

                if (
                    chosen_wp
                    and rough_loc.distance(chosen_wp.transform.location)
                    <= float(config.SNAP_STRICT_DIST)
                ):
                    offset_count = 0

                if chosen_wp is None:
                    self.stats["skipped_no_waypoint"] += 1
                    prev_rough_loc = rough_loc
                    continue

                valid_loc = carla.Location(
                    chosen_wp.transform.location.x,
                    chosen_wp.transform.location.y,
                    chosen_wp.transform.location.z + 0.2,
                )
                valid_rot = chosen_wp.transform.rotation
                last_wp = chosen_wp
                prev_rough_loc = rough_loc
                try:
                    if last_lane_id is not None and chosen_wp.lane_id != last_lane_id:
                        lane_changes += 1
                    last_lane_id = chosen_wp.lane_id
                except Exception:
                    last_lane_id = None

                valid_rot.yaw += float(config.YAW_CORRECTION)
                if prev_valid_yaw is not None:
                    dy = valid_rot.yaw - prev_valid_yaw
                    if dy > 180:
                        dy -= 360
                    elif dy < -180:
                        dy += 360
                    dy = max(-30.0, min(30.0, dy))
                    valid_rot.yaw = prev_valid_yaw + dy
                prev_valid_yaw = valid_rot.yaw

                track.add_frame(TrackFrame(rel_time, valid_loc, valid_rot, v_mps))

            if len(track.frames) > 0:
                track.sort_frames()
                self.stats["snap_fallback_waypoint"] += used_fallback_wp
                self.stats["parsed_lane_changes"] += lane_changes
                temp_tracks.append(track)

        temp_tracks.sort(key=lambda t: t.start_time)
        temp_tracks = _select_tracks(temp_tracks)

        if _get_bool_from_env("TP_REBASE_TIME", False):
            temp_tracks = _rebase_tracks_to_zero(temp_tracks)
            temp_tracks.sort(key=lambda t: t.start_time)

        temp_tracks = _compress_tracks_spawn_schedule(temp_tracks)

        if config.TRACK_LIMIT is not None:
            temp_tracks = temp_tracks[: config.TRACK_LIMIT]

        for t in temp_tracks:
            try:
                t.next_spawn_time = t.global_start_time()
            except Exception:
                t.next_spawn_time = t.start_time

        if _get_bool_from_env("TP_SPAWN_SCHEDULE_COMPRESS", False):
            preview_n = _get_int_from_env("TP_SPAWN_SCHEDULE_PREVIEW_N", 10)
            for t in temp_tracks[: max(0, preview_n)]:
                try:
                    logger.info(
                        "Spawn schedule preview: track=%s start=%.3f global_start=%.3f next_spawn=%.3f time_offset=%.3f dur=%.3f",
                        getattr(t, "id", "?"),
                        float(getattr(t, "start_time", 0.0)),
                        float(t.global_start_time()),
                        float(getattr(t, "next_spawn_time", 0.0)),
                        float(getattr(t, "time_offset", 0.0)),
                        max(
                            0.0,
                            float(getattr(t, "end_time", 0.0))
                            - float(getattr(t, "start_time", 0.0)),
                        ),
                    )
                except Exception:
                    continue

        # —— StitchAdapter 所需的坐标变换参数缓存 ——
        self._data_start_point = data_start_point
        self._data_angle = data_angle
        self._stitch_cos_a = cos_a
        self._stitch_sin_a = sin_a
        self._stitch_off_x = off_x
        self._stitch_off_y = off_y

        self.pending_tracks = temp_tracks
        _info(f"数据加载完成: {len(temp_tracks)} 条轨迹")

        try:
            all_v = []
            for t in temp_tracks:
                for f in getattr(t, "frames", []):
                    try:
                        all_v.append(float(getattr(f, "v", 0.0)))
                    except Exception:
                        continue
            if all_v:
                all_v_sorted = sorted(all_v)
                p50 = all_v_sorted[int(0.50 * (len(all_v_sorted) - 1))]
                p90 = all_v_sorted[int(0.90 * (len(all_v_sorted) - 1))]
                p99 = all_v_sorted[int(0.99 * (len(all_v_sorted) - 1))]
                logger.info(
                    "Speed stats (m/s, after factor=%.6f, in_kmh=%s): n=%d min=%.3f p50=%.3f p90=%.3f p99=%.3f max=%.3f",
                    float(config.SPEED_FACTOR),
                    str(config.DATA_SPEED_IN_KMH),
                    len(all_v),
                    float(min(all_v)),
                    float(p50),
                    float(p90),
                    float(p99),
                    float(max(all_v)),
                )
        except Exception as e:
            logger.debug("Speed stats collection failed: %s", e)
        if temp_tracks:
            try:
                t0 = temp_tracks[0]
                _info(
                    "Track[0] summary: "
                    f"frames={len(t0.frames)} start={t0.start_time:.3f}s end={t0.end_time:.3f}s "
                    f"duration={max(0.0, t0.end_time - t0.start_time):.3f}s"
                )
            except Exception as e:
                logger.debug("Track summary print failed: %s", e)

        def _pending_key(t):
            v = getattr(t, "next_spawn_time", None)
            if v is not None:
                return float(v)
            try:
                return float(t.global_start_time())
            except Exception:
                return float(getattr(t, "start_time", 0.0))

        self.pending_tracks.sort(key=_pending_key)

    def get_blueprint(self, type_str):
        bp_name = "vehicle.audi.tt"
        if "truck" in type_str:
            bp_name = "vehicle.carlamotors.carlacola"
        elif "bus" in type_str:
            bp_name = "vehicle.mitsubishi.fusorosa"
        bp = self.bp_lib.find(bp_name)
        if bp.has_attribute("color"):
            c = f"{random.randint(0,255)},{random.randint(0,255)},{random.randint(0,255)}"
            bp.set_attribute("color", c)
        return bp

    def _is_spawn_clear(self, loc):
        for active_t in self.active_tracks:
            if active_t.actor:
                act_loc = active_t.actor.get_location()
                if act_loc.distance(loc) < float(config.SPAWN_OVERLAP_BLOCK_DIST_M):
                    return False
        return True

    def _build_spawn_transforms(self, base_frame):
        carla = require_carla()

        candidates = []

        def add_loc(loc):
            candidates.append(
                carla.Transform(
                    loc + carla.Location(z=float(config.SPAWN_Z_OFFSET_M)),
                    base_frame.rot,
                )
            )

        add_loc(base_frame.loc)

        try:
            wp0 = self.map.get_waypoint(
                base_frame.loc,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
        except Exception as e:
            logger.debug("build_spawn_transforms: get_waypoint failed: %s", e)
            return candidates[: max(1, int(config.SPAWN_MAX_CANDIDATES))]

        if wp0 is None:
            return candidates[: max(1, int(config.SPAWN_MAX_CANDIDATES))]

        offsets = [d for d in config.SPAWN_CANDIDATE_OFFSETS_M if float(d) > 1e-6]
        for d in offsets:
            try:
                prev_wps = wp0.previous(float(d))
            except Exception:
                prev_wps = []
            try:
                next_wps = wp0.next(float(d))
            except Exception:
                next_wps = []

            for wps in (prev_wps, next_wps):
                if not wps:
                    continue
                wp = wps[0]
                add_loc(wp.transform.location)

                if bool(config.SPAWN_TRY_ADJACENT_LANES):
                    neighs = []
                    try:
                        neighs.append(wp.get_left_lane())
                    except Exception:
                        neighs.append(None)
                    try:
                        neighs.append(wp.get_right_lane())
                    except Exception:
                        neighs.append(None)
                    for neigh in neighs:
                        if neigh and neigh.lane_type == carla.LaneType.Driving:
                            add_loc(neigh.transform.location)

            if len(candidates) >= int(config.SPAWN_MAX_CANDIDATES):
                break

        return candidates[: max(1, int(config.SPAWN_MAX_CANDIDATES))]

    def _schedule_spawn_retry(self, track, reason):
        backoff_enabled = _get_bool_from_env("TP_SPAWN_BACKOFF", True)
        backoff_base = _get_float_from_env(
            "TP_SPAWN_BACKOFF_BASE_SECONDS",
            float(config.SPAWN_RETRY_DELAY_SECONDS),
        )
        backoff_max = _get_float_from_env(
            "TP_SPAWN_BACKOFF_MAX_SECONDS",
            max(2.0, float(config.SPAWN_DEFER_SECONDS)),
        )
        backoff_jitter = _get_float_from_env("TP_SPAWN_BACKOFF_JITTER_SECONDS", 0.15)
        backoff_growth = _get_float_from_env("TP_SPAWN_BACKOFF_GROWTH", 1.5)

        try:
            global_end = float(track.global_end_time())
        except Exception:
            global_end = float(track.end_time)

        remaining = global_end - float(self.current_traj_time)
        min_delay = max(0.0, float(config.SPAWN_RETRY_MIN_DELAY_SECONDS))
        min_remaining = max(0.0, float(config.SPAWN_RETRY_MIN_REMAINING_SECONDS))

        if remaining <= (min_remaining + min_delay):
            self.stats["spawn_abandoned_near_end"] += 1
            logger.info(
                "Spawn abandoned (near end): track=%s reason=%s traj_time=%.3fs end=%.3fs remaining=%.3fs",
                track.id,
                reason,
                self.current_traj_time,
                global_end,
                remaining,
            )
            return False

        base_delay = max(0.0, float(config.SPAWN_RETRY_DELAY_SECONDS))
        if backoff_enabled:
            try:
                attempt = max(1, int(getattr(track, "spawn_attempts", 0)))
            except Exception:
                attempt = 1

            reason_min = 0.0
            if str(reason).strip().lower() in ("overlap", "max_active"):
                reason_min = float(config.SPAWN_DEFER_SECONDS)

            delay0 = float(backoff_base) * (
                float(backoff_growth) ** float(max(0, attempt - 1))
            )
            delay0 = min(
                float(backoff_max),
                max(
                    float(base_delay),
                    float(reason_min),
                    float(delay0),
                ),
            )

            j = 0.0
            if float(backoff_jitter) > 0.0:
                try:
                    j = random.uniform(
                        -abs(float(backoff_jitter)),
                        abs(float(backoff_jitter)),
                    )
                except Exception:
                    j = 0.0
            base_delay = max(0.0, delay0 + j)

        max_delay = max(0.0, remaining - min_remaining)
        delay = min(float(base_delay), float(max_delay))
        delay = max(float(min_delay), float(delay))
        track.next_spawn_time = self.current_traj_time + delay
        self._requeue_track(track)
        return True

    def apply_vehicle_control(self, vehicle, current_frame, lookahead_frame, dt, track_id=None):
        carla = require_carla()

        trans = vehicle.get_transform()
        loc = trans.location

        track = None
        if track_id is not None:
            try:
                for t in self.active_tracks:
                    if t.id == track_id:
                        track = t
                        break
            except Exception:
                track = None

        if track is None:
            class _Dummy:
                teleport_over_ticks: int
                teleport_cooldown_ticks: int
                cmd_vx: Optional[float]
                cmd_vy: Optional[float]
                cmd_speed: Optional[float]
                catchup_err_f: float
                cmd_wz: float

                def __init__(self) -> None:
                    self.teleport_over_ticks = 0
                    self.teleport_cooldown_ticks = 0
                    self.cmd_vx = None
                    self.cmd_vy = None
                    self.cmd_speed = None
                    self.catchup_err_f = 0.0
                    self.cmd_wz = 0.0

            track = _Dummy()

        mode = (os.getenv("TP_REPLAY_CONTROL_MODE") or str(config.REPLAY_CONTROL_MODE)).strip().lower()
        if mode not in ("physics_follow", "kinematic"):
            mode = "physics_follow"

        dist_vec = carla.Vector3D(
            current_frame.loc.x - loc.x,
            current_frame.loc.y - loc.y,
            0,
        )
        dist_err = math.hypot(dist_vec.x, dist_vec.y)

        if mode == "kinematic":
            try:
                vehicle.set_simulate_physics(False)
            except RuntimeError:
                pass
            vehicle.set_transform(carla.Transform(current_frame.loc, current_frame.rot))
            try:
                vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
            except RuntimeError:
                pass
            return

        try:
            yaw_err = float(
                abs(
                    (current_frame.rot.yaw - trans.rotation.yaw + 180.0) % 360.0
                    - 180.0
                )
            )
        except Exception:
            yaw_err = 0.0

        if getattr(track, "teleport_cooldown_ticks", 0) > 0:
            track.teleport_cooldown_ticks -= 1

        if dist_err > float(config.PHYSICS_SYNC_DIST) or yaw_err > float(
            config.TELEPORT_YAW_GATE_DEG
        ):
            if getattr(track, "teleport_cooldown_ticks", 0) <= 0:
                track.teleport_over_ticks = int(getattr(track, "teleport_over_ticks", 0)) + 1
        else:
            track.teleport_over_ticks = 0

        cons = _get_int_from_env("TP_TELEPORT_CONSEC_TICKS", int(config.TELEPORT_CONSECUTIVE_TICKS))
        cooldown = _get_int_from_env("TP_TELEPORT_COOLDOWN_TICKS", int(config.TELEPORT_COOLDOWN_TICKS))

        if (
            getattr(track, "teleport_cooldown_ticks", 0) <= 0
            and getattr(track, "teleport_over_ticks", 0) >= max(1, cons)
        ):
            track.teleport_over_ticks = 0
            track.teleport_cooldown_ticks = max(0, int(cooldown))

            target_v_mag = max(0.0, float(current_frame.v)) * float(config.PLAYBACK_SPEED)
            yaw_rad = math.radians(float(current_frame.rot.yaw))
            init_vel = carla.Vector3D(
                math.cos(yaw_rad) * target_v_mag,
                math.sin(yaw_rad) * target_v_mag,
                0,
            )
            vehicle.set_transform(carla.Transform(current_frame.loc, current_frame.rot))
            vehicle.set_target_velocity(init_vel)
            vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
            track.cmd_speed = float(target_v_mag)
            track.cmd_wz = 0.0
            self.stats["teleports"] += 1

            tp = self.stats["teleports"]
            if tp <= 10 or tp % 50 == 0:
                logger.warning(
                    "Teleport sync: traj_time=%.3fs track=%s dist_err=%.3fm yaw_err=%.1fdeg (dist_th=%.3f cons=%d cooldown=%d)",
                    self.current_traj_time,
                    str(track_id) if track_id is not None else "?",
                    dist_err,
                    yaw_err,
                    float(config.PHYSICS_SYNC_DIST),
                    cons,
                    cooldown,
                )
            return

        target_speed = max(0.0, float(current_frame.v) * float(config.PLAYBACK_SPEED))

        alpha_e = _get_float_from_env("TP_CATCH_UP_ERROR_ALPHA", float(config.CATCH_UP_ERROR_ALPHA))
        db_m = _get_float_from_env("TP_CATCH_UP_DEADBAND_M", float(config.CATCH_UP_DEADBAND_M))
        prev_e = float(getattr(track, "catchup_err_f", 0.0))
        e_f = (1.0 - float(alpha_e)) * prev_e + float(alpha_e) * float(dist_err)
        track.catchup_err_f = e_f
        e_used = max(0.0, float(e_f) - float(db_m))
        catch_up = min(
            e_used * float(config.CATCH_UP_GAIN),
            max(0.0, target_speed) * 0.5,
        )
        raw_speed = target_speed + catch_up

        if getattr(track, "cmd_speed", None) is None:
            track.cmd_speed = target_speed
        prev_speed = float(getattr(track, "cmd_speed", 0.0))
        max_acc = max(0.1, _get_float_from_env("TP_MAX_ACCEL_MPS2", float(config.MAX_ACCEL_MPS2)))
        max_dec = max(0.1, _get_float_from_env("TP_MAX_DECEL_MPS2", float(config.MAX_DECEL_MPS2)))
        dts = max(1e-3, float(dt) if dt is not None else 1e-3)
        lo_sp = prev_speed - max_dec * dts
        hi_sp = prev_speed + max_acc * dts
        speed_limited = float(min(max(float(raw_speed), float(lo_sp)), float(hi_sp)))
        alpha_sp = _get_float_from_env("TP_CMD_LPF_ALPHA_SPEED", float(config.CMD_LPF_ALPHA_SPEED))
        track.cmd_speed = (1.0 - float(alpha_sp)) * prev_speed + float(alpha_sp) * speed_limited

        dx_la = float(lookahead_frame.loc.x - loc.x)
        dy_la = float(lookahead_frame.loc.y - loc.y)
        dir_norm = float(math.hypot(dx_la, dy_la))
        if dir_norm > 0.1:
            dir_x = dx_la / dir_norm
            dir_y = dy_la / dir_norm
        else:
            fwd = trans.get_forward_vector()
            dir_x = float(fwd.x)
            dir_y = float(fwd.y)

        yaw_for_v = math.radians(float(trans.rotation.yaw))
        raw_vx = math.cos(yaw_for_v) * float(track.cmd_speed)
        raw_vy = math.sin(yaw_for_v) * float(track.cmd_speed)
        alpha_v = _get_float_from_env("TP_CMD_LPF_ALPHA_V", float(config.CMD_LPF_ALPHA_V))
        if getattr(track, "cmd_vx", None) is None:
            track.cmd_vx = raw_vx
            track.cmd_vy = raw_vy
        else:
            prev_vx = track.cmd_vx
            prev_vy = track.cmd_vy
            if prev_vx is None or prev_vy is None:
                prev_vx = raw_vx
                prev_vy = raw_vy
            track.cmd_vx = (1.0 - float(alpha_v)) * float(prev_vx) + float(alpha_v) * float(raw_vx)
            track.cmd_vy = (1.0 - float(alpha_v)) * float(prev_vy) + float(alpha_v) * float(raw_vy)

        cmd_vx_val = track.cmd_vx
        cmd_vy_val = track.cmd_vy
        if cmd_vx_val is None or cmd_vy_val is None:
            cmd_vx_val = raw_vx
            cmd_vy_val = raw_vy

        vehicle.set_target_velocity(
            carla.Vector3D(
                float(cmd_vx_val),
                float(cmd_vy_val),
                vehicle.get_velocity().z,
            )
        )

        current_yaw = math.radians(trans.rotation.yaw)
        target_yaw = math.atan2(float(dir_y), float(dir_x))
        diff_yaw = target_yaw - current_yaw
        while diff_yaw > math.pi:
            diff_yaw -= 2 * math.pi
        while diff_yaw < -math.pi:
            diff_yaw += 2 * math.pi

        target_ang_vel_z = float(diff_yaw) * 2.0
        max_wz = _get_float_from_env("TP_MAX_YAW_RATE_RAD_S", float(config.MAX_YAW_RATE_RAD_S))
        max_aw = _get_float_from_env("TP_MAX_YAW_ACCEL_RAD_S2", float(config.MAX_YAW_ACCEL_RAD_S2))
        target_ang_vel_z = float(
            min(max(float(target_ang_vel_z), -float(max_wz)), float(max_wz))
        )

        prev_wz = float(getattr(track, "cmd_wz", 0.0))
        if dt and float(dt) > 1e-6:
            lo = prev_wz - float(max_aw) * float(dt)
            hi = prev_wz + float(max_aw) * float(dt)
            target_ang_vel_z = float(
                min(max(float(target_ang_vel_z), float(lo)), float(hi))
            )

        alpha_w = _get_float_from_env("TP_CMD_LPF_ALPHA_W", float(config.CMD_LPF_ALPHA_W))
        track.cmd_wz = (1.0 - float(alpha_w)) * prev_wz + float(alpha_w) * float(target_ang_vel_z)

        vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, float(track.cmd_wz)))

        steer_cmd = float(min(max(float(diff_yaw) * 1.5, -1.0), 1.0))
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0,
                brake=0,
                steer=steer_cmd,
                manual_gear_shift=False,
            )
        )

    def _try_disable_collision(self, actor):
        carla = require_carla()

        if actor is None:
            return False

        if hasattr(actor, "set_collision_enabled"):
            try:
                actor.set_collision_enabled(False)
                return True
            except TypeError:
                pass
            except RuntimeError as e:
                logger.debug("set_collision_enabled(False) failed: %s", e)

            ce = getattr(carla, "CollisionEnabled", None)
            if ce is not None:
                for name in (
                    "NoCollision",
                    "NO_COLLISION",
                    "Disabled",
                    "DISABLED",
                ):
                    if hasattr(ce, name):
                        try:
                            actor.set_collision_enabled(getattr(ce, name))
                            return True
                        except (TypeError, RuntimeError) as e:
                            logger.debug("set_collision_enabled(%s) failed: %s", name, e)

        if hasattr(actor, "set_collisions"):
            try:
                actor.set_collisions(False)
                return True
            except (TypeError, RuntimeError) as e:
                logger.debug("set_collisions(False) failed: %s", e)

        return False

    def _teleport_actor_away(self, actor):
        carla = require_carla()

        if actor is None:
            return
        try:
            t = actor.get_transform()
            t.location.z = float(config.FINISH_TELEPORT_Z)
            actor.set_transform(t)
            actor.set_target_velocity(carla.Vector3D(0, 0, 0))
            actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        except RuntimeError as e:
            logger.debug("teleport_actor_away failed: %s", e)

    def _handle_track_finished_once(self, track):
        carla = require_carla()

        if track is None or track.actor is None or track.finished:
            return

        behavior = str(config.FINISH_BEHAVIOR).strip().lower()
        actor = track.actor

        try:
            actor.set_target_velocity(carla.Vector3D(0, 0, 0))
            actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        except RuntimeError:
            pass

        if behavior == "destroy":
            try:
                actor.destroy()
            except RuntimeError as e:
                logger.debug("destroy finished actor failed: %s", e)
            track.actor = None
            track.finished = True
            self.stats["finished_tracks"] += 1
            track.finish_timer = float(config.DESTROY_DELAY) + 1.0
            logger.info(
                "Track finished: track=%s traj_time=%.3fs behavior=destroy",
                track.id,
                self.current_traj_time,
            )
            return

        try:
            actor.set_simulate_physics(False)
        except RuntimeError:
            pass

        no_collision = False
        if behavior == "ghost":
            no_collision = self._try_disable_collision(actor)

        if behavior == "teleport_away" or (behavior == "ghost" and not no_collision):
            self._teleport_actor_away(actor)

        track.finished = True
        self.stats["finished_tracks"] += 1
        logger.info(
            "Track finished: track=%s traj_time=%.3fs behavior=%s no_collision=%s destroy_delay=%.3fs",
            track.id,
            self.current_traj_time,
            behavior,
            no_collision,
            float(config.DESTROY_DELAY),
        )

    def tick(self, delta_time):
        carla = require_carla()

        self.current_traj_time += float(delta_time) * float(config.PLAYBACK_SPEED)

        while (
            self.pending_tracks
            and self.pending_tracks[0].next_spawn_time <= self.current_traj_time
        ):
            if (
                config.MAX_ACTIVE_VEHICLES is not None
                and len(self.active_tracks) >= int(config.MAX_ACTIVE_VEHICLES)
            ):
                break

            track = self.pending_tracks.pop(0)

            try:
                global_end = track.global_end_time()
            except Exception:
                global_end = track.end_time

            if self.current_traj_time > float(global_end):
                self.stats["spawn_skipped_past_end"] += 1
                logger.info(
                    "Skip spawn (past end_time): track=%s traj_time=%.3fs end=%.3fs",
                    track.id,
                    self.current_traj_time,
                    float(global_end),
                )
                continue

            try:
                spawn_time = float(track.global_start_time())
            except Exception:
                spawn_time = float(getattr(track, "start_time", 0.0))
            state = track.get_state_at_time(spawn_time)
            if not isinstance(state, TrackFrame):
                self.stats["spawn_deferred_no_state"] += 1
                logger.info(
                    "Spawn deferred (no state yet): track=%s next_spawn=%.3fs",
                    track.id,
                    self.current_traj_time,
                )
                self._schedule_spawn_retry(track, reason="no_state")
                continue

            if not self._is_spawn_clear(state.loc):
                self.stats["spawn_blocked_overlap"] += 1
                logger.info(
                    "Spawn blocked (overlap), retry: track=%s next_spawn=%.3fs",
                    track.id,
                    self.current_traj_time,
                )
                self._schedule_spawn_retry(track, reason="overlap")
                continue

            bp = self.get_blueprint(track.type_str)
            track.spawn_attempts += 1
            actor = None

            spawn_transforms = self._build_spawn_transforms(state)
            self.stats["spawn_candidates_tried"] += len(spawn_transforms)

            spawn_exc = None
            for spawn_trans in spawn_transforms:
                if not self._is_spawn_clear(spawn_trans.location):
                    continue
                try:
                    if hasattr(self.world, "try_spawn_actor"):
                        actor = self.world.try_spawn_actor(bp, spawn_trans)
                    else:
                        actor = self.world.spawn_actor(bp, spawn_trans)
                except Exception as e:
                    spawn_exc = e
                    actor = None
                if actor is not None:
                    break

            if actor is None:
                self.stats["spawn_failures"] += 1
                if spawn_exc is not None:
                    self.stats["spawn_fail_exception"] += 1
                    logger.warning(
                        "Spawn failed (exception) track=%s type=%r attempt=%d: %s",
                        track.id,
                        track.type_str,
                        track.spawn_attempts,
                        spawn_exc,
                    )
                else:
                    self.stats["spawn_fail_none"] += 1
                if track.spawn_attempts < max(1, int(config.SPAWN_MAX_ATTEMPTS)):
                    logger.info(
                        "Spawn retry scheduled: track=%s attempt=%d next_spawn=%.3fs pending=%d",
                        track.id,
                        track.spawn_attempts,
                        self.current_traj_time,
                        len(self.pending_tracks),
                    )
                    self._schedule_spawn_retry(track, reason="spawn_fail")
                    continue

                logger.warning(
                    "Spawn returned None track=%s type=%r attempts=%d",
                    track.id,
                    track.type_str,
                    track.spawn_attempts,
                )
                self.stats["spawn_abandoned_max_attempts"] += 1
                continue

            track.actor = actor
            self.stats["spawn_success"] += 1
            track.actor.set_simulate_physics(bool(config.ENABLE_PHYSICS))
            track.just_spawned = True

            logger.info(
                "Spawned: track=%s type=%s attempts=%d traj_time=%.3fs loc=(%.2f,%.2f,%.2f)",
                track.id,
                track.type_str,
                track.spawn_attempts,
                self.current_traj_time,
                state.loc.x,
                state.loc.y,
                state.loc.z,
            )

            init_v = float(state.v) * float(config.PLAYBACK_SPEED)
            yaw_rad = math.radians(float(state.rot.yaw))
            track.actor.set_target_velocity(
                carla.Vector3D(
                    math.cos(yaw_rad) * init_v,
                    math.sin(yaw_rad) * init_v,
                    0,
                )
            )
            track.cmd_speed = float(init_v)
            track.cmd_wz = 0.0
            track.catchup_err_f = 0.0

            track.spawned = True
            self.active_tracks.append(track)

        if len(self.pending_tracks) > 1:
            try:
                self.pending_tracks.sort(
                    key=lambda t: float(getattr(t, "next_spawn_time", 0.0))
                )
            except Exception:
                pass

        for i in range(len(self.active_tracks) - 1, -1, -1):
            track = self.active_tracks[i]
            current_state = track.get_state_at_time(self.current_traj_time)

            if isinstance(current_state, TrackFrame):
                current_speed = max(0.0, float(current_state.v))
                adaptive_lookahead = float(config.LOOKAHEAD_TIME) * (
                    1.0 + float(config.LOOKAHEAD_SPEED_GAIN) * current_speed
                )
                adaptive_lookahead = max(0.4, min(adaptive_lookahead, 2.0))
                lookahead_state = track.get_state_at_time(
                    self.current_traj_time + adaptive_lookahead
                )
            else:
                lookahead_state = track.get_state_at_time(
                    self.current_traj_time + float(config.LOOKAHEAD_TIME)
                )

            if current_state == "FINISHED":
                self._handle_track_finished_once(track)

                track.finish_timer += float(delta_time)
                if track.finish_timer > float(config.DESTROY_DELAY):
                    if track.actor:
                        track.actor.destroy()
                        track.actor = None
                    logger.info("Track destroyed: track=%s", track.id)
                    self.stats["destroyed_tracks"] += 1
                    self.active_tracks.pop(i)
                    self._maybe_extend_replay(track)

            elif isinstance(current_state, TrackFrame):
                if track.actor:
                    if track.just_spawned:
                        track.just_spawned = False
                        continue

                    target_lookahead = lookahead_state
                    if not isinstance(target_lookahead, TrackFrame):
                        target_lookahead = current_state

                    self.apply_vehicle_control(
                        track.actor,
                        current_state,
                        target_lookahead,
                        delta_time,
                        track_id=track.id,
                    )

        return len(self.active_tracks)

    def _requeue_track(self, track):
        inserted = False
        for j in range(len(self.pending_tracks)):
            if self.pending_tracks[j].next_spawn_time > track.next_spawn_time:
                self.pending_tracks.insert(j, track)
                inserted = True
                break
        if not inserted:
            self.pending_tracks.append(track)

    def _maybe_extend_replay(self, track):
        if not self.extend_replay:
            return False
        if not self.extend_until_s or self.extend_until_s <= 0.0:
            return False
        if self.current_traj_time >= self.extend_until_s:
            return False
        if track is None:
            return False
        if track.loop_count >= max(0, int(self.extend_max_loops)):
            return False
        if not track.frames:
            return False

        gap = _get_float_from_env("TP_EXTEND_GAP_SECONDS", 0.5)
        new_start_global = self.current_traj_time + max(0.0, float(gap))
        track.loop_count += 1
        track.time_offset = new_start_global - float(track.start_time)
        track.next_spawn_time = track.global_start_time()
        track.spawn_attempts = 0
        track.actor = None
        track.spawned = False
        track.finished = False
        track.finish_timer = 0.0
        track.just_spawned = False

        self._requeue_track(track)
        self.stats["replays_extended"] += 1
        logger.info(
            "Replay-extend: re-queued track=%s loop=%d next_spawn=%.3fs start=%.3fs end=%.3fs",
            track.id,
            track.loop_count,
            track.next_spawn_time,
            track.global_start_time(),
            track.global_end_time(),
        )
        return True
