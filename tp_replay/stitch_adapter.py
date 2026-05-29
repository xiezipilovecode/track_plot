"""拼接轨迹适配器。

将拼接引擎输出的 stitched JSON 转换为 tp_replay 可用的 VehicleTrack 列表。
转换过程中复用 engine.py 的坐标变换参数（cos_a, sin_a, off_x, off_y, data_start_point）。
"""

from __future__ import annotations

import json
import logging
import math
import os
from typing import List, Optional, Tuple

from . import config
from .carla_compat import require_carla
from .models import TrackFrame, VehicleTrack

logger = logging.getLogger(__name__)


class StitchAdapter:
    """拼接轨迹数据 → VehicleTrack 适配器。

    为 TM 自动驾驶模式构建轻量级 VehicleTrack：
    - frames[0]: 生成状态（首帧位置 + 朝向 + 速度）
    - speed_profile: 速度曲线列表 [(相对秒, m/s), ...]
    """

    def __init__(self, engine):
        """engine: 已初始化的 ReplayEngine 实例（需先运行 process_data 以缓存变换参数）"""
        self.engine = engine
        self.carla = require_carla()
        self._off_x = 0.0
        self._off_y = 0.0

    # ── 公开接口 ──────────────────────────────────────────

    def load_and_convert(
        self,
        json_path: str,
        min_quality: float = 0.5,
        min_cameras: int = 2,
        max_tracks: Optional[int] = None,
    ) -> List[VehicleTrack]:
        """主入口：加载拼接 JSON → 过滤 → 坐标变换 → VehicleTrack 列表。"""
        raw = self._load_json(json_path)
        logger.info("Loaded %d stitched trajectories from %s", len(raw), json_path)

        filtered = self._filter(raw, min_quality, min_cameras, max_tracks)
        logger.info("Filtered to %d (min_quality=%.1f, min_cameras=%d)", len(filtered), min_quality, min_cameras)

        global_min_ts = self._global_min_ts(filtered)
        self.engine.global_start_time_raw = global_min_ts

        cos_a, sin_a = self._transform_params()
        data_start = self._data_start()

        tracks = []
        for i, st in enumerate(filtered):
            vt = self._to_vehicle_track(st, i, global_min_ts, data_start, cos_a, sin_a)
            if vt and vt.frames:
                tracks.append(vt)

        logger.info("Built %d VehicleTracks", len(tracks))
        return tracks

    # ── 内部方法 ──────────────────────────────────────────

    def _load_json(self, path: str) -> List[dict]:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            return data
        if isinstance(data, dict):
            for key in ("trajectories", "stitched_trajectories", "data"):
                if key in data:
                    return data[key]
            return [data]
        return []

    def _filter(self, tracks: List[dict], min_q: float, min_c: int, max_n: Optional[int]) -> List[dict]:
        result = []
        for t in tracks:
            if t.get("quality_score", 0) < min_q:
                continue
            if t.get("camera_count", 0) < min_c:
                continue
            result.append(t)
            if max_n and len(result) >= max_n:
                break
        return result

    def _global_min_ts(self, tracks: List[dict]) -> float:
        m = float("inf")
        for t in tracks:
            for n in t.get("nodes", []):
                ts = n.get("timestamp")
                if ts and ts < m:
                    m = ts
        return float(m) if not math.isinf(m) else 0.0

    def _transform_params(self) -> Tuple[float, float]:
        if hasattr(self.engine, "_stitch_cos_a"):
            self._off_x = getattr(self.engine, "_stitch_off_x", 0.0)
            self._off_y = getattr(self.engine, "_stitch_off_y", 0.0)
            return self.engine._stitch_cos_a, self.engine._stitch_sin_a
        logger.warning("No cached transform params; using identity")
        return 1.0, 0.0

    def _data_start(self) -> Tuple[float, float]:
        if hasattr(self.engine, "_data_start_point"):
            return self.engine._data_start_point
        return (0.0, 0.0)

    def _to_world(self, x: float, y: float, ds: tuple, ca: float, sa: float):
        """节点坐标 → CARLA Location。"""
        rx = (x - ds[0]) * float(config.DATA_SCALE)
        ry = (y - ds[1]) * float(config.DATA_SCALE) * float(config.SCALE_Y)
        rot_x = rx * ca - ry * sa
        rot_y = rx * sa + ry * ca
        return self.carla.Location(
            rot_x + self.engine.entry_loc.x + self._off_x,
            rot_y + self.engine.entry_loc.y + self._off_y,
            self.engine.entry_loc.z,
        )

    def _snap_to_road(self, loc):
        """车道吸附。"""
        try:
            wp = self.engine.map.get_waypoint(loc, project_to_road=True,
                                              lane_type=self.carla.LaneType.Driving)
            if wp:
                return wp.transform.location, wp.transform.rotation
        except Exception:
            pass
        return loc, self.carla.Rotation()

    def _to_vehicle_track(self, st: dict, tid: int, gts: float,
                          ds: tuple, ca: float, sa: float) -> Optional[VehicleTrack]:
        """单条拼接轨迹 → VehicleTrack（仅 spawn 帧 + 速度曲线，给 TM autopilot 用）。"""
        nodes = st.get("nodes", [])
        valid_nodes = [n for n in nodes if not n.get("interpolated", False) and n.get("y") is not None]
        if not valid_nodes:
            return None

        vtype = st.get("vehicle_type", "car")
        track = VehicleTrack(vehicle_id=f"ST_{tid:06d}", type_str=vtype)
        track.speed_profile = []  # [(rel_seconds, speed_mps), ...]
        track.stitch_quality = st.get("quality_score", 0)
        track.stitch_cameras = st.get("camera_count", 0)

        first_frame_set = False
        last_v_mps = 0.0

        for node in nodes:
            ts = node.get("timestamp")
            if ts is None:
                continue

            rel_time = (float(ts) - gts) / 1000.0
            y = node.get("y", 0.0)

            x = node.get("x")
            if x is None or abs(float(x)) < 1.0:
                x = ds[0]

            loc = self._to_world(float(x), float(y), ds, ca, sa)
            snap_loc, snap_rot = self._snap_to_road(loc)

            speed_raw = node.get("speed")
            if speed_raw is not None:
                try:
                    v_mps = float(speed_raw) * float(config.SPEED_FACTOR)
                    last_v_mps = v_mps
                except (ValueError, TypeError):
                    v_mps = last_v_mps
            else:
                v_mps = last_v_mps

            if not first_frame_set and not node.get("interpolated", False):
                frame = TrackFrame(ts=rel_time, loc=snap_loc, rot=snap_rot, v=v_mps)
                track.add_frame(frame)
                first_frame_set = True

            track.speed_profile.append((rel_time, v_mps))

        if not track.frames:
            for node in nodes:
                if not node.get("interpolated", False):
                    ts = node.get("timestamp", gts)
                    rel_time = (float(ts) - gts) / 1000.0
                    y = node.get("y", 0.0)
                    x = node.get("x") or ds[0]
                    loc = self._to_world(float(x), float(y), ds, ca, sa)
                    snap_loc, snap_rot = self._snap_to_road(loc)
                    frame = TrackFrame(ts=rel_time, loc=snap_loc, rot=snap_rot, v=last_v_mps)
                    track.add_frame(frame)
                    break

        if track.speed_profile:
            track.end_time = track.speed_profile[-1][0]

        return track if track.frames else None

    # ── 全帧模式（供引擎 kinematic 回放使用） ──────────────────

    def load_for_kinematic(
        self,
        json_path: str,
        min_quality: float = 0.5,
        min_cameras: int = 2,
        max_tracks: Optional[int] = None,
    ) -> List[VehicleTrack]:
        """加载拼接 JSON，构建全帧 VehicleTrack（每节点一个 TrackFrame）。

        供引擎的 kinematic 模式使用——每个节点都被路点吸附后作为 TrackFrame，
        引擎的 _update_track 会按时间插值驱动车辆平滑行驶。
        """
        raw = self._load_json(json_path)
        logger.info("Loaded %d stitched trajectories", len(raw))
        filtered = self._filter(raw, min_quality, min_cameras, max_tracks)
        logger.info("Filtered to %d", len(filtered))

        global_min_ts = self._global_min_ts(filtered)
        self.engine.global_start_time_raw = global_min_ts

        cos_a, sin_a = self._transform_params()
        data_start = self._data_start()

        tracks = []
        for i, st_data in enumerate(filtered):
            vt = self._to_full_vehicle_track(st_data, i, global_min_ts, data_start, cos_a, sin_a)
            if vt and vt.frames:
                tracks.append(vt)

        logger.info("Built %d full-frame VehicleTracks for kinematic replay", len(tracks))
        return tracks

    def _to_full_vehicle_track(self, st: dict, tid: int, gts: float,
                                ds: tuple, ca: float, sa: float) -> Optional[VehicleTrack]:
        """单条拼接轨迹 → 全帧 VehicleTrack（每个节点都是 TrackFrame）。

        引擎的 kinematic 模式会在相邻帧间插值，实现平滑行驶。
        """
        nodes = st.get("nodes", [])
        if len(nodes) < 2:
            return None

        vtype = st.get("vehicle_type", "car")
        track = VehicleTrack(vehicle_id=f"ST_{tid:06d}", type_str=vtype)
        track.stitch_quality = st.get("quality_score", 0)
        track.stitch_cameras = st.get("camera_count", 0)
        last_v_mps = 0.0

        for node in nodes:
            ts = node.get("timestamp")
            if ts is None:
                continue
            rel_time = (float(ts) - gts) / 1000.0

            y = node.get("y", 0.0)
            x = node.get("x")
            if x is None or abs(float(x)) < 1.0:
                x = ds[0]

            loc = self._to_world(float(x), float(y), ds, ca, sa)
            snap_loc, snap_rot = self._snap_to_road(loc)

            speed_raw = node.get("speed")
            if speed_raw is not None:
                try:
                    v_mps = float(speed_raw) * float(config.SPEED_FACTOR)
                    last_v_mps = v_mps
                except (ValueError, TypeError):
                    v_mps = last_v_mps
            else:
                v_mps = last_v_mps

            frame = TrackFrame(ts=rel_time, loc=snap_loc, rot=snap_rot, v=v_mps)
            track.add_frame(frame)

        return track if len(track.frames) >= 2 else None
