"""TM 自动驾驶轨迹回放编排器。

使用 CARLA Traffic Manager 进行自动驾驶（AI 转向 + 碰撞避免），
而非逐帧 kinematic teleport。每辆车从拼接轨迹的生成位置出发，
TM 根据轨迹速度曲线动态调整目标速度。
"""

from __future__ import annotations

import logging
import random
from typing import List, Optional

from . import config
from .carla_compat import require_carla
from .models import VehicleTrack

logger = logging.getLogger(__name__)


def _info(msg: str) -> None:
    print(msg)
    logger.info(msg)


class StitchAutopilot:
    """TM 自动驾驶编排器。

    每 tick：
    1. 检查是否有到期车辆待生成 → spawn + 启用 autopilot
    2. 周期性更新活跃车辆 TM 目标速度（来自轨迹速度曲线）
    3. 销毁已完成行程的车辆
    """

    def __init__(self, world, client, engine):
        carla = require_carla()

        self.world = world
        self.client = client
        self.engine = engine
        self.bp_lib = world.get_blueprint_library()

        # TM 初始化
        tm_port = int(config.STITCH_TM_PORT)
        self.tm = client.get_trafficmanager(tm_port)
        self.tm.set_synchronous_mode(True)
        self.tm.set_global_distance_to_leading_vehicle(float(config.STITCH_TM_FOLLOW_DISTANCE))
        self.tm.set_hybrid_physics_mode(False)
        self.tm.set_random_device_seed(0)

        # 状态
        self.pending_spawns: List[VehicleTrack] = []
        self.active: List[VehicleTrack] = []
        self.current_time: float = 0.0
        self._speed_update_timer: float = 0.0
        self._spawned_total: int = 0
        self._finished_total: int = 0
        self._spawn_failures: int = 0
        self._speed_limit_mps: float = float(config.STITCH_TM_SPEED_LIMIT_MPS)

        # 车辆蓝图缓存
        self._bp_cache = self._build_bp_cache()

        logger.info(
            "StitchAutopilot initialized: TM port=%d follow_dist=%.1f speed_limit=%.1f m/s",
            tm_port, config.STITCH_TM_FOLLOW_DISTANCE, self._speed_limit_mps,
        )

    def _build_bp_cache(self) -> dict:
        """构建车型 → 蓝图缓存。"""
        carla = require_carla()
        cache = {}
        type_map = {
            "car": "vehicle.tesla.model3",
            "truck": "vehicle.carlamotors.carlacola",
            "bus": "vehicle.volkswagen.t2",
            "van": "vehicle.dodge.charger_police",
            "tanker": "vehicle.carlamotors.carlacola",
            "pika": "vehicle.tesla.model3",
            "motorbike": "vehicle.bh.crossbike",
        }
        for vtype, bp_id in type_map.items():
            bp = self.bp_lib.find(bp_id)
            if bp:
                bp.set_attribute("role_name", f"stitch_{vtype}")
                cache[vtype] = bp
        return cache

    # ── 主循环 ────────────────────────────────────────────

    def load(self, tracks: List[VehicleTrack]):
        """加载 VehicleTrack 列表，按 start_time 排序。"""
        self.pending_spawns = sorted(tracks, key=lambda t: t.start_time)
        _info(f"StitchAutopilot loaded {len(self.pending_spawns)} tracks")
        logger.info("Spawn range: %.1fs – %.1fs",
                     self.pending_spawns[0].start_time if self.pending_spawns else 0,
                     self.pending_spawns[-1].start_time if self.pending_spawns else 0)

    def tick(self, delta_time: float) -> int:
        """单帧更新。返回活跃车辆数。"""
        self.current_time += delta_time * float(config.PLAYBACK_SPEED)

        self._spawn_due()
        self._update_speeds(delta_time)
        self._destroy_finished()

        return len(self.active)

    # ── 车辆生成 ──────────────────────────────────────────

    def _spawn_due(self):
        """生成到期的车辆。"""
        max_active = int(config.STITCH_TM_MAX_ACTIVE)
        stagger = int(config.STITCH_TM_SPAWN_STAGGER)
        spawned_this_tick = 0

        while (self.pending_spawns
               and len(self.active) < max_active
               and spawned_this_tick < stagger):

            t = self.pending_spawns[0]
            if t.start_time > self.current_time:
                break

            self.pending_spawns.pop(0)

            if t.actor and t.actor.is_alive:
                self.active.append(t)
                continue

            if not t.frames:
                self._spawn_failures += 1
                continue

            frame = t.frames[0]
            loc = frame.loc
            rot = frame.rot

            bp = self._bp_cache.get(t.type_str, self._bp_cache.get("car"))
            if bp is None:
                self._spawn_failures += 1
                continue

            try:
                transform = require_carla().Transform(loc, rot)
                actor = self.world.try_spawn_actor(bp, transform)
                if actor is None:
                    self._spawn_failures += 1
                    continue

                self._configure_autopilot(actor)
                self._set_initial_speed(actor, frame.v)
                t.actor = actor
                t.spawned = True
                self.active.append(t)
                self._spawned_total += 1
                spawned_this_tick += 1

            except Exception as e:
                self._spawn_failures += 1
                logger.debug("Spawn failed for %s: %s", t.id, e)

    def _configure_autopilot(self, vehicle):
        """配置 TM 自动驾驶参数。"""
        tm_port = int(config.STITCH_TM_PORT)
        vehicle.set_autopilot(True, tm_port)
        self.tm.ignore_lights_percentage(vehicle, 100.0)
        self.tm.ignore_signs_percentage(vehicle, 100.0)
        self.tm.auto_lane_change(vehicle, False)

        fwd_speed_pct = self._tm_speed_pct(vehicle, self._speed_limit_mps * 0.5)
        self.tm.vehicle_percentage_speed_difference(vehicle, fwd_speed_pct)

    def _set_initial_speed(self, vehicle, v_mps: float):
        """设置初始 TM 目标速度。"""
        pct = self._tm_speed_pct(vehicle, v_mps)
        self.tm.vehicle_percentage_speed_difference(vehicle, pct)

    # ── 速度更新 ──────────────────────────────────────────

    def _update_speeds(self, delta_time: float):
        """周期性更新 TM 目标速度。"""
        interval = float(config.STITCH_TM_SPEED_UPDATE_INTERVAL)
        self._speed_update_timer += delta_time * float(config.PLAYBACK_SPEED)

        if self._speed_update_timer < interval:
            return
        self._speed_update_timer = 0.0

        for t in self.active:
            v_mps = self._interpolate_speed(t)
            pct = self._tm_speed_pct(t.actor, v_mps)
            try:
                self.tm.vehicle_percentage_speed_difference(t.actor, pct)
            except Exception as e:
                logger.debug("TM speed update failed for %s: %s", t.id, e)

    def _interpolate_speed(self, track: VehicleTrack) -> float:
        """从速度曲线插值当前目标速度。"""
        profile = getattr(track, "speed_profile", [])
        if not profile:
            return self._speed_limit_mps * 0.7

        ct = self.current_time - max(0.0, track.start_time - track.frames[0].ts) if track.frames else self.current_time

        # 找到前后两个速度点
        prev = profile[0]
        for p in profile:
            if p[0] > ct:
                # 在 prev 和 p 之间插值
                dt = p[0] - prev[0]
                if dt > 0:
                    t_frac = (ct - prev[0]) / dt
                    return prev[1] + (p[1] - prev[1]) * t_frac
                return p[1]
            prev = p

        return prev[1]  # 超出范围用最后一个速度

    def _tm_speed_pct(self, vehicle, target_mps: float) -> float:
        """TM speed_difference 百分比。负值=减速, 正值=加速。"""
        limit = self._speed_limit_mps
        pct = (target_mps / max(0.1, limit) - 1.0) * 100.0
        return max(-90.0, min(90.0, pct))

    # ── 车辆销毁 ──────────────────────────────────────────

    def _destroy_finished(self):
        """销毁已走完行程的车辆。"""
        destroy_delay = float(config.DESTROY_DELAY)
        to_remove = []

        for t in self.active:
            end_time = t.end_time
            if end_time > 0 and self.current_time > end_time + destroy_delay:
                to_remove.append(t)

        for t in to_remove:
            try:
                if t.actor and t.actor.is_alive:
                    t.actor.destroy()
            except Exception:
                pass
            self.active.remove(t)
            self._finished_total += 1

    # ── 清理 ──────────────────────────────────────────────

    def cleanup(self):
        """销毁所有活跃车辆。"""
        for t in self.active:
            try:
                if t.actor and t.actor.is_alive:
                    t.actor.destroy()
            except Exception:
                pass
        self.active.clear()
        logger.info("StitchAutopilot cleanup: destroyed=%d total=%d failures=%d",
                     self._finished_total, self._spawned_total, self._spawn_failures)

    @property
    def stats(self) -> dict:
        return {
            "active": len(self.active),
            "pending": len(self.pending_spawns),
            "spawned": self._spawned_total,
            "finished": self._finished_total,
            "failures": self._spawn_failures,
            "current_time": self.current_time,
        }
