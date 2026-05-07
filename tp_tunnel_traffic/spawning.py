from __future__ import annotations

import math
from typing import Any, Iterable


def _planar_dist2(a, b) -> float:
    dx = float(a.x) - float(b.x)
    dy = float(a.y) - float(b.y)
    return dx * dx + dy * dy


def _lane_aware_blocked(
    actor_loc,
    spawn_tf,
    *,
    same_lane_clearance_m: float,
    same_lane_lateral_m: float,
    other_lane_clearance_m: float,
    other_lane_lateral_m: float,
) -> bool:
    # 仅使用 yaw 构造 forward/right 做近似的“纵向/横向”分解。
    # 目的：同车道保持足够纵向间距；相邻车道仅做最小碰撞净空，避免跨车道互相阻塞导致大量生成失败。
    yaw_deg = float(getattr(getattr(spawn_tf, "rotation", None), "yaw", 0.0))
    yaw = math.radians(yaw_deg)
    fx = math.cos(yaw)
    fy = math.sin(yaw)
    rx = -fy
    ry = fx

    dx = float(actor_loc.x) - float(spawn_tf.location.x)
    dy = float(actor_loc.y) - float(spawn_tf.location.y)
    lon = dx * fx + dy * fy
    lat = abs(dx * rx + dy * ry)

    if lat <= float(same_lane_lateral_m):
        return abs(lon) < float(same_lane_clearance_m)
    if lat <= float(other_lane_lateral_m):
        return (dx * dx + dy * dy) < float(other_lane_clearance_m) * float(other_lane_clearance_m)
    return False


def try_spawn_vehicle(
    world,
    blueprint,
    transform,
    clearance_m: float,
    existing_actors: Iterable[Any],
    *,
    lane_aware_clearance: bool = False,
    same_lane_clearance_m: float | None = None,
    same_lane_lateral_m: float = 1.6,
    other_lane_clearance_m: float = 2.5,
    other_lane_lateral_m: float = 6.0,
):
    if not lane_aware_clearance:
        for actor in existing_actors:
            try:
                if actor is not None and actor.is_alive:
                    d = actor.get_location().distance(transform.location)
                    if d < clearance_m:
                        return None
            except Exception:
                continue
    else:
        same_lane_eff = float(clearance_m) if same_lane_clearance_m is None else float(same_lane_clearance_m)
        for actor in existing_actors:
            try:
                if actor is None or not actor.is_alive:
                    continue
                loc = actor.get_location()
                if _lane_aware_blocked(
                    loc,
                    transform,
                    same_lane_clearance_m=same_lane_eff,
                    same_lane_lateral_m=float(same_lane_lateral_m),
                    other_lane_clearance_m=float(other_lane_clearance_m),
                    other_lane_lateral_m=float(other_lane_lateral_m),
                ):
                    return None
            except Exception:
                continue

        # lane-aware 模式下仍保留一个极小的“硬碰撞”净空，防止异常 actor（不在车道上）贴脸生成。
        try:
            hard2 = float(other_lane_clearance_m) * float(other_lane_clearance_m)
            for actor in existing_actors:
                try:
                    if actor is None or not actor.is_alive:
                        continue
                    if _planar_dist2(actor.get_location(), transform.location) < hard2:
                        return None
                except Exception:
                    continue
        except Exception:
            pass

    try:
        return world.try_spawn_actor(blueprint, transform)
    except Exception:
        return None


def destroy_spawned_actors(actors: Iterable[Any]):
    for actor in actors:
        try:
            if actor is not None and actor.is_alive:
                actor.destroy()
        except Exception:
            continue
