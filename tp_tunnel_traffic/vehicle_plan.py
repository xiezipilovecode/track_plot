from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class VehicleSpawnPlan:
    lane_index: int
    waypoint_index: int
    blueprint_id: str
    color: str | None = None
    speed_diff_percent: float = 0.0


def build_vehicle_plan(world, lane_points, config, rng: random.Random) -> List[VehicleSpawnPlan]:
    bp_lib = world.get_blueprint_library()
    blueprints = list(bp_lib.filter(config.blueprint_filter))
    if config.blueprints_deny:
        deny = {x.strip() for x in config.blueprints_deny.split(",") if x.strip()}
        blueprints = [bp for bp in blueprints if bp.id not in deny]
    deny_keywords = (
        "bike",
        "bicycle",
        "motorbike",
        "moto",
        "vespa",
        "cyclist",
        "walker",
        "police",
        "firetruck",
        "ambulance",
        "truck",
        "bus",
        "van",
        "carlamotors.firetruck",
        "tesla.cybertruck",
        "volkswagen.t2",
        "volkswagen.t2_2021",
    )
    preferred = [
        bp
        for bp in blueprints
        if not any(k in bp.id.lower() for k in deny_keywords)
        and any(
            k in bp.id.lower()
            for k in (
                "audi",
                "bmw",
                "chevrolet",
                "citroen",
                "ford",
                "lincoln",
                "mercedes",
                "nissan",
                "tesla.model3",
                "toyota",
                "volkswagen",
                "mini",
                "kia",
                "hyundai",
                "honda",
                "mazda",
                "subaru",
                "renault",
                "seat",
                "peugeot",
            )
        )
    ]
    if preferred:
        blueprints = preferred
    if not blueprints:
        raise RuntimeError("没有可用的车辆蓝图")

    def _cum_dist_m(points) -> List[float]:
        if not points:
            return []
        out = [0.0]
        for i in range(1, len(points)):
            a = points[i - 1].transform.location
            b = points[i].transform.location
            dx = b.x - a.x
            dy = b.y - a.y
            dz = b.z - a.z
            out.append(out[-1] + (dx * dx + dy * dy + dz * dz) ** 0.5)
        return out

    def _idx_at_dist(cum: List[float], dist_m: float) -> int:
        # 线性扫描即可（lane 点数通常不大；避免引入 bisect 依赖/类型复杂度）
        if not cum:
            return 0
        d = max(0.0, float(dist_m))
        for i, v in enumerate(cum):
            if v >= d:
                return i
        return len(cum) - 1

    def _get_int(name: str, default: int) -> int:
        try:
            return int(getattr(config, name))
        except Exception:
            return int(default)

    def _get_float(name: str, default: float) -> float:
        try:
            return float(getattr(config, name))
        except Exception:
            return float(default)

    # 代理车流优先使用 TT_PROXY_*；否则回退到兼容字段
    min_per_lane = _get_int("proxy_min_per_lane", _get_int("min_vehicles_per_lane", 1))
    max_per_lane = _get_int("proxy_max_per_lane", _get_int("max_vehicles_per_lane", 1))
    if max_per_lane < min_per_lane:
        min_per_lane, max_per_lane = max_per_lane, min_per_lane
    spawn_start_ratio = max(0.0, min(1.0, _get_float("proxy_spawn_start_ratio", 0.0)))
    follow_distance_m = max(0.0, _get_float("proxy_follow_distance_m", _get_float("tm_follow_distance", 8.0)))
    # 预留入口距离（米）：让主车能在 warmup 后可靠生成
    reserved_entry_m = max(0.0, _get_float("proxy_reserved_entry_m", 0.0))
    # 生成间距：至少 follow_distance + 车长缓冲（粗略用 5m）
    min_spacing_m = max(4.5, follow_distance_m + 3.5)
    fallback_speed_diff_percent = _get_float("proxy_speed_diff_percent", _get_float("tm_speed_diff_percent", 0.0))

    def _lane_speed_diff_percent(lane_index: int) -> float:
        # 左快中稳右慢：lane_index 对应 [-1, -2, -3] -> [0, 1, 2]
        if lane_index == 0:
            lo = _get_float("proxy_speed_diff_left_min", -8.0)
            hi = _get_float("proxy_speed_diff_left_max", -4.0)
        elif lane_index == 1:
            lo = _get_float("proxy_speed_diff_mid_min", -2.0)
            hi = _get_float("proxy_speed_diff_mid_max", 2.0)
        elif lane_index == 2:
            lo = _get_float("proxy_speed_diff_right_min", 4.0)
            hi = _get_float("proxy_speed_diff_right_max", 9.0)
        else:
            return float(fallback_speed_diff_percent)
        if hi < lo:
            lo, hi = hi, lo
        return float(rng.uniform(lo, hi))

    plans: List[VehicleSpawnPlan] = []
    for lane_index, lane_wps in enumerate(lane_points):
        if not lane_wps:
            continue

        desired = rng.randint(int(min_per_lane), int(max_per_lane))
        if desired <= 0:
            continue

        cum = _cum_dist_m(lane_wps)
        lane_len = float(cum[-1]) if cum else 0.0
        if lane_len <= 1e-3:
            continue

        start_m = max(float(reserved_entry_m), float(spawn_start_ratio) * lane_len)
        if start_m >= lane_len:
            continue

        avail = max(0.0, lane_len - start_m)
        max_possible = int(avail // float(min_spacing_m)) + 1 if avail > 1e-3 else 0
        if max_possible <= 0:
            continue
        total = min(int(desired), int(max_possible))
        if total <= 0:
            continue

        chosen_indices: List[int] = []
        used = set()
        for i in range(total):
            base_d = start_m + float(i) * float(min_spacing_m)
            jitter = (rng.random() - 0.5) * float(min_spacing_m) * 0.10
            d = max(start_m, min(lane_len, base_d + jitter))
            idx = _idx_at_dist(cum, d)
            idx = max(0, min(idx, len(lane_wps) - 2))
            while idx in used and idx < len(lane_wps) - 2:
                idx += 1
            if idx in used:
                continue
            used.add(idx)
            chosen_indices.append(idx)

        for wp_idx in chosen_indices:
            bp = rng.choice(blueprints)
            color = None
            if bp.has_attribute("color"):
                try:
                    values = list(bp.get_attribute("color").recommended_values)
                    if values:
                        color = rng.choice(values)
                except Exception:
                    color = None
            speed_diff_percent = _lane_speed_diff_percent(int(lane_index))
            plans.append(
                VehicleSpawnPlan(
                    lane_index=lane_index,
                    waypoint_index=int(wp_idx),
                    blueprint_id=bp.id,
                    color=color,
                    speed_diff_percent=float(speed_diff_percent),
                )
            )
    return plans
