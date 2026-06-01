"""拼接轨迹车道路径对齐。

Phase 1: 从 XODR 提取三车道中心路径
Phase 2: 按摄像头对 x 像素值做三分位聚类 → 车道映射
Phase 3: 逐节点根据 x+camera_id 分配车道 → 车道路径查表 → world_loc
"""

from __future__ import annotations

import bisect
import math
import os
from typing import Dict, List, Optional, Tuple

from .carla_compat import require_carla


def load_lane_paths(xodr_path: str, step_m: float = 2.0) -> dict:
    """从 XODR 地图提取三车道中心路径。

    Returns:
        {"-1": [(x, y, z, yaw), ...], "-2": [...], "-3": [...]}
        每条车道按世界 Y 坐标升序排列。
    """
    from tp_tunnel_traffic.lane_sampling import build_three_lane_paths

    carla = require_carla()
    raw = build_three_lane_paths(carla, xodr_path, step_m=step_m, spawn_z=0.5)

    result = {}
    for lane_id in ("-1", "-2", "-3"):
        pts = []
        for pp in raw[lane_id]:
            loc = pp.transform.location
            yaw = pp.transform.rotation.yaw
            pts.append((loc.x, loc.y, loc.z, yaw))
        # 按世界 Y 排序，保证二分查找正确
        pts.sort(key=lambda p: p[1])
        result[lane_id] = pts
    return result


def cluster_x_to_lanes(trajs: List[dict]) -> Dict[str, Tuple[float, float]]:
    """对每个摄像头的实测节点 x 值做三分位聚类。

    Args:
        trajs: 拼接轨迹列表，每个含 "nodes" 列表

    Returns:
        {camera_id: (tercile1, tercile2)}
        x < tercile1 → 车道 -1 (左)
        tercile1 ≤ x < tercile2 → 车道 -2 (中)
        x ≥ tercile2 → 车道 -3 (右)
    """
    from collections import defaultdict

    cam_xs = defaultdict(list)
    for t in trajs:
        for n in t.get("nodes", []):
            x = n.get("x")
            cam = n.get("camera_id", "")
            if x is not None and cam and cam != "INTERP":
                cam_xs[cam].append(float(x))

    clusters = {}
    for cam, xs in cam_xs.items():
        xs.sort()
        n = len(xs)
        t1 = xs[n // 3]
        t2 = xs[2 * n // 3]
        clusters[cam] = (t1, t2)

    return clusters
