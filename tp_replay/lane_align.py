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
