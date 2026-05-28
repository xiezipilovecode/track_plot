"""跨摄像头轨迹匹配模块。

计算代价矩阵 → 匈牙利算法求解 → 返回匹配对。
"""

from __future__ import annotations

import logging
from typing import List, Tuple, Optional

import numpy as np

from .config import StitchingConfig
from .models import CameraDataset, Trajectory, TrajectoryNode

logger = logging.getLogger(__name__)

# 延迟导入 scipy（仅匹配时需要）
try:
    from scipy.optimize import linear_sum_assignment as _hungarian
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False
    _hungarian = None


def _compute_pair_cost(
    traj_a: Trajectory,
    traj_b: Trajectory,
    config: StitchingConfig,
) -> float:
    """计算一对轨迹的匹配代价（值越小越匹配）。

    使用：
    - traj_a 的最后一个有效节点（前摄像头出口）
    - traj_b 的第一个有效节点（后摄像头入口）
    """
    end_a = traj_a.last_valid()
    start_b = traj_b.first_valid()
    if end_a is None or start_b is None:
        return float("inf")

    time_diff_ms = start_b.timestamp - end_a.timestamp
    y_diff = start_b.y - end_a.y

    # ── 时间代价（归一化到秒） ──
    cost = config.weight_time * time_diff_ms / 1000.0

    # ── 位置代价（归一化到米，y_diff 单位 cm） ──
    cost += config.weight_position * abs(y_diff) / 100.0

    # ── 速度一致性代价 ──
    if end_a.speed is not None and end_a.speed > 1.0 and start_b.speed is not None:
        speed_a_cm_ms = end_a.speed * config.speed_kmh_to_cm_ms  # km/h → cm/ms
        if speed_a_cm_ms > 0.001:
            expected_travel_ms = abs(y_diff) / speed_a_cm_ms
            speed_cost = abs(expected_travel_ms - time_diff_ms) / 1000.0
            cost += config.weight_speed * speed_cost

    # ── 类型一致性代价（多数投票） ──
    if traj_a.majority_type != traj_b.majority_type:
        cost += config.weight_type * config.penalty_type_mismatch

    return cost


def _prefilter(
    traj_a: Trajectory,
    traj_b: Trajectory,
    config: StitchingConfig,
) -> bool:
    """快速预筛选：时空约束 + 类型初筛。"""
    end_a = traj_a.last_valid()
    start_b = traj_b.first_valid()
    if end_a is None or start_b is None:
        return False

    # 时间约束：前轨迹必须早于后轨迹
    time_diff = start_b.timestamp - end_a.timestamp
    if time_diff <= 0 or time_diff > config.max_time_gap_ms:
        return False

    # 空间约束
    y_diff = start_b.y - end_a.y
    if abs(y_diff) > config.max_y_gap_cm:
        return False

    # 宽松的类型预检
    if traj_a.majority_type != traj_b.majority_type:
        return False

    return True


def match_camera_pair(
    dataset_a: CameraDataset,
    dataset_b: CameraDataset,
    config: StitchingConfig,
) -> List[Tuple[Trajectory, Trajectory, float]]:
    """匈牙利算法匹配相邻两个摄像头的轨迹（分批 + 去重）。"""
    if not _HAS_SCIPY:
        logger.error("scipy not available")
        return []

    active_a = dataset_a.get_active_trajs(
        min_nodes=config.min_nodes_per_track,
        skip_static=config.skip_static_tracks,
    )
    active_b = dataset_b.get_active_trajs(
        min_nodes=config.min_nodes_per_track,
        skip_static=config.skip_static_tracks,
    )

    if not active_a or not active_b:
        return []

    window_ms = config.time_window_minutes * 60_000
    all_matches_raw = []

    for batch_a, batch_b, _ in _partition_by_time(
        active_a, active_b, window_ms, config.max_time_gap_ms,
    ):
        if not batch_a or not batch_b:
            continue
        batch_matches = _match_one_batch(batch_a, batch_b, config)
        all_matches_raw.extend(batch_matches)

    # 去重：每个 traj_b 最多匹配一个 traj_a（保留代价最小的）
    best_for_b: dict = {}
    for traj_a, traj_b, cost in all_matches_raw:
        key = id(traj_b)
        if key not in best_for_b or cost < best_for_b[key][2]:
            best_for_b[key] = (traj_a, traj_b, cost)

    all_matches = sorted(best_for_b.values(), key=lambda x: x[2])

    total = len(all_matches)
    logger.info(
        "Match result %s ↔ %s: %d matched (rate=%.1f%%) [raw=%d batches]",
        dataset_a.camera_id, dataset_b.camera_id,
        total,
        100.0 * total / max(1, min(len(active_a), len(active_b))),
        len(all_matches_raw),
    )
    return all_matches


def _partition_by_time(
    active_a: list,
    active_b: list,
    window_ms: int,
    max_gap_ms: int,
):
    """按时间窗口分批，相邻窗口有重叠以覆盖边界。"""
    all_a = sorted(active_a, key=lambda t: t.start_time)
    all_b = sorted(active_b, key=lambda t: t.start_time)

    if not all_a:
        return

    min_time = min(all_a[0].start_time, all_b[0].start_time if all_b else float("inf"))
    max_time = max(all_a[-1].end_time, all_b[-1].end_time if all_b else float("-inf"))

    # 使用有重叠的滑动窗口
    overlap_ms = max_gap_ms  # 窗口重叠量 = 最大匹配间隙
    step_ms = window_ms - overlap_ms
    if step_ms <= 0:
        step_ms = window_ms // 2

    window_start = min_time
    while window_start < max_time:
        window_end = window_start + window_ms + overlap_ms

        # 收集窗口内的轨迹
        batch_a = [t for t in all_a
                    if t.start_time < window_end and t.end_time > window_start - max_gap_ms]
        batch_b = [t for t in all_b
                    if t.start_time < window_end and t.end_time > window_start - max_gap_ms]

        if batch_a and batch_b:
            yield batch_a, batch_b, window_start

        window_start += step_ms


def _match_one_batch(
    batch_a: list,
    batch_b: list,
    config: StitchingConfig,
) -> List[Tuple[Trajectory, Trajectory, float]]:
    """单批匈牙利匹配。"""
    n_a = len(batch_a)
    n_b = len(batch_b)
    n_max = max(n_a, n_b)

    # 限制单批矩阵大小（安全阀）
    MAX_BATCH_SIZE = 5000
    if n_max > MAX_BATCH_SIZE:
        logger.warning(
            "Batch too large (%d × %d), limiting to %d",
            n_a, n_b, MAX_BATCH_SIZE,
        )
        batch_a = batch_a[:MAX_BATCH_SIZE]
        batch_b = batch_b[:MAX_BATCH_SIZE]
        n_a = len(batch_a)
        n_b = len(batch_b)
        n_max = max(n_a, n_b)

    # 构建代价矩阵
    cost_matrix = np.full((n_max, n_max), 1e9, dtype=np.float64)
    candidate_count = 0

    for i, traj_a in enumerate(batch_a):
        for j, traj_b in enumerate(batch_b):
            if _prefilter(traj_a, traj_b, config):
                cost = _compute_pair_cost(traj_a, traj_b, config)
                cost_matrix[i, j] = cost
                candidate_count += 1

    # 匈牙利算法
    row_ind, col_ind = _hungarian(cost_matrix)

    # 过滤有效匹配
    matches = []
    for r, c in zip(row_ind, col_ind):
        if r < n_a and c < n_b:
            cost_val = cost_matrix[r, c]
            if cost_val < config.max_acceptable_cost:
                matches.append((batch_a[r], batch_b[c], float(cost_val)))

    return matches


def match_all_pairs(
    datasets: dict,
    config: StitchingConfig,
) -> List[List[Tuple[Trajectory, Trajectory, float]]]:
    """对 5 对相邻摄像头执行匹配。

    Returns:
        [matches_01, matches_12, matches_23, matches_34, matches_45]
    """
    all_matches = []
    for cam_a, cam_b in config.camera_pairs:
        ds_a = datasets.get(cam_a)
        ds_b = datasets.get(cam_b)
        if ds_a is None or ds_b is None:
            logger.warning("Missing dataset for %s or %s", cam_a, cam_b)
            all_matches.append([])
            continue

        matches = match_camera_pair(ds_a, ds_b, config)
        all_matches.append(matches)

    return all_matches
