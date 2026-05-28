"""轨迹数据解析器。

扫描数据目录 → 按摄像头分组 → 逐文件解析 → 过滤 → 构建 CameraDataset。
"""

from __future__ import annotations

import logging
import os
import re
from collections import defaultdict
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from .config import StitchingConfig
from .models import CameraDataset, Trajectory, TrajectoryNode

logger = logging.getLogger(__name__)

# 文件名正则：K615+703TV023_2024-07-25-14-40-19.txt
_FILENAME_RE = re.compile(
    r"K(\d+)\+(\d+)TV(\d+)_(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2})\.txt"
)


def parse_filename(filename: str) -> Optional[dict]:
    """解析文件名，提取摄像头ID、时间窗口等信息。"""
    m = _FILENAME_RE.match(filename)
    if not m:
        return None
    return {
        "km_main": int(m.group(1)),
        "km_offset": int(m.group(2)),
        "camera_id": f"TV{m.group(3)}",
        "year": int(m.group(4)),
        "month": int(m.group(5)),
        "day": int(m.group(6)),
        "hour": int(m.group(7)),
        "minute": int(m.group(8)),
        "second": int(m.group(9)),
    }


def scan_data_directory(
    data_root: str,
    cameras: List[str],
    max_files: int = 0,
) -> Dict[str, List[str]]:
    """扫描数据目录，将文件按摄像头ID分组。

    max_files: 0=全部; >0=每摄像头处理文件数上限
    """
    camera_files: Dict[str, List[str]] = {cam: [] for cam in cameras}

    all_txt = [
        f for f in os.listdir(data_root)
        if f.endswith(".txt") and not f.startswith(".")
    ]

    # 按摄像头分组后分别取前 N 个
    for fname in sorted(all_txt):
        info = parse_filename(fname)
        if info is None:
            continue
        cam_id = info["camera_id"]
        if cam_id not in camera_files:
            continue

        if max_files > 0 and len(camera_files[cam_id]) >= max_files:
            # 检查是否所有摄像头都已满
            if all(len(v) >= max_files for v in camera_files.values()):
                break
            continue

        full_path = os.path.join(data_root, fname)
        camera_files[cam_id].append(full_path)

    return camera_files


def parse_file(
    file_path: str,
    camera_id: str,
    config: StitchingConfig,
) -> List[Trajectory]:
    """解析单个 txt 文件，返回 Trajectory 列表。

    每行一条轨迹，空格分隔，6 字段一组节点。
    """
    trajectories: List[Trajectory] = []
    fname = os.path.basename(file_path)
    info = parse_filename(fname)
    window_start = ""
    if info:
        window_start = (
            f"{info['year']}-{info['month']:02d}-{info['day']:02d} "
            f"{info['hour']:02d}:{info['minute']:02d}:{info['second']:02d}"
        )

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except OSError as e:
        logger.warning("Failed to read %s: %s", file_path, e)
        return trajectories

    for line in lines:
        line = line.strip()
        if not line:
            continue

        tokens = line.split()
        if len(tokens) < 6:
            continue

        traj = Trajectory(
            camera_id=camera_id,
            file_name=fname,
            window_start=window_start,
        )

        # 逐节点解析（每 6 个 token 一组）
        for i in range(0, len(tokens), 6):
            node = TrajectoryNode.from_tokens(
                tokens, i,
                camera_id=camera_id,
                skip_zero_zero=config.skip_zero_zero_nodes,
            )
            if node is not None:
                traj.add_node(node)

        if traj.nodes:
            trajectories.append(traj)

    return trajectories


def parse_all_files(
    camera_files: Dict[str, List[str]],
    config: StitchingConfig,
) -> Dict[str, CameraDataset]:
    """解析所有文件，构建 CameraDataset 字典。"""
    datasets: Dict[str, CameraDataset] = {}

    for cam_id in config.cameras:
        files = camera_files.get(cam_id, [])
        dataset = CameraDataset(camera_id=cam_id)

        total_nodes = 0
        total_tracks = 0
        for fp in files:
            tracks = parse_file(fp, cam_id, config)
            dataset.trajectories.extend(tracks)
            total_tracks += len(tracks)
            total_nodes += sum(len(t.nodes) for t in tracks)

        datasets[cam_id] = dataset
        logger.info(
            "Parsed %s: %d files, %d tracks, %d nodes",
            cam_id, len(files), total_tracks, total_nodes,
        )

    return datasets


def merge_within_camera(
    dataset: CameraDataset,
    config: StitchingConfig,
) -> CameraDataset:
    """同一摄像头内跨文件窗口合并（Step 0）。

    按窗口时间排序，合并相邻窗口边界附近、时空约束通过的轨迹对。
    合并策略：将匹配的轨迹 b 的节点追加到轨迹 a，然后移除 b。
    """
    if len(dataset.trajectories) < 2:
        return dataset

    # 按窗口起始时间 + y 坐标排序
    sorted_trajs = sorted(
        dataset.trajectories,
        key=lambda t: (t.window_start, t.y_start),
    )

    merged = []
    used = set()
    max_time_gap = config.within_cam_max_time_gap_ms
    max_y_gap = config.within_cam_max_y_gap_cm

    for i, traj_a in enumerate(sorted_trajs):
        if i in used or not traj_a.is_multi_node:
            continue

        last_a = traj_a.last_valid()
        if last_a is None:
            continue

        best_j = -1
        best_cost = float("inf")

        # 只在后续 N 个轨迹中搜索（窗口相邻）
        search_range = min(i + 20, len(sorted_trajs))
        for j in range(i + 1, search_range):
            if j in used or not sorted_trajs[j].is_multi_node:
                continue

            traj_b = sorted_trajs[j]
            first_b = traj_b.first_valid()
            if first_b is None:
                continue

            time_diff = first_b.timestamp - last_a.timestamp
            y_diff = first_b.y - last_a.y

            # 时空约束
            if time_diff <= 0 or time_diff > max_time_gap:
                continue
            if abs(y_diff) > max_y_gap:
                continue

            # 类型一致性
            if traj_a.majority_type != traj_b.majority_type:
                continue

            # 简单代价
            cost = time_diff / 1000.0 + abs(y_diff) / 100.0
            if cost < best_cost:
                best_cost = cost
                best_j = j

        if best_j >= 0:
            # 合并：将 traj_b 的节点追加到 traj_a
            for node in sorted_trajs[best_j].nodes:
                traj_a.add_node(node)
            used.add(best_j)
            # 清除缓存后重新排序
            traj_a._valid_nodes = None
            traj_a._majority_type = None

        merged.append(traj_a)

    logger.info(
        "Within-camera merge for %s: %d -> %d tracks",
        dataset.camera_id,
        len(dataset.trajectories),
        len(merged),
    )

    return CameraDataset(camera_id=dataset.camera_id, trajectories=merged)
