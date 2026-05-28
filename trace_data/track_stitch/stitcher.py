"""轨迹拼接编排引擎。

串联：解析 → 同摄像头合并 → 跨摄像头匹配 → 盲区插值 → 轨迹组装 → 导出 JSON。
"""

from __future__ import annotations

import json
import logging
import os
import time
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

from .config import StitchingConfig
from .matcher import match_all_pairs
from .models import (
    CameraDataset, StitchedTrajectory,
    Trajectory, TrajectoryNode,
)
from .parser import (
    merge_within_camera,
    parse_all_files,
    scan_data_directory,
)

logger = logging.getLogger(__name__)


class Stitcher:
    """轨迹拼接编排引擎。"""

    def __init__(self, config: StitchingConfig):
        self.config = config
        self.stats: dict = {
            "total_files": 0,
            "total_tracks_parsed": 0,
            "total_nodes_parsed": 0,
            "tracks_after_merge": 0,
            "matched_pairs": 0,
            "stitched_trajectories": 0,
            "unmatched_fragments": 0,
            "interpolated_nodes": 0,
            "elapsed_seconds": 0.0,
        }

    # ── 主流程 ──────────────────────────────────────────────

    def run(self) -> List[StitchedTrajectory]:
        """执行完整拼接流程。"""
        t0 = time.perf_counter()

        # Step 0+1: 扫描 + 解析
        logger.info("=" * 50)
        logger.info("Step 0+1: Scanning and parsing data files...")
        camera_files = scan_data_directory(
            self.config.data_root,
            self.config.cameras,
            max_files=self.config.max_files_to_scan,
        )
        self.stats["total_files"] = sum(len(v) for v in camera_files.values())

        datasets = parse_all_files(camera_files, self.config)
        self.stats["total_tracks_parsed"] = sum(
            len(ds.trajectories) for ds in datasets.values()
        )
        self.stats["total_nodes_parsed"] = sum(
            sum(len(t.nodes) for t in ds.trajectories)
            for ds in datasets.values()
        )

        # Step 0b: 同摄像头跨文件合并
        logger.info("=" * 50)
        logger.info("Step 0b: Within-camera cross-file merging...")
        merged_datasets = {}
        for cam_id in self.config.cameras:
            ds = datasets.get(cam_id)
            if ds is None:
                continue
            merged_ds = merge_within_camera(ds, self.config)
            merged_datasets[cam_id] = merged_ds
        self.stats["tracks_after_merge"] = sum(
            len(ds.trajectories) for ds in merged_datasets.values()
        )

        # 打印统计
        for cam_id in self.config.cameras:
            ds = merged_datasets.get(cam_id)
            if ds:
                summary = ds.to_summary()
                logger.info(
                    "  %s: total=%d multi=%d static=%d active=%d y=[%.0f, %.0f]",
                    cam_id,
                    summary["total_tracks"],
                    summary["multi_node_tracks"],
                    summary["static_tracks"],
                    summary["active_tracks"],
                    summary["y_min"],
                    summary["y_max"],
                )

        # Step 2: 跨摄像头匹配
        logger.info("=" * 50)
        logger.info("Step 2: Cross-camera Hungarian matching...")
        all_pair_matches = match_all_pairs(merged_datasets, self.config)
        total_matched = sum(len(m) for m in all_pair_matches)
        self.stats["matched_pairs"] = total_matched
        logger.info("Total matched pairs: %d", total_matched)

        # Step 3: 轨迹组装（链式拼接）
        logger.info("=" * 50)
        logger.info("Step 3: Assembling stitched trajectories...")
        stitched = self._assemble_trajectories(
            merged_datasets, all_pair_matches,
        )
        self.stats["stitched_trajectories"] = len(stitched)
        logger.info("Assembled %d stitched trajectories", len(stitched))

        # Step 4: 盲区插值
        logger.info("=" * 50)
        logger.info("Step 4: Blind zone interpolation...")
        interp_count = 0
        for st in stitched:
            interp_count += self._interpolate_gaps(st)
        self.stats["interpolated_nodes"] = interp_count
        logger.info("Inserted %d interpolated nodes", interp_count)

        # 排序 + 质量评分
        for st in stitched:
            st.sort_nodes()
            st.quality_score = self._compute_quality(st)

        self.stats["elapsed_seconds"] = time.perf_counter() - t0
        logger.info("=" * 50)
        logger.info(
            "Stitching complete: %d trajectories in %.1f seconds",
            len(stitched),
            self.stats["elapsed_seconds"],
        )

        return stitched

    # ── 轨迹组装 ────────────────────────────────────────────

    def _assemble_trajectories(
        self,
        datasets: Dict[str, CameraDataset],
        all_pair_matches: List[List[Tuple[Trajectory, Trajectory, float]]],
    ) -> List[StitchedTrajectory]:
        """将匹配对链式组装为完整轨迹。

        通过 BFS/贪心连接，从 TV023 开始向后追溯。
        """
        # 构建匹配图
        # match_graph[traj_a] = [(traj_b, cost, pair_idx)]
        match_graph: dict = defaultdict(list)
        all_trajs_b_used: set = set()

        for pair_idx, matches in enumerate(all_pair_matches):
            for traj_a, traj_b, cost in matches:
                match_graph[id(traj_a)].append((traj_b, cost, pair_idx))
                all_trajs_b_used.add(id(traj_b))

        stitched_list: List[StitchedTrajectory] = []
        traj_id = 0

        # 找到所有起点的轨迹（TV023 中没有被作为 "b" 使用的）
        first_ds = datasets.get(self.config.cameras[0])
        if first_ds is None:
            return stitched_list

        for traj in first_ds.get_active_trajs(
            min_nodes=self.config.min_nodes_per_track,
            skip_static=self.config.skip_static_tracks,
        ):
            chain = self._build_chain(
                traj, match_graph, datasets,
            )
            if chain:
                stitched = self._chain_to_stitched(
                    chain, traj_id,
                )
                stitched_list.append(stitched)
                traj_id += 1

        logger.info(
            "Chained %d trajectories from %d starting fragments",
            len(stitched_list),
            len(first_ds.get_active_trajs(
                min_nodes=self.config.min_nodes_per_track,
                skip_static=self.config.skip_static_tracks,
            )),
        )

        return stitched_list

    def _build_chain(
        self,
        start_traj: Trajectory,
        match_graph: dict,
        datasets: Dict[str, CameraDataset],
    ) -> List[Tuple[Trajectory, float, int]]:
        """从起始轨迹向后追溯匹配链。"""
        chain = [(start_traj, 0.0, -1)]  # (traj, cost, pair_idx)
        current = start_traj

        while True:
            next_candidates = match_graph.get(id(current), [])
            if not next_candidates:
                break

            # 选代价最小的匹配对
            next_candidates.sort(key=lambda x: x[1])
            best = next_candidates[0]

            chain.append(best)
            current = best[0]

        return chain if len(chain) >= 2 else []  # 至少跨 2 个摄像头

    def _chain_to_stitched(
        self,
        chain: List[Tuple[Trajectory, float, int]],
        traj_id: int,
    ) -> StitchedTrajectory:
        """将匹配链转为 StitchedTrajectory。"""
        stitched = StitchedTrajectory(
            trajectory_id=f"TRAJ_{traj_id:06d}",
        )

        for i, (traj, cost, pair_idx) in enumerate(chain):
            for node in traj.nodes:
                if node.is_valid:
                    stitched.nodes.append(node)

            # 记录匹配对
            if i > 0:
                prev_cam = chain[i - 1][0].camera_id
                this_cam = traj.camera_id
                stitched.matched_pairs.append((prev_cam, this_cam, cost))

        return stitched

    # ── 盲区插值 ────────────────────────────────────────────

    def _interpolate_gaps(self, stitched: StitchedTrajectory) -> int:
        """在轨迹节点间插入盲区插值点。

        Returns: 插入的插值节点数
        """
        if len(stitched.nodes) < 2:
            return 0

        stitched.sort_nodes()
        interp_count = 0
        new_nodes = []

        config = self.config
        max_gap = config.interp_max_gap_ms

        for i in range(len(stitched.nodes)):
            new_nodes.append(stitched.nodes[i])

            if i >= len(stitched.nodes) - 1:
                continue

            curr = stitched.nodes[i]
            next_node = stitched.nodes[i + 1]

            # 跳过相邻同摄像头内的节点间隙（只看跨摄像头间隙）
            if curr.camera_id == next_node.camera_id:
                continue

            dt_ms = next_node.timestamp - curr.timestamp
            if dt_ms <= 0 or dt_ms > max_gap:
                continue

            dy = next_node.y - curr.y

            # 估算插值速度
            if curr.speed and curr.speed > 1.0:
                est_speed = curr.speed
            else:
                est_speed = (dy / dt_ms) * config.cm_ms_to_kmh if dt_ms > 0 else 0

            n_points = config.interp_points_per_gap
            for p in range(1, n_points + 1):
                ratio = p / (n_points + 1)
                interp_node = TrajectoryNode(
                    timestamp=int(curr.timestamp + dt_ms * ratio),
                    x=0.0,
                    y=curr.y + dy * ratio,
                    speed=est_speed,
                    acceleration=None,
                    vehicle_type=curr.vehicle_type,
                    is_valid=True,
                    interpolated=True,
                    camera_id="INTERP",
                )
                new_nodes.append(interp_node)
                interp_count += 1

        stitched.nodes = new_nodes
        return interp_count

    # ── 质量评分 ────────────────────────────────────────────

    def _compute_quality(self, stitched: StitchedTrajectory) -> float:
        """计算拼接轨迹质量评分（0-1）。"""
        if not stitched.nodes:
            return 0.0

        # 完整性评分（摄像头覆盖数 / 6）
        completeness = min(1.0, stitched.camera_count / 6.0)

        # 连续性评分（1 - 插值比）
        continuity = 1.0 - stitched.interp_ratio

        # 匹配置信度（1 / (1 + avg_match_cost)）
        if stitched.matched_pairs:
            avg_cost = sum(c for _, _, c in stitched.matched_pairs) / len(stitched.matched_pairs)
            match_conf = 1.0 / (1.0 + avg_cost / 1000.0)
        else:
            match_conf = 0.5

        return 0.3 * completeness + 0.3 * continuity + 0.2 * match_conf + 0.2 * min(1.0, stitched.camera_count / 3.0)

    # ── 导出 ────────────────────────────────────────────────

    def export(
        self,
        stitched: List[StitchedTrajectory],
        output_dir: str,
    ) -> str:
        """导出拼接结果到 JSON 文件。"""
        os.makedirs(output_dir, exist_ok=True)

        output_path = os.path.join(output_dir, self.config.output_json)
        data = {
            "metadata": {
                "source": self.config.data_root,
                "cameras": self.config.cameras,
                "stats": self.stats,
            },
            "trajectories": [s.to_dict() for s in stitched],
        }

        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        logger.info("Exported %d trajectories to %s", len(stitched), output_path)

        # 导出统计
        stats_path = os.path.join(output_dir, self.config.output_stats)
        with open(stats_path, "w", encoding="utf-8") as f:
            json.dump(self.stats, f, ensure_ascii=False, indent=2)

        return output_path
