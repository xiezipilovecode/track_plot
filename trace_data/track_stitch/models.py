"""轨迹拼接数据模型。"""

from __future__ import annotations

import json
from collections import Counter
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

import numpy as np


@dataclass
class TrajectoryNode:
    """单个轨迹节点（6字段）。"""

    timestamp: int          # Unix 毫秒时间戳
    x: float                # 像素 x 坐标（可能为 0=invalid）
    y: float                # 世界 y 坐标（cm）
    speed: Optional[float]  # 瞬时速度（km/h），可能为 None
    acceleration: Optional[float]  # 加速度
    vehicle_type: str       # car / truck / tanker / van / bus / pika / motorbike
    is_valid: bool = True   # False 表示 (0,0) 异常点
    interpolated: bool = False   # 插值节点标记
    camera_id: str = ""     # 来源摄像头

    @classmethod
    def from_tokens(
        cls,
        tokens: List[str],
        idx: int,
        camera_id: str = "",
        skip_zero_zero: bool = True,
    ) -> Optional["TrajectoryNode"]:
        """从 6 元组 token 创建节点。idx 为 token 起始索引。"""
        try:
            ts = int(float(tokens[idx]))
            x = float(tokens[idx + 1])
            y = float(tokens[idx + 2])

            # 速度处理
            speed_raw = tokens[idx + 3] if idx + 3 < len(tokens) else "null"
            speed = None
            if speed_raw.strip().lower() not in ("null", ""):
                try:
                    speed = float(speed_raw)
                except ValueError:
                    speed = None

            # 加速度处理
            accel_raw = tokens[idx + 4] if idx + 4 < len(tokens) else "null"
            accel = None
            if accel_raw.strip().lower() not in ("null", ""):
                try:
                    accel = float(accel_raw)
                except ValueError:
                    accel = None

            vtype = tokens[idx + 5] if idx + 5 < len(tokens) else "car"

            # (0,0) 检测
            is_valid = True
            if skip_zero_zero and abs(x) < 1.0 and abs(y) < 1.0:
                is_valid = False

            return cls(
                timestamp=ts,
                x=x,
                y=y,
                speed=speed,
                acceleration=accel,
                vehicle_type=vtype,
                is_valid=is_valid,
                camera_id=camera_id,
            )
        except (ValueError, IndexError):
            return None

    def to_dict(self) -> dict:
        return {
            "timestamp": self.timestamp,
            "x": self.x if not self.interpolated else None,
            "y": self.y,
            "speed": self.speed,
            "acc": self.acceleration,
            "type": self.vehicle_type,
            "camera": self.camera_id if not self.interpolated else "INTERP",
            "interpolated": self.interpolated,
        }


@dataclass
class Trajectory:
    """一条完整轨迹（同一摄像头内的检测片段）。"""

    nodes: List[TrajectoryNode] = field(default_factory=list)
    camera_id: str = ""
    file_name: str = ""
    window_start: str = ""

    # ── 缓存属性 ──
    _majority_type: Optional[str] = field(default=None, repr=False)
    _is_static: Optional[bool] = field(default=None, repr=False)
    _valid_nodes: Optional[List[TrajectoryNode]] = field(default=None, repr=False)

    def add_node(self, node: TrajectoryNode) -> None:
        self.nodes.append(node)
        # 清除缓存
        self._majority_type = None
        self._is_static = None
        self._valid_nodes = None

    @property
    def valid_nodes(self) -> List[TrajectoryNode]:
        if self._valid_nodes is None:
            self._valid_nodes = [n for n in self.nodes if n.is_valid]
        return self._valid_nodes

    @property
    def node_count(self) -> int:
        return len(self.valid_nodes)

    @property
    def is_multi_node(self) -> bool:
        return self.node_count >= 2

    @property
    def majority_type(self) -> str:
        """多数投票类型。"""
        if self._majority_type is None:
            types = [n.vehicle_type for n in self.valid_nodes]
            if not types:
                self._majority_type = "unknown"
            else:
                self._majority_type = Counter(types).most_common(1)[0][0]
        return self._majority_type

    @property
    def is_static(self) -> bool:
        """静止轨迹检测：y 标准差 < 100cm 且 平均速度 < 5km/h。"""
        if self._is_static is None:
            vn = self.valid_nodes
            if len(vn) < 2:
                self._is_static = True
                return self._is_static

            y_vals = [n.y for n in vn]
            y_std = float(np.std(y_vals))

            speeds = [n.speed for n in vn if n.speed is not None]
            avg_speed = float(np.mean(speeds)) if speeds else 0.0

            self._is_static = (y_std < 100.0) and (avg_speed < 5.0)
        return self._is_static

    @property
    def start_time(self) -> int:
        return self.valid_nodes[0].timestamp if self.valid_nodes else 0

    @property
    def end_time(self) -> int:
        return self.valid_nodes[-1].timestamp if self.valid_nodes else 0

    @property
    def duration_ms(self) -> int:
        return self.end_time - self.start_time

    @property
    def y_start(self) -> float:
        return self.valid_nodes[0].y if self.valid_nodes else 0.0

    @property
    def y_end(self) -> float:
        return self.valid_nodes[-1].y if self.valid_nodes else 0.0

    @property
    def avg_speed_kmh(self) -> float:
        speeds = [n.speed for n in self.valid_nodes if n.speed is not None]
        return float(np.mean(speeds)) if speeds else 0.0

    def first_valid(self) -> Optional[TrajectoryNode]:
        return self.valid_nodes[0] if self.valid_nodes else None

    def last_valid(self) -> Optional[TrajectoryNode]:
        return self.valid_nodes[-1] if self.valid_nodes else None


@dataclass
class CameraDataset:
    """单摄像头数据集。"""

    camera_id: str
    trajectories: List[Trajectory] = field(default_factory=list)

    @property
    def y_min(self) -> float:
        return min(
            (t.valid_nodes[0].y for t in self.trajectories if t.valid_nodes),
            default=float("inf"),
        )

    @property
    def y_max(self) -> float:
        return max(
            (t.valid_nodes[-1].y for t in self.trajectories if t.valid_nodes),
            default=float("-inf"),
        )

    def get_active_trajs(self, min_nodes: int = 2, skip_static: bool = True) -> List[Trajectory]:
        """获取可用于匹配的活跃轨迹。"""
        result = []
        for t in self.trajectories:
            if t.node_count < min_nodes:
                continue
            if skip_static and t.is_static:
                continue
            result.append(t)
        return result

    def to_summary(self) -> dict:
        total = len(self.trajectories)
        multi = sum(1 for t in self.trajectories if t.is_multi_node)
        static = sum(1 for t in self.trajectories if t.is_static)
        return {
            "camera_id": self.camera_id,
            "total_tracks": total,
            "multi_node_tracks": multi,
            "static_tracks": static,
            "active_tracks": multi - static,
            "y_min": self.y_min,
            "y_max": self.y_max,
        }


@dataclass
class StitchedTrajectory:
    """拼接后的完整轨迹。"""

    trajectory_id: str
    nodes: List[TrajectoryNode] = field(default_factory=list)
    matched_pairs: List[tuple] = field(default_factory=list)  # (cam_a, cam_b, cost)
    quality_score: float = 0.0

    @property
    def vehicle_type(self) -> str:
        types = [n.vehicle_type for n in self.nodes if n.is_valid]
        return Counter(types).most_common(1)[0][0] if types else "unknown"

    @property
    def camera_count(self) -> int:
        cams = set(n.camera_id for n in self.nodes if n.camera_id and n.camera_id != "INTERP")
        return len(cams)

    @property
    def interp_ratio(self) -> float:
        if not self.nodes:
            return 0.0
        return sum(1 for n in self.nodes if n.interpolated) / len(self.nodes)

    def sort_nodes(self) -> None:
        self.nodes.sort(key=lambda n: n.timestamp)

    def to_dict(self) -> dict:
        return {
            "trajectory_id": self.trajectory_id,
            "vehicle_type": self.vehicle_type,
            "camera_count": self.camera_count,
            "interp_ratio": round(self.interp_ratio, 4),
            "quality_score": round(self.quality_score, 4),
            "nodes": [n.to_dict() for n in self.nodes],
            "matched_pairs": [
                [a, b, round(c, 2)] for a, b, c in self.matched_pairs
            ],
        }
