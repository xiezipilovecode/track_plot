"""轨迹拼接配置。

采用 @dataclass 模式，与 tp_tunnel_traffic/config.py 风格一致。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class StitchingConfig:
    """轨迹拼接引擎全部配置参数。"""

    # ── 数据路径 ──
    data_root: str = (
        r"E:\code\track_plot\trace_data\track24.7.25-24.8.5_79_6lu"
    )
    output_dir: str = (
        r"E:\code\track_plot\trace_data\track_stitch\output"
    )

    # ── 六路摄像头（按隧道里程排序） ──
    cameras: List[str] = field(default_factory=lambda: [
        "TV023",  # K615+703
        "TV024",  # K615+833
        "TV025",  # K615+963
        "TV026",  # K616+093
        "TV027",  # K616+223
        "TV028",  # K616+323
    ])

    # ── 文件过滤 ──
    time_window_minutes: int = 30     # 匹配时间窗口（分钟）
    file_pattern: str = "*.txt"

    # ── 轨迹过滤 ──
    min_nodes_per_track: int = 2      # 最少有效节点数（<2=不参与匹配）
    skip_static_tracks: bool = True   # 跳过静止轨迹
    static_y_std_cm: float = 100.0    # 静止判定：y标准差阈值(cm)
    static_speed_kmh: float = 5.0     # 静止判定：平均速度阈值(km/h)

    # ── 节点过滤 ──
    skip_zero_zero_nodes: bool = True # 过滤 (0,0) 异常点

    # ── 车辆类型（7种） ──
    vehicle_types: List[str] = field(default_factory=lambda: [
        "car", "truck", "tanker", "van", "bus", "pika", "motorbike",
    ])

    # ── 速度/坐标转换 ──
    speed_kmh_to_cm_ms: float = 1.0 / 36.0  # km/h → cm/ms
    cm_ms_to_kmh: float = 36.0              # cm/ms → km/h

    # ── 跨摄像头匹配 ──
    max_time_gap_ms: int = 120_000    # 最大时间差（ms）= 2分钟
    max_y_gap_cm: float = 25_000.0    # 最大y坐标差（cm）= 250m
    max_acceptable_cost: float = 5_000.0  # 匈牙利匹配代价上限

    # ── 代价函数权重 ──
    weight_time: float = 1.0          # 时间差权重
    weight_position: float = 1.0      # 位置差权重
    weight_speed: float = 0.5         # 速度一致性权重
    weight_type: float = 10.0         # 类型惩罚系数
    penalty_type_mismatch: float = 50.0  # 类型不匹配固定惩罚

    # ── 同摄像头跨文件合并 ──
    within_cam_max_time_gap_ms: int = 60_000   # 窗口间最大间隙(ms)
    within_cam_max_y_gap_cm: float = 5_000.0   # 最大y差(cm)
    # ── y 方向容忍度 ──
    max_y_reversals: int = 3   # 允许的 y 坐标反向次数（0=严格单调）

    # ── 盲区插值 ──
    interp_max_gap_ms: int = 30_000   # 最大插值间隙(ms)，超过标记低置信
    interp_points_per_gap: int = 10   # 每间隙插值点数
    # ── 性能 ──
    max_files_to_scan: int = 0       # 快速扫描模式（0=全部）

    # ── 输出 ──
    output_json: str = "stitched_trajectories.json"
    output_stats: str = "stitching_stats.json"

    @property
    def camera_pairs(self) -> List[tuple]:
        """返回 5 对相邻摄像头。"""
        return [
            (self.cameras[i], self.cameras[i + 1])
            for i in range(len(self.cameras) - 1)
        ]
