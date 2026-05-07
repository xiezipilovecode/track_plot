import argparse
import glob
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_single_file(path: str) -> pd.DataFrame:
    """Load one txt file.

    The raw file may have multiple records on one line. Each record is 6 tokens:
    timestamp, x, y, speed, accel, type.
    """
    with open(path, "r", encoding="utf-8") as f:
        tokens = f.read().split()

    if len(tokens) < 6:
        return pd.DataFrame(columns=["t", "x", "y", "v", "a", "cls", "file"])

    # Truncate tail if not divisible by 6
    n_rec = len(tokens) // 6
    tokens = tokens[: n_rec * 6]

    arr = np.array(tokens).reshape(-1, 6)
    df = pd.DataFrame(arr, columns=["t", "x", "y", "v", "a", "cls"])

    # Basic type conversion
    df["t"] = pd.to_numeric(df["t"], errors="coerce").astype("Int64")
    df["x"] = pd.to_numeric(df["x"], errors="coerce")
    df["y"] = pd.to_numeric(df["y"], errors="coerce")
    df["v"] = pd.to_numeric(df["v"], errors="coerce")
    df["a"] = pd.to_numeric(df["a"], errors="coerce")

    df["file"] = os.path.basename(path)

    # Drop rows without valid timestamp or position
    df = df.dropna(subset=["t", "x", "y"]).copy()

    # filter out obvious invalid points like (0, 0)
    mask_valid_pos = ~((df["x"] == 0.0) & (df["y"] == 0.0))
    df = df[mask_valid_pos]

    return df


def load_files(pattern: str, max_files: Optional[int] = None) -> pd.DataFrame:
    paths = sorted(glob.glob(pattern))
    if max_files is not None and max_files > 0:
        paths = paths[:max_files]

    if not paths:
        raise FileNotFoundError(f"No files matched pattern: {pattern}")

    dfs = [load_single_file(p) for p in paths]
    df = pd.concat(dfs, ignore_index=True)

    # Sort by time globally
    df = df.sort_values("t").reset_index(drop=True)

    # Normalize time and space for visualization
    t0 = df["t"].min()
    y0 = df["y"].min()
    df["t_rel"] = (df["t"] - t0) / 1000.0  # seconds
    df["y_rel"] = df["y"] - y0
    return df


@dataclass
class TrackState:
    track_id: int
    t_last: int
    y_last: float
    v_last: float
    cls: str


def _predict_y(track: TrackState, t: int, v_meas: float) -> float:
    """Predict longitudinal position using simple constant-velocity model."""

    dt = (t - track.t_last) / 1000.0
    if dt <= 0:
        return track.y_last

    v_use = track.v_last if not np.isnan(track.v_last) else v_meas
    return track.y_last + v_use * dt


def assign_track_ids(
    df: pd.DataFrame,
    max_time_gap: float = 2.0,
    max_space_gap: float = 150.0,
    same_type: bool = True,
) -> pd.DataFrame:
    """More refined multi-target linking based on (t, y, v).

    相比之前的逐点贪心，本方法按时间戳分组，在每个时间步内进行
    一对一匹配（一个轨迹在同一时间只匹配到一个观测），可以有效
    减少串轨现象。
    """

    df = df.sort_values("t").reset_index(drop=True).copy()

    # 活跃轨迹集合
    active_tracks: Dict[int, TrackState] = {}
    next_track_id = 0
    assigned_track_ids = np.full(len(df), -1, dtype=int)

    # 按时间分组，每个时间步做一轮关联
    for t_val, group in df.groupby("t", sort=True):
        idxs = group.index.to_list()

        # 预先清理时间过久未更新的轨迹
        to_remove: List[int] = []
        for tid, state in active_tracks.items():
            dt = (t_val - state.t_last) / 1000.0
            if dt <= 0 or dt > max_time_gap:
                to_remove.append(tid)
        for tid in to_remove:
            del active_tracks[tid]

        if not active_tracks:
            # 当前没有活跃轨迹，则本时间步所有观测都新建轨迹
            for idx in idxs:
                row = df.loc[idx]
                v_meas = row["v"] if not pd.isna(row["v"]) else 0.0
                tid = next_track_id
                next_track_id += 1
                active_tracks[tid] = TrackState(
                    track_id=tid,
                    t_last=int(row["t"]),
                    y_last=float(row["y"]),
                    v_last=float(v_meas),
                    cls=str(row["cls"]),
                )
                assigned_track_ids[idx] = tid
            continue

        # 构造候选 (track, obs) 及代价值
        candidates: List[Tuple[int, int, float]] = []  # (tid, idx, cost)
        for tid, state in active_tracks.items():
            for idx in idxs:
                row = df.loc[idx]
                if same_type and str(row["cls"]) != state.cls:
                    continue
                v_meas = row["v"] if not pd.isna(row["v"]) else 0.0
                y_pred = _predict_y(state, int(row["t"]), float(v_meas))
                cost = abs(float(row["y"]) - y_pred)
                if cost <= max_space_gap:
                    candidates.append((tid, idx, cost))

        # 若没有任何候选，则本时间步所有观测均开启新轨迹
        if not candidates:
            for idx in idxs:
                row = df.loc[idx]
                v_meas = row["v"] if not pd.isna(row["v"]) else 0.0
                tid = next_track_id
                next_track_id += 1
                active_tracks[tid] = TrackState(
                    track_id=tid,
                    t_last=int(row["t"]),
                    y_last=float(row["y"]),
                    v_last=float(v_meas),
                    cls=str(row["cls"]),
                )
                assigned_track_ids[idx] = tid
            continue

        # 按 cost 从小到大排序，做一对一贪心匹配
        candidates.sort(key=lambda x: x[2])
        used_tracks: set[int] = set()
        used_obs: set[int] = set()

        for tid, idx, _ in candidates:
            if tid in used_tracks or idx in used_obs:
                continue
            # 选中这一对
            used_tracks.add(tid)
            used_obs.add(idx)

            row = df.loc[idx]
            v_meas = row["v"] if not pd.isna(row["v"]) else 0.0
            state = active_tracks[tid]
            # 更新轨迹状态
            state.t_last = int(row["t"])
            state.y_last = float(row["y"])
            state.v_last = float(v_meas)
            state.cls = str(row["cls"])
            assigned_track_ids[idx] = tid

        # 剩余未匹配的观测各自新建轨迹
        for idx in idxs:
            if idx in used_obs:
                continue
            row = df.loc[idx]
            v_meas = row["v"] if not pd.isna(row["v"]) else 0.0
            tid = next_track_id
            next_track_id += 1
            active_tracks[tid] = TrackState(
                track_id=tid,
                t_last=int(row["t"]),
                y_last=float(row["y"]),
                v_last=float(v_meas),
                cls=str(row["cls"]),
            )
            assigned_track_ids[idx] = tid

    df["track_id"] = assigned_track_ids
    return df


def plot_spacetime_before(df: pd.DataFrame, out_path: str, vehicle_type: Optional[str] = None) -> None:
    data = df
    if vehicle_type is not None:
        data = data[data["cls"] == vehicle_type]

    if data.empty:
        print("No data to plot for 'before' plot.")
        return

    plt.figure(figsize=(8, 6))
    # Color by source file
    for fname, g in data.groupby("file"):
        plt.scatter(g["t_rel"], g["y_rel"], s=4, label=fname, alpha=0.6)

    plt.xlabel("time (s, relative)")
    plt.ylabel("longitudinal position y (relative)")
    plt.title("Space-time plot BEFORE stitching (colored by file)")
    if data["file"].nunique() <= 10:
        plt.legend(fontsize=6)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()


def plot_spacetime_after(df: pd.DataFrame, out_path: str, vehicle_type: Optional[str] = None) -> None:
    data = df
    if vehicle_type is not None:
        data = data[data["cls"] == vehicle_type]

    if data.empty:
        print("No data to plot for 'after' plot.")
        return

    plt.figure(figsize=(8, 6))
    # Color by track_id
    for tid, g in data.groupby("track_id"):
        plt.plot(g["t_rel"], g["y_rel"], linewidth=0.8, alpha=0.7)

    plt.xlabel("time (s, relative)")
    plt.ylabel("longitudinal position y (relative)")
    plt.title("Space-time plot AFTER stitching (colored by track)")
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Stitch multi-camera trajectories and plot space-time diagrams.")
    parser.add_argument(
        "--pattern",
        type=str,
        default="K615+703TV023_2024-07-25-*.txt",
        help="Glob pattern to select input txt files.",
    )
    parser.add_argument(
        "--max-files",
        type=int,
        default=0,
        help="Use at most this many files; <=0 means use all.",
    )
    parser.add_argument(
        "--vehicle-type",
        type=str,
        default="all",
        help="Filter by vehicle type (e.g. 'car', 'truck'); set to 'all' to disable.",
    )
    parser.add_argument(
        "--output-prefix",
        type=str,
        default="demo",
        help="Prefix for output PNG files.",
    )
    parser.add_argument(
        "--output-data-dir",
        type=str,
        default="stitched_data",
        help="Directory to save stitched trajectory data.",
    )
    parser.add_argument(
        "--output-fig-dir",
        type=str,
        default="plots",
        help="Directory to save before/after space-time plots.",
    )
    args = parser.parse_args()

    df = load_files(args.pattern, args.max_files)

    vtype = None if args.vehicle_type == "all" else args.vehicle_type

    os.makedirs(args.output_data_dir, exist_ok=True)
    os.makedirs(args.output_fig_dir, exist_ok=True)

    before_path = os.path.join(args.output_fig_dir, f"{args.output_prefix}_spacetime_before.png")
    after_path = os.path.join(args.output_fig_dir, f"{args.output_prefix}_spacetime_after.png")

    print(f"Loaded {len(df)} points from files matching {args.pattern}")
    print(f"Files involved: {df['file'].nunique()}")

    plot_spacetime_before(df, before_path, vehicle_type=vtype)

    df_stitched = assign_track_ids(df)

    # 画拼接后的时空图
    plot_spacetime_after(df_stitched, after_path, vehicle_type=vtype)

    # 保存拼接后的完整点集
    stitched_path = os.path.join(args.output_data_dir, f"{args.output_prefix}_stitched_all.csv")
    df_stitched.to_csv(stitched_path, index=False)

    print(f"Saved before-stitch plot to {before_path}")
    print(f"Saved after-stitch plot to {after_path}")
    print(f"Saved stitched trajectories to {stitched_path}")


if __name__ == "__main__":
    main()
