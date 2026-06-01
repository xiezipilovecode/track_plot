"""拼接轨迹全量质量验证工具。

用法:
    python check_traj.py --input stitched_trajectories.json [--output anomalous.json]
"""

from __future__ import annotations
import argparse, json, sys
from collections import Counter


def check_y_monotonic(nodes):
    revs = []
    for i in range(len(nodes) - 1):
        if nodes[i + 1]["y"] < nodes[i]["y"]:
            revs.append((i, nodes[i]["y"], nodes[i + 1]["y"]))
    return revs


def check_ts_monotonic(nodes):
    revs = []
    for i in range(len(nodes) - 1):
        if nodes[i + 1]["timestamp"] <= nodes[i]["timestamp"]:
            revs.append((i, nodes[i]["timestamp"], nodes[i + 1]["timestamp"]))
    return revs


def check_speed_anomaly(nodes, max_kmh=200.0):
    bad = []
    for i, n in enumerate(nodes):
        spd = n.get("speed")
        if spd is not None and spd > max_kmh:
            bad.append((i, spd))
    return bad


def check_cam_order(nodes):
    """摄像头应遵循 TV023→TV024→TV025→TV026→TV027→TV028 顺序。"""
    order = {"TV023": 0, "TV024": 1, "TV025": 2, "TV026": 3, "TV027": 4, "TV028": 5}
    prev_idx = -1
    bad = []
    for i, n in enumerate(nodes):
        cam = n.get("camera", "")
        if cam == "INTERP" or cam not in order:
            continue
        idx = order[cam]
        if idx < prev_idx:
            bad.append((i, cam))
        prev_idx = max(prev_idx, idx)
    return bad


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input", required=True, help="拼接轨迹 JSON 路径")
    p.add_argument("--output", help="异常轨迹输出路径")
    p.add_argument("--max-rev", type=int, default=3, help="允许的 y 反向次数上限")
    args = p.parse_args()

    with open(args.input, "r", encoding="utf-8") as f:
        data = json.load(f)
    trajs = data.get("trajectories", data if isinstance(data, list) else [])
    print(f"加载 {len(trajs)} 条轨迹")

    anomalies = []
    stats = {"total": len(trajs), "y_rev": 0, "ts_rev": 0, "speed_bad": 0,
             "cam_order_bad": 0, "interp_high": 0}

    for t in trajs:
        tid = t.get("trajectory_id", "?")
        nodes = t.get("nodes", [])
        if not nodes:
            continue

        y_revs = check_y_monotonic(nodes)
        ts_revs = check_ts_monotonic(nodes)
        speed_bad = check_speed_anomaly(nodes)
        cam_bad = check_cam_order(nodes)

        n_measured = sum(1 for n in nodes if not n.get("interpolated", False))
        n_interp = sum(1 for n in nodes if n.get("interpolated", False))
        interp_ratio = n_interp / max(len(nodes), 1)

        issues = {}
        if len(y_revs) > args.max_rev:
            issues["y_rev"] = len(y_revs)
            stats["y_rev"] += 1
        if ts_revs:
            issues["ts_rev"] = len(ts_revs)
            stats["ts_rev"] += 1
        if speed_bad:
            issues["speed_bad"] = len(speed_bad)
            stats["speed_bad"] += 1
        if cam_bad:
            issues["cam_order_bad"] = len(cam_bad)
            stats["cam_order_bad"] += 1
        if interp_ratio > 0.9:
            issues["interp_ratio"] = f"{interp_ratio:.2f}"
            stats["interp_high"] += 1

        if issues:
            anomalies.append({"trajectory_id": tid, "issues": issues,
                              "nodes": len(nodes), "quality": t.get("quality_score", 0)})

    print(f"\n验证结果:")
    print(f"  总轨迹: {stats['total']}")
    print(f"  y反向超标(>{args.max_rev}次): {stats['y_rev']}")
    print(f"  时间戳反向: {stats['ts_rev']}")
    print(f"  速度异常(>200km/h): {stats['speed_bad']}")
    print(f"  摄像头顺序异常: {stats['cam_order_bad']}")
    print(f"  插值比过高(>0.9): {stats['interp_high']}")
    print(f"  异常轨迹总数: {len(anomalies)}")

    if args.output and anomalies:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(anomalies, f, ensure_ascii=False, indent=2)
        print(f"\n异常轨迹已导出: {args.output}")


if __name__ == "__main__":
    main()
