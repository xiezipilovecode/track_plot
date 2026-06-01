# Stitch Pipeline Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix P0 defects (multi-start assembly, configurable y-reversal tolerance), P1/P2 fixes, dead code removal, and rewrite validation tool.

**Architecture:** Modify 4 existing files (config.py, models.py, stitcher.py, check_traj.py). No new files. Core stitching algorithm (parser, matcher, interpolation logic) unchanged.

**Tech Stack:** Python 3, scipy (linear_sum_assignment), numpy, existing project conventions.

---

### Task 1: Fix config.py — add max_y_reversals, remove dead config

**Files:**
- Modify: `E:\code\track_plot\trace_data\track_stitch\config.py`

- [ ] **Step 1: Add `max_y_reversals` field to `StitchingConfig`**

After line 71 (after `within_cam_min_overlap_nodes`), add:

```python
    # ── y 方向容忍度 ──
    max_y_reversals: int = 3   # 允许的 y 坐标反向次数（0=严格单调）
```

- [ ] **Step 2: Remove dead config fields**

Remove these unused fields from `StitchingConfig`:
- `within_cam_min_overlap_nodes: int = 1` (line ~69)
- `interp_small_gap_ms: int = 5_000` (line ~76)
- `batch_size: int = 1_000` (line ~80)
- `output_unmatched: str = "unmatched_fragments.json"` (line ~85)

- [ ] **Step 3: Run syntax check**

Run: `python -m py_compile trace_data/track_stitch/config.py`
Expected: no output (success)

- [ ] **Step 4: Commit**

```bash
git add trace_data/track_stitch/config.py
git commit -m "fix(stitch): add max_y_reversals config, remove dead fields"
```

---

### Task 2: Fix models.py — y_min/y_max + is_static

**Files:**
- Modify: `E:\code\track_plot\trace_data\track_stitch\models.py`

- [ ] **Step 1: Fix `CameraDataset.y_min` and `y_max`**

Read lines 191-206. Replace the property bodies to iterate all valid_nodes of all trajectories:

```python
    @property
    def y_min(self) -> float:
        vals = []
        for t in self.trajectories:
            for n in t.valid_nodes:
                if n.y is not None:
                    vals.append(n.y)
        return min(vals) if vals else 0.0

    @property
    def y_max(self) -> float:
        vals = []
        for t in self.trajectories:
            for n in t.valid_nodes:
                if n.y is not None:
                    vals.append(n.y)
        return max(vals) if vals else 0.0
```

- [ ] **Step 2: Fix `is_static` — ddof=1**

Find `np.std(y_vals)` on line 148, change to:

```python
        y_vals = np.array([n.y for n in nodes if n.y is not None])
        if len(y_vals) < 2:
            return True
        std_y = float(np.std(y_vals, ddof=1))
        return std_y < 100.0 and (speeds[-1] or 0) < 5.0
```

- [ ] **Step 3: Run syntax check**

Run: `python -m py_compile trace_data/track_stitch/models.py`
Expected: no output

- [ ] **Step 4: Commit**

```bash
git add trace_data/track_stitch/models.py
git commit -m "fix(stitch): fix y_min/y_max to iterate all nodes, fix is_static ddof=1"
```

---

### Task 3: Fix stitcher.py — multi-start assembly + max_rev + quality + dead code

**Files:**
- Modify: `E:\code\track_plot\trace_data\track_stitch\stitcher.py`

- [ ] **Step 1: Replace `_assemble_trajectories` with multi-start version**

Read lines 145-228. Replace the entire method:

```python
    def _assemble_trajectories(self, match_graph, all_matches):
        """多起点轨迹组装：从每个摄像头出发独立组装，去重后返回。"""
        all_chains = []
        for start_cam in self.config.cameras:
            if start_cam not in match_graph:
                continue
            for start_key in match_graph[start_cam]:
                chain = self._build_chain(start_key, match_graph, all_matches)
                if chain and len(chain) >= 2:
                    stitched = self._chain_to_stitched(chain, all_matches)
                    if stitched and _check_y_monotonic(stitched, max_rev=self.config.max_y_reversals):
                        self._compute_quality(stitched, all_matches)
                        all_chains.append(stitched)
        # 去重：时间戳重叠度 > 0.5 的两条链保留质量分更高的
        all_chains.sort(key=lambda s: s.quality_score, reverse=True)
        keep = []
        for c in all_chains:
            c_ts = set(n.timestamp for n in c.nodes)
            dup = False
            for k in keep:
                k_ts = set(n.timestamp for n in k.nodes)
                overlap = len(c_ts & k_ts) / max(len(c_ts), len(k_ts), 1)
                if overlap > 0.5:
                    dup = True
                    break
            if not dup:
                keep.append(c)
        logger.info("Multi-start assembly: %d chains -> %d after dedup", len(all_chains), len(keep))
        return keep
```

Note: Also remove the old lines 183 that called `_check_y_monotonic(stitched, max_rev=0)` — the new code above uses `self.config.max_y_reversals` instead.

- [ ] **Step 2: Fix quality score — use config camera count**

Read `_compute_quality` method (lines 327-347). Replace with:

```python
    def _compute_quality(self, stitched, all_matches):
        cam_n = len(self.config.cameras)
        if cam_n == 0:
            stitched.quality_score = 0.0
            return
        camera_count = stitched.camera_count
        coverage = min(1.0, camera_count / cam_n)
        interp_ratio = stitched.interp_ratio
        continuity = 1.0 - interp_ratio
        costs = [m["cost"] for m in stitched.matched_pairs if m["cost"] < 1e8]
        avg_cost = sum(costs) / max(len(costs), 1)
        match_conf = 1.0 / (1.0 + avg_cost / 1000.0)
        stitched.quality_score = 0.5 * coverage + 0.3 * continuity + 0.2 * match_conf
```

- [ ] **Step 3: Add post-interpolation monotonicity check**

In `_interpolate_gaps` (line 265), after the interpolation loop, add:

```python
        # 插值后重新校验单调性
        if stitched.nodes:
            if not _check_y_monotonic(stitched, max_rev=self.config.max_y_reversals):
                logger.warning("Trajectory %s non-monotonic after interpolation, discarding", stitched.trajectory_id)
                return 0
```

- [ ] **Step 4: Remove dead code — `unmatched_fragments`**

Find `"unmatched_fragments": 0` in `__init__` (line ~42) and remove the entire line. Also remove it from `export()` if present.

- [ ] **Step 5: Run syntax check**

Run: `python -m py_compile trace_data/track_stitch/stitcher.py`
Expected: no output

- [ ] **Step 6: Commit**

```bash
git add trace_data/track_stitch/stitcher.py
git commit -m "fix(stitch): multi-start assembly + configurable max_rev + fixed quality scoring + post-interp check"
```

---

### Task 4: Rewrite check_traj.py — full validation tool

**Files:**
- Modify: `E:\code\track_plot\trace_data\track_stitch\check_traj.py`

- [ ] **Step 1: Replace entire file content**

Replace the entire content of `check_traj.py` with:

```python
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
```

- [ ] **Step 2: Run syntax check**

Run: `python -m py_compile trace_data/track_stitch/check_traj.py`
Expected: no output

- [ ] **Step 3: Commit**

```bash
git add trace_data/track_stitch/check_traj.py
git commit -m "feat(stitch): rewrite check_traj.py — full scan with y_rev/ts_rev/speed/cam_order/interp checks"
```

---

### Task 5: Full pipeline test — quick mode

- [ ] **Step 1: Run quick mode to verify pipeline**

```bash
cd E:\code\track_plot
python -m trace_data.track_stitch.main --quick
```

Expected: No crashes. Output shows multi-start assembly stats and dedup info. Trajectory count should increase from previous ~28K.

- [ ] **Step 2: Run validation on output**

```bash
python trace_data/track_stitch/check_traj.py --input trace_data/track_stitch/output/stitched_trajectories.json
```

Expected: Summary stats showing anomaly counts. No crashes.

- [ ] **Step 3: Commit** (if any fixes applied during testing)
