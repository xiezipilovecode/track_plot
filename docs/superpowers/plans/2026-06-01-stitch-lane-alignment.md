# Stitch Lane Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace x-coordinate-based world transform with lane-path lookup, so stitch vehicles drive on their correct lanes.

**Architecture:** New `tp_replay/lane_align.py` module handles XODR lane path extraction, x-to-lane tercile clustering, and lane point lookup. `tp_replay/main.py` stitch modes call it in Phase 3 to build wpts.

**Tech Stack:** Python 3, CARLA Python API, existing `tp_tunnel_traffic/lane_sampling.py` for XODR parsing

---

### Task 1: Create `tp_replay/lane_align.py` — lane path loading

**Files:**
- Create: `E:\code\track_plot\tp_replay\lane_align.py`

- [ ] **Step 1: Create lane_align.py with load_lane_paths()**

```python
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
```

- [ ] **Step 2: Run compileall to verify syntax**

Run: `python -m py_compile tp_replay/lane_align.py`
Expected: no output (success)

- [ ] **Step 3: Commit**

```
git add tp_replay/lane_align.py
git commit -m "feat(stitch): add lane_align.load_lane_paths — XODR lane path extraction"
```

---

### Task 2: Add x-to-lane clustering

**Files:**
- Modify: `E:\code\track_plot\tp_replay\lane_align.py`

- [ ] **Step 1: Add cluster_x_to_lanes()**

```python
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
```

- [ ] **Step 2: Run compileall**

Run: `python -m py_compile tp_replay/lane_align.py`

- [ ] **Step 3: Commit**

```
git add tp_replay/lane_align.py
git commit -m "feat(stitch): add lane_align.cluster_x_to_lanes — x tercile clustering"
```

---

### Task 3: Add lane assignment and path lookup

**Files:**
- Modify: `E:\code\track_plot\tp_replay\lane_align.py`

- [ ] **Step 1: Add assign_lane() and find_closest_lane_point()**

```python
def assign_lane(x: Optional[float], camera_id: str,
                clusters: Dict[str, Tuple[float, float]]) -> str:
    """根据 x 像素值 + camera_id 确定车道号。

    Returns: "-1", "-2", or "-3"
    """
    if x is None or camera_id not in clusters:
        return "-2"  # 默认中间车道
    t1, t2 = clusters[camera_id]
    if x < t1:
        return "-1"
    elif x < t2:
        return "-2"
    else:
        return "-3"


def find_closest_lane_point(world_y: float, lane_path: List[Tuple[float, float, float, float]]) -> Tuple[float, float, float, float]:
    """在车道路径中二分查找最接近 world_y 的路点。

    Args:
        world_y: CARLA 世界 Y 坐标
        lane_path: [(x, y, z, yaw), ...]，已按 y 升序

    Returns:
        (x, y, z, yaw) — 最近路点
    """
    ys = [p[1] for p in lane_path]
    idx = bisect.bisect_left(ys, world_y)
    if idx == 0:
        return lane_path[0]
    if idx >= len(lane_path):
        return lane_path[-1]
    prev_pt = lane_path[idx - 1]
    next_pt = lane_path[idx]
    if abs(prev_pt[1] - world_y) <= abs(next_pt[1] - world_y):
        return prev_pt
    return next_pt
```

- [ ] **Step 2: Run compileall**

Run: `python -m py_compile tp_replay/lane_align.py`

- [ ] **Step 3: Commit**

```
git add tp_replay/lane_align.py
git commit -m "feat(stitch): add assign_lane / find_closest_lane_point / node_to_world_loc"
```

---

### Task 4: Modify `_run_stitch_kinematic` Phase 3

**Files:**
- Modify: `E:\code\track_plot\tp_replay\main.py` — Phase 3 (lines ~192-254)

- [ ] **Step 1: Read current Phase 3 code to confirm line numbers**

Read `E:\code\track_plot\tp_replay\main.py` lines 160-255.

- [ ] **Step 2: Add lane_align import and initialization at start of function**

After line 159 (end of `_get_bp`), insert:

```python
    # ── 车道路径初始化 ──
    from .lane_align import load_lane_paths, cluster_x_to_lanes, node_to_world_loc

    try:
        lane_paths = load_lane_paths(config.XODR_PATH)
    except Exception as e:
        _info(f"警告：车道路径加载失败 ({e})，回退原方案")
        lane_paths = None

    # x→车道聚类（在过滤后、构建 wpts 前）
```

- [ ] **Step 3: Add clustering call after trajectory filtering**

After the `_info(f"加载 {len(filtered)} 条轨迹...")` line, insert:

```python
    clusters = cluster_x_to_lanes(filtered) if lane_paths else {}
```

- [ ] **Step 4: Modify Phase 3 wpts building loop to use lane_align**

Replace the current wpts building loop (lines ~198-227) with:

```python
    for t in filtered:
        nodes = t["nodes"]
        stime = nodes[0]["timestamp"] / 1000.0 - global_min_ts

        wpts = []
        last_lane = "-2"  # 插值节点默认车道
        for n in nodes:
            cam_id = n.get("camera_id", "")
            if cam_id == "INTERP":
                # 插值节点：沿用上一个实测节点的车道
                n_lane = last_lane
            elif lane_paths and clusters:
                x_val = n.get("x")
                n_lane = assign_lane(x_val, cam_id, clusters) if x_val is not None else last_lane
                last_lane = n_lane
            else:
                n_lane = None

            if n_lane is not None and lane_paths:
                lane_path = lane_paths.get(n_lane, [])
                if lane_path:
                    rx = (float(n.get("x") or ds[0]) - ds[0]) * sc
                    ry = (float(n.get("y", 0)) - ds[1]) * sc * sy
                    rough_y = rx * sa + ry * ca + engine.entry_loc.y + oy
                    px, py, pz, pyaw = find_closest_lane_point(rough_y, lane_path)
                    loc = carla.Location(px, py, pz)
                else:
                    loc = carla.Location(engine.entry_loc.x, engine.entry_loc.y, engine.entry_loc.z)
            else:
                # 回退：原坐标变换方案
                y_n = n["y"]
                x_n = n.get("x") or ds[0]
                rx = (x_n - ds[0]) * sc
                ry = (y_n - ds[1]) * sc * sy
                loc = carla.Location(
                    rx * ca - ry * sa + engine.entry_loc.x + ox,
                    rx * sa + ry * ca + engine.entry_loc.y + oy,
                    engine.entry_loc.z,
                )
                try:
                    wp = engine.map.get_waypoint(loc, project_to_road=True,
                                                  lane_type=carla.LaneType.Driving)
                    if wp: loc = wp.transform.location
                except Exception: pass
            wpts.append((loc, n.get("speed") or 0, n["timestamp"]))

        if len(wpts) < 2: continue
        states.append(_VS(t["trajectory_id"], wpts, stime, t.get("vehicle_type", "car")))
```

- [ ] **Step 5: Remove the old entrance protection code** (the first_valid/snapped/MIN_SPAWN_DIST_M logic, lines ~214-229 in current version) since lane paths always produce valid road positions.

- [ ] **Step 6: Run compileall to verify syntax**

Run: `python -m py_compile tp_replay/main.py`

- [ ] **Step 7: Commit**

```
git add tp_replay/main.py
git commit -m "feat(stitch): _run_stitch_kinematic Phase 3 uses lane_align for wpt construction"
```

---

### Task 5: Modify `_run_stitch_simple` Phase 3

**Files:**
- Modify: `E:\code\track_plot\tp_replay\main.py` — `_run_stitch_simple` function

- [ ] **Step 1: Read current _run_stitch_simple code**

Read `E:\code\track_plot\tp_replay\main.py` lines 37-132.

- [ ] **Step 2: Apply same lane_align integration as _run_stitch_kinematic**

Same pattern: add `load_lane_paths` at start, `cluster_x_to_lanes` for the selected trajectory, replace wpts building loop with `node_to_world_loc`.

The single-vehicle mode only has one trajectory, so clustering works on that one trajectory's nodes. The logic is identical — just the filter differs (single trajectory vs multi).

- [ ] **Step 3: Run compileall**

Run: `python -m py_compile tp_replay/main.py`

- [ ] **Step 4: Commit**

```
git add tp_replay/main.py
git commit -m "feat(stitch): _run_stitch_simple Phase 3 uses lane_align for wpt construction"
```

---

### Task 6: Integration test — single vehicle

- [ ] **Step 1: Run stitch_simple with lane alignment**

```bat
set TP_REPLAY_MODE=stitch_simple
set TP_STITCH_JSON_PATH=E:\code\track_plot\trace_data\track_stitch\output\stitched_trajectories.json
set TP_DATA_FILE_PATH=E:\code\track_plot\trace_data\test_data\data_6lu2.txt
set TP_STITCH_NTH=1
python auto_control_main.py
```

Expected:
- Console shows lane path loading success
- Vehicle spawns on correct lane (visually in driver view)
- Vehicle drives smoothly along waypoints
- Output: `单车测试: TRAJ_XXXXX nodes=NN 完成`

- [ ] **Step 2: Commit** (if new files or changes from debugging)

---

### Task 7: Integration test — multi vehicle

- [ ] **Step 1: Run stitch_kinematic with lane alignment**

```bat
set TP_REPLAY_MODE=stitch_kinematic
set TP_STITCH_JSON_PATH=E:\code\track_plot\trace_data\track_stitch\output\stitched_trajectories.json
set TP_DATA_FILE_PATH=E:\code\track_plot\trace_data\test_data\data_6lu2.txt
set TP_STITCH_TM_MAX_ACTIVE=30
set TP_STITCH_MAX_START_S=600
python auto_control_main.py
```

Expected:
- Multiple vehicles spawn on different lanes
- Vehicles follow their respective trajectories
- No vehicles fall through ground or spawn at entrance edge
- Console: `Time: X.Xs | Active: N | Spawned: N | Fail: 0 | Queue: N`

- [ ] **Step 2: Verify lane distribution**

Observe CARLA view: vehicles should be spread across left (-1), center (-2), and right (-3) lanes, not all on the same lane.

- [ ] **Step 3: Commit** (if any fixes)
