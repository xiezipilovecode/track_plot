# Data Format

<div class="tat-lead">
TAT follows a clean, run‑granular organization inspired by KITTI and nuScenes. All data is keyed by <code>world_frame</code> — a single integer that aligns images, labels, instance masks, and COCO annotations across all sensors.
</div>

---

## 1. Directory Structure

```text
dataset/
  coco_annotations.json              ← 全局合并 COCO (所有 61 runs)
  batch_summary.json                 ← 批量采集统计
  auto_collect_config.json           ← 续跑状态
  proxy_<actor_id>/
    run_YYYYmmdd_HHMMSS/
      images/
        ego/              (65 PNG, 800×600)
        front/            (65 PNG)
        front_left/       (65 PNG)
        front_right/      (65 PNG)
        left/             (65 PNG)
        right/            (65 PNG)
        rear/             (65 PNG)
        ego_instance/     (65 PNG instance masks)
        front_instance/   (65 PNG)
        ... (7 total instance dirs)
      labels.jsonl                   (65 samples, JSON Lines)
      labels_2d/
        coco_instances.json          (per‑run COCO)
      metadata.json
      target.json
      effective_config.json          (optional)
```

<div class="tat-info-box">
  <div class="box-title">💡 Run‑level Organization</div>
  Each <code>run_*</code> represents a continuous ~180‑second collection segment from one proxy vehicle. With 65 frames per run, this provides enough temporal context for sequence models while keeping each run self‑contained for clean train/val/test splitting.
</div>

---

## 2. Synchronization

All data aligned by `world_frame` (CARLA snapshot ID):

```python
frame = label.get("world_frame", label.get("frame"))
fname = f"{int(frame)}.png"         # → "14253298.png"
path  = f"images/{camera}/{fname}"
```

> `world_frame` is a counter, not a wall‑clock timestamp. For offline time‑step, use `metadata.json → fixed_delta_seconds`.

---

## 3. `labels.jsonl`

JSON Lines — one JSON object per line (65 lines per run). Each line captures the full ego‑vehicle state at one `world_frame`.

| Field | Type | Description |
|-------|------|-------------|
| `world_frame` | `int` | CARLA snapshot frame ID (primary key) |
| `frame` | `int` | Synonym (backward compatibility) |
| `actor_id` | `int` | Collected proxy vehicle |
| `throttle` | `float` | ∈ [0, 1] |
| `steer` | `float` | ∈ [‑1, 1] (negative = left) |
| `brake` | `float` | ∈ [0, 1] |
| `hand_brake` | `bool` | |
| `reverse` | `bool` | |
| `vehicle_x/y/z` | `float` | World position (meters, CARLA coordinate frame) |
| `vehicle_yaw` | `float` | Yaw in degrees (available in v2.0 runs) |
| `speed_mps` | `float` | Speed in m/s (available in v2.0 runs) |

---

## 4. COCO 2D Detection

### 4.1 Detection Samples

Below are real COCO annotations from proxy_1796 (frame 14253298 — 38 total bboxes across 7 cameras):

<div class="tat-img-grid split-64">
  <div class="tat-img-figure">
    <img src="/coco-detection-front.png" alt="COCO 2D bboxes on front camera" />
    <div class="caption"><strong>front</strong> — 9 vehicles detected</div>
  </div>
  <div class="tat-img-figure">
    <img src="/coco-detection-front_left.png" alt="COCO 2D bboxes on front_left camera" />
    <div class="caption"><strong>front_left</strong> — 9 vehicles detected</div>
  </div>
</div>

### 4.2 Global Merged COCO (`coco_annotations.json`) ⭐

All 61 runs merged into a single COCO‑format file — ready to feed directly into any detection framework:

| Statistic | Value |
|-----------|-------|
| COCO images | 24,528 |
| COCO annotations | 27,763 |
| Categories | 1 (`vehicle`, id=1) |
| Avg. annotations/image | 1.13 |
| File size | ~5 MB (compressed JSON) |

Generate it yourself from the raw dataset:

```bat
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset
```

The merged file uses globally unique `image_id` and `annotation_id` values. Image paths are relative to the dataset root (`proxy_<id>/run_<ts>/images/<cam>/<frame>.png`).

### COCO JSON Structure

```json
{
  "images": [
    {"id":1, "file_name":"proxy_1796/.../images/ego/14253298.png",
     "width":800, "height":600, "frame":14253298, "camera":"ego"}
  ],
  "annotations": [
    {"id":1, "image_id":1, "category_id":1,
     "bbox":[285, 220, 95, 80], "area":7600}
  ],
  "categories": [
    {"id":1, "name":"vehicle"}
  ]
}
```

---

## 5. `metadata.json`

Run‑level metadata. Key fields:

| Field | Type | Description |
|-------|------|-------------|
| `xodr_path` | `str` | OpenDRIVE map path (QingShiLing.xodr) |
| `cameras` | `list` | Camera mount poses + imaging parameters |
| `collect_frame_stride` | `int` | Sample every N simulator ticks |
| `fixed_delta_seconds` | `float` | Simulation time step (sync mode) |
| `sync_mode` | `bool` | Sync mode enabled? |
| `target_actor_id` | `int` | Collected proxy vehicle actor ID |

---

## 6. Limitations

- ❌ No per‑frame wall‑clock timestamps — `world_frame` counter only
- ❌ No matrix‑form camera intrinsics/extrinsics — pose + fov provided instead
- ❌ Camera‑only — no LiDAR, radar, or depth sensors
- ✅ 2D bboxes + instance masks fully provided for all 24,528 frames

---

## 7. Validation

```bat
conda activate carla
cd /d E:\code\track_plot

# Per‑run validation
python -m tp_tunnel_traffic.validate_dataset_run ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS

# COCO / instance validation
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
```
