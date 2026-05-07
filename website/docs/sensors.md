# Sensors

<div class="tat-lead">
The TAT dataset captures <strong>7 synchronized RGB cameras</strong> per vehicle, providing full surround‑view coverage of the tunnel environment. Every camera frame comes with a corresponding <strong>pixel‑level instance segmentation mask</strong> — 24,528 masks in total, 100% paired.
</div>

---

## Multi‑View Sample (7 Cameras)

All 7 synchronized cameras at a single `world_frame` (frame 14253598, proxy_1796, run_20260506_163611):

<div class="tat-img-full">
  <img src="/multi-view-grid.png" alt="All 7 cameras at one frame" />
  <div class="caption">All 7 synchronized cameras — ego, front, front_left, front_right, left, right, rear (frame 14253598)</div>
</div>

<div class="tat-narrative">
The multi‑view configuration provides comprehensive spatial awareness: the <strong>forward‑facing cameras</strong> (front, front‑left, front‑right) capture the road ahead and adjacent lanes; the <strong>side cameras</strong> (left, right) monitor lateral traffic; the <strong>rear camera</strong> tracks following vehicles; and the <strong>ego (bird`s‑eye) camera</strong> provides a top‑down view useful for localization and lane‑keeping analysis.
</div>

---

## RGB vs. Instance Segmentation

Every RGB frame has a corresponding pixel‑level instance mask. Below are three camera views with their instance masks — each color represents a distinct vehicle instance.

<div class="tat-img-full">
  <img src="/instance-comparison.png" alt="RGB vs Instance Segmentation comparison" />
  <div class="caption">Top row: RGB camera views. Bottom row: Instance segmentation masks (each color = one vehicle instance). All from frame 14253598.</div>
</div>

<div class="tat-highlight">
  <strong>100% Coverage:</strong> All 24,528 RGB images have matching instance segmentation masks — no missing frames, no partial coverage. This makes TAT immediately usable for supervised instance segmentation training.
</div>

---

## Camera Suite

| Camera | Directory | Orientation | Frames (total) | Primary Use Case |
|--------|-----------|-------------|:---:|-------|
| Ego (overhead) | `ego` | Top‑down / bird`s‑eye | 3,504 | Localization, lane‑keeping visualization |
| Front | `front` | Forward | 3,504 | Primary driving perspective |
| Front‑Left | `front_left` | Forward‑left (~45°) | 3,504 | Left lane & blind spot coverage |
| Front‑Right | `front_right` | Forward‑right (~45°) | 3,504 | Right lane & blind spot coverage |
| Left | `left` | Left side (~90°) | 3,504 | Lateral awareness |
| Right | `right` | Right side (~90°) | 3,504 | Lateral awareness |
| Rear | `rear` | Backward | 3,504 | Following traffic monitoring |

**Total**: 7 cameras × 3,504 frames = **24,528 RGB images + 24,528 instance masks** (61 runs × 65 frames/run).

---

## Camera Properties (from `metadata.json`)

Each camera entry in `metadata.json → cameras[]` provides mount pose and imaging parameters:

| Field | Type | Meaning |
|-------|------|---------|
| `name` | `str` | Camera name (matches image subdirectory) |
| `x`, `y`, `z` | `float` | Mount position relative to vehicle body (meters) |
| `pitch` | `float` | Pitch angle (degrees) |
| `yaw` | `float` | Yaw angle (degrees) |
| `roll` | `float` | Roll angle (degrees) |
| `width` | `int` | Image width (px) — 800 |
| `height` | `int` | Image height (px) — 600 |
| `fov` | `float` | Horizontal field‑of‑view (degrees) |

> For geometric or sensor‑fusion tasks that require matrix‑form intrinsics, approximate `K` (pinhole model) as:

```
fx = width  / (2 * tan(fov / 2))
fy = height / (2 * tan(fov / 2))
cx = width  / 2
cy = height / 2
```

---

## Instance Segmentation

Each run includes 7 `*_instance/` directories — one per RGB camera — containing PNG masks where each pixel value encodes a unique vehicle instance ID.

| Attribute | Value |
|-----------|-------|
| Format | PNG (800×600, 24‑bit RGB) |
| Encoding | Per‑pixel instance ID (color‑coded by COCO mapping) |
| Coverage | 24,528 masks — 100% of RGB frames |
| Generation | `TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1` |
| Suffix convention | `TT_COLLECT_INSTANCE_SUFFIX=_instance` |

<div class="tat-info-box">
  <div class="box-title">🔍 Reading Instance Masks in Python</div>

```python
from PIL import Image
import numpy as np

mask = np.array(Image.open("images/front_instance/14253598.png"))
unique_ids = np.unique(mask)
print(f"Vehicles in frame: {len(unique_ids) - 1}")  # subtract background (0)
```

The color‑to‑instance‑ID mapping is defined in the COCO annotations — each `annotation` in `coco_instances.json` links an instance to its bounding box.
</div>
