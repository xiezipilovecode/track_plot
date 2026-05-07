# Tasks & Benchmarks

<div class="tat-lead">
TAT v2.0 supports <strong>four recommended tasks</strong> spanning perception and control. With 24,528 annotated frames and 27,763 COCO bboxes, the dataset provides sufficient scale for training production‑grade models. A formal leaderboard is planned for a future release.
</div>

---

## Dataset Scale for ML

| Aspect | Detail |
|--------|--------|
| Total samples (labels.jsonl) | 3,965 (65 per run × 61 runs) |
| Image samples (all cameras) | 24,528 |
| Instance mask samples | 24,528 |
| COCO annotations | 27,763 |
| Recommended split | 70/15/15 by run (43 train / 9 val / 9 test) |
| Training images (train × 7 cams) | ~19,565 |
| Training COCO bboxes | ~19,434 |

<div class="tat-highlight">
  <strong>Split Rule:</strong> Always split by run, never by frame. A single <code>run_*</code> must be entirely in train, val, or test. This prevents temporal leakage — frames within a run are only seconds apart and highly correlated.
</div>

---

## 1. Behavior Cloning (BC)

Learn a direct mapping from camera pixels to vehicle controls. This is the primary end‑to‑end task supported by TAT.

| Aspect | Detail |
|--------|--------|
| **Input** | Single camera image (800×600) or multi‑camera stack |
| **Output** | `steer` ∈ [‑1, 1], `throttle` ∈ [0, 1], `brake` ∈ [0, 1] |
| **Training samples** | ~19,565 images (train split × 7 cameras) |
| **Metrics** | MSE / MAE per control channel; lane‑keeping success rate in closed‑loop simulation |
| **Split** | By run — prevents temporal information leakage |

<div class="tat-narrative">
  <strong>Suggested approach:</strong> Start with a single front‑camera CNN (ResNet‑18/50 backbone), then experiment with multi‑view fusion. Tunnel walls and lane markings provide strong geometric priors that multi‑view architectures can exploit for improved lateral control.
</div>

---

## 2. 2D Vehicle Detection

Detect vehicles in tunnel images using standard COCO‑compatible frameworks.

| Aspect | Detail |
|--------|--------|
| **Input** | RGB image (800×600) |
| **Output** | Bounding boxes: `[x, y, w, h]` per vehicle |
| **Format** | COCO JSON — per‑run or global merged `coco_annotations.json` |
| **Classes** | 1 (`vehicle`) |
| **Training bboxes** | ~19,434 (train split) |
| **Compatible frameworks** | Detectron2, MMDetection, YOLO, DETR, Faster R‑CNN |

<div class="tat-info-box">
  <div class="box-title">🚀 Quick Start with Detectron2</div>

```python
# Load the global merged COCO
from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.data.datasets import register_coco_instances

register_coco_instances("tat_train", {},
    "dataset/coco_annotations.json",  # ← global merged file
    "dataset"                          # image root
)
```

The global `coco_annotations.json` is plug‑and‑play — no additional preprocessing needed.
</div>

---

## 3. Instance Segmentation

Predict per‑pixel vehicle instance masks from RGB input.

| Aspect | Detail |
|--------|--------|
| **Input** | RGB image (800×600) |
| **Output** | Per‑pixel instance mask (unique ID per vehicle) |
| **Training masks** | ~19,565 (train split) |
| **Ground truth format** | PNG under `images/<camera>_instance/` |
| **Compatible frameworks** | Mask R‑CNN, YOLACT, SOLO, Mask2Former |

<div class="tat-narrative">
  Instance masks provide dense supervision for segmentation models. Combined with the multi‑view setup, this enables training models that can reason about vehicle shapes from multiple viewpoints — particularly valuable in tunnels where occlusions and lighting variations are common.
</div>

---

## 4. Speed & Yaw Regression (Auxiliary)

| Aspect | Detail |
|--------|--------|
| **Input** | Single or multi‑camera image |
| **Output** | `speed_mps` (m/s), `vehicle_yaw` (degrees) |
| **Metrics** | RMSE, Pearson correlation |
| **Use case** | Auxiliary loss for BC models; standalone ego‑state estimator |

---

## Future Plans

- **Leaderboard** — formal submission format + automated evaluation script.
- **Multi‑view 3D detection** — once intrinsic/extrinsic matrices are exported in matrix form.
- **Trajectory prediction** — temporal modeling across `world_frame` sequences.
- **Additional runs** — expanding beyond the initial 61 vehicles.
