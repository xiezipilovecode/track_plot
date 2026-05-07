# DevKit

A lightweight toolkit for reading, validating, and visualizing the TAT dataset.

---

## 1. Offline Validation

```bat
conda activate carla
cd /d E:\code\track_plot

# Per‑run validation (images ↔ labels alignment)
python -m tp_tunnel_traffic.validate_dataset_run ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS

# COCO / instance output validation
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS

# Export inspection samples
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS ^
  --sample-cameras ego,front ^
  --sample-limit 3
```

---

## 2. Merge Global COCO

Combine all 61 runs into one `coco_annotations.json` ready for detection frameworks:

```bat
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset
```

Output: `dataset/coco_annotations.json` — 24,528 images, 27,763 annotations.

---

## 3. Reading Data (Python)

### 3.1 Load Global COCO

```python
import json

coco = json.load(open("dataset/coco_annotations.json"))
print(f"Images: {len(coco['images'])}, Annotations: {len(coco['annotations'])}")
# Images: 24528, Annotations: 27763
```

### 3.2 Load `labels.jsonl`

```python
import json
from pathlib import Path

def load_labels(run_dir: Path) -> list[dict]:
    labels = []
    with open(run_dir / "labels.jsonl") as f:
        for line in f:
            if line.strip():
                labels.append(json.loads(line))
    return labels

run = Path("dataset/proxy_1796/run_20260506_163611")
labels = load_labels(run)
print(f"{len(labels)} samples")  # 65
```

### 3.3 Align Images with Labels

```python
def get_image_path(run_dir: Path, camera: str, world_frame: int) -> Path:
    return run_dir / "images" / camera / f"{int(world_frame)}.png"

path = get_image_path(run, "front", labels[0]["world_frame"])
print(path.exists())  # True
```

### 3.4 Load Instance Mask

```python
from PIL import Image

mask_path = run / "images" / "front_instance" / f"{labels[0]['world_frame']}.png"
mask = Image.open(mask_path)
# Each pixel value = unique vehicle instance ID
```

---

## 4. Enumerate All Runs

```python
def discover_runs(dataset_root: Path) -> list[Path]:
    runs = []
    for proxy_dir in sorted(dataset_root.glob("proxy_*")):
        for run_dir in sorted(proxy_dir.glob("run_*")):
            runs.append(run_dir)
    return runs

runs = discover_runs(Path("dataset"))
print(f"Found {len(runs)} runs")  # 61
```

---

## 5. Auto Collection

The v2.0 dataset was collected across 6 batches with varied traffic density and speed.

| Env Variable | Default | Description |
|-------------|---------|-------------|
| `TT_AUTO_COLLECT_ENABLE` | `0` | Enable auto collection |
| `TT_AUTO_COLLECT_SECONDS_PER_RUN` | `180` | Duration per vehicle (seconds) |
| `TT_AUTO_COLLECT_MAX_RUNS` | `61` | Maximum runs |
| `TT_AUTO_COLLECT_MODE` | `balanced` | balanced / cycle / random |
| `TT_AUTO_COLLECT_MIN_SPEED_MPS` | `5.0` | Minimum speed threshold |
| `TT_AUTO_COLLECT_COOLDOWN_S` | `5` | Cooldown between runs |

Launch:

```bat
python -m tp_tunnel_traffic.tests.test_auto_collect
```

Outputs: `dataset/batch_summary.json`, `dataset/auto_collect_config.json`.

---

## 6. Environment Variables Reference

| Variable | Default | Purpose |
|----------|---------|---------|
| `TT_COLLECT_ENABLE` | — | Enable dataset collection |
| `TT_COLLECT_FRAME_STRIDE` | `10` | Write every N ticks |
| `TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION` | — | Generate instance mask PNGs |
| `TT_COLLECT_WRITE_COCO` | — | Generate COCO 2D JSON |
| `TT_COLLECT_INSTANCE_SUFFIX` | `_instance` | Instance camera suffix |
| `TT_COLLECT_COCO_MIN_AREA_PX2` | `200` | Minimum bbox area |
| `TT_COLLECT_COCO_MAX_HEIGHT_RATIO` | `0.9` | Max bbox height ratio |
