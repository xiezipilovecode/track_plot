# Download

Data packs organized by component. Figures reflect the **v2.0 release** (61 vehicles, 24,528 images).

::: warning Terms
All downloads require acceptance of the dataset license (research / non‑commercial use only). See [License & Citation](./license).
:::

---

## Dataset Scale (v2.0)

| Metric | Value |
|--------|-------|
| Vehicles (proxy actors) | 61 |
| Runs | 61 |
| RGB cameras | 7 per vehicle |
| Frames per camera | 3,504 |
| Frames per run | 65 |
| **Total RGB images** | **24,528** |
| **Instance masks** | **24,528** |
| **COCO annotations** | **27,763** |
| Avg. run duration | ~180 seconds |
| Collection date | 2026‑05‑06 19:26 (UTC+8) |
| Collection batches | 6 (varied density/speed) |

---

## Data Packs

| Package | Content | Notes |
|---------|---------|-------|
| **TAT‑Images‑RGB** | All `images/` — 7 cameras × 3,504 frames | 24,528 PNG (800×600). One archive per run recommended |
| **TAT‑Instance‑Masks** | All `images/*_instance/` — 7 cameras × 3,504 masks | 24,528 PNG masks, 100% paired with RGB |
| **TAT‑Labels** | `labels.jsonl`, `metadata.json`, `target.json` (×61 runs) | ~KB per run |
| **TAT‑COCO** | `coco_annotations.json` (global merged) + per‑run `labels_2d/*.json` | 27,763 bboxes, 1 category (vehicle) |
| **TAT‑Config** | `effective_config.json`, `batch_summary.json`, `auto_collect_config.json` | Reproducibility snapshots |

---

## Splitting Policy

Split **by run**, not by frame:

✅ **Correct**: `run_20260506_163611` → entirely in train  
❌ **Wrong**: partial frames from one run in train, rest in val

Recommended split (61 runs):

| Split | Runs | Frames (per camera) | % |
|-------|------|---------------------|---|
| Train | 43 | ~2,795 | ~70% |
| Val | 9 | ~585 | ~15% |
| Test | 9 | ~585 | ~15% |

---

## Verification

```bat
conda activate carla
cd /d E:\code\track_plot

python -m tp_tunnel_traffic.validate_dataset_run ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
```

---

## Download Status

| Pack | Status |
|------|--------|
| TAT‑Images‑RGB | 🟡 Preparing |
| TAT‑Instance‑Masks | 🟡 Preparing |
| TAT‑Labels | 🟡 Preparing |
| TAT‑COCO | 🟡 Preparing |
| TAT‑Config | 🟡 Preparing |
