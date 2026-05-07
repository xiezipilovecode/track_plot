---
layout: home
hero:
  name: "TAT Dataset"
  text: TunnelAutopilot‑Tunnel
  tagline: 61 vehicles · 3,504 frames/camera · 24,528 RGB images · 24,528 instance masks · 27,763 COCO bboxes
  image:
    src: /muti-view-sample.png
    alt: TAT multi-view camera sample
  actions:
    - theme: brand
      text: Browse Data Format
      link: /data-format
    - theme: alt
      text: Download Dataset
      link: /download
features:
  - icon: 📊
    title: 61 Vehicles · 61 Runs
    details: Collected across 6 batches with varied traffic density and speed. Each run captures ~180 seconds of autonomous driving in a realistic 3‑lane CARLA tunnel — a total of over 3 hours of driving data.
  - icon: 📷
    title: 7‑Camera Surround View
    details: Bird`s‑eye (ego), forward, forward‑left/right, side‑left/right, and rear — all hard‑synchronized by <code>world_frame</code>. 3,504 frames per camera provide rich spatial context for behavior cloning and 3D perception.
  - icon: 🎯
    title: Instance Segmentation (24K Masks)
    details: Pixel‑level vehicle instance masks paired with <strong>every</strong> RGB frame — 24,528 masks with 100% coverage. Each vehicle instance labeled with a unique color ID, ready for segmentation model training.
  - icon: 📦
    title: COCO 2D Detection (27K Bboxes)
    details: 27,763 bounding boxes across all frames. Available as per‑run COCO files and a global merged <code>coco_annotations.json</code>. Plug‑and‑play with Detectron2, MMDetection, YOLO, and other COCO‑compatible frameworks.
  - icon: 🕹️
    title: Full Control Supervision
    details: Per‑frame steer / throttle / brake, ego world pose (x, y, z, yaw), and speed (m/s). 65 labeled frames per run provide dense control signals for behavior cloning and control regression.
  - icon: 🤖
    title: Automated Collection Pipeline
    details: 6‑batch collection with configurable density/speed ranges. Fully reproducible — <code>batch_summary.json</code> tracks every run, <code>auto_collect_config.json</code> enables pause‑and‑resume.
---

## Key Statistics

<div class="tat-stats-grid">
  <div class="tat-stat-card">
    <div class="stat-number">61</div>
    <div class="stat-label">Vehicles<br/>6 collection batches</div>
  </div>
  <div class="tat-stat-card">
    <div class="stat-number">24,528</div>
    <div class="stat-label">RGB Images<br/>800×600 px</div>
  </div>
  <div class="tat-stat-card">
    <div class="stat-number">24,528</div>
    <div class="stat-label">Instance Masks<br/>100% paired</div>
  </div>
  <div class="tat-stat-card">
    <div class="stat-number">27,763</div>
    <div class="stat-label">COCO Bboxes<br/>Avg. 1.13 / image</div>
  </div>
</div>

---

## About the Dataset

<div class="tat-narrative">

**TunnelAutopilot‑Tunnel (TAT)** is a camera‑only autonomous driving dataset collected in a realistic CARLA 3‑lane tunnel environment. Unlike sunny‑day highway benchmarks, TAT captures the unique perceptual challenges of tunnel driving:

- **Low and uneven illumination** — tunnel lighting creates sharp shadows and glare spots that challenge vision models.
- **Structural repetition** — uniform walls, lane markings, and ceiling patterns make feature matching and localization difficult.
- **Dynamic multi‑vehicle traffic** — 61 proxy vehicles drive simultaneously with realistic car‑following and lane‑changing behaviors.

The dataset provides **7 synchronized RGB cameras** per vehicle, **instance segmentation masks** for every frame, and **27,763 COCO 2D bounding boxes** across all runs. A global merged COCO file makes it trivial to plug into standard detection frameworks.

</div>

<div class="tat-info-box">
  <div class="box-title">📖 Collection Methodology</div>
  Data was collected using an <strong>automated balanced strategy</strong>: the system iterates through proxy vehicles, records ~180 seconds of driving per vehicle, then switches to the next. Six batches were run with varied traffic density and vehicle speed ranges to ensure behavioral diversity. All collection parameters are recorded in <code>batch_summary.json</code> and <code>auto_collect_config.json</code> for full reproducibility.
</div>

---

## Detailed Statistics

| Statistic | Value |
|-----------|-------|
| **Total runs** | 61 |
| **Vehicles** | 61 (1 run per vehicle) |
| **RGB cameras** | 7 per vehicle |
| **Frames per camera** | 3,504 |
| **Frames per run** | 65 |
| **Total RGB images** | 24,528 |
| **Instance masks** | 24,528 (100% paired) |
| **COCO annotations** | 27,763 |
| **Avg. annotations/image** | 1.13 |
| **Avg. run duration** | ~180 seconds |
| **Image resolution** | 800 × 600 px |
| **Collection date** | 2026‑05‑06 19:26 (UTC+8) |
| **Collection duration** | ~3 hours (6 batches) |
| **Simulator** | CARLA (sync mode, `fixed_delta_seconds`) |
| **Map** | QingShiLing.xodr — 3‑lane tunnel |

---

## Visual Overview

<div class="tat-img-grid cols-2" style="margin-top:0">
  <div class="tat-img-figure">
    <img src="/multi-view-grid.png" alt="All 7 cameras" loading="lazy" />
    <div class="caption">7 synchronized cameras — proxy_1796, frame 14253598</div>
  </div>
  <div class="tat-img-figure">
    <img src="/instance-comparison.png" alt="Instance segmentation comparison" loading="lazy" />
    <div class="caption">RGB vs instance segmentation masks — ego, front, front_right</div>
  </div>
  <div class="tat-img-figure">
    <img src="/coco-detection-front.png" alt="COCO detection front — 9 vehicles" loading="lazy" />
    <div class="caption">COCO 2D detection — front view (9 vehicles)</div>
  </div>
  <div class="tat-img-figure">
    <img src="/coco-detection-front_left.png" alt="COCO detection front_left — 9 vehicles" loading="lazy" />
    <div class="caption">COCO 2D detection — front‑left view (9 vehicles)</div>
  </div>
</div>

---

## News

- **2026‑05‑06** — v2.0 release. 61 vehicles across 6 batches, 24.5K RGB + instance masks + 27.7K COCO bboxes. Global merged `coco_annotations.json` available for detection frameworks.
- **2026‑05‑06 (earlier)** — v1.0 pilot batch. 84 vehicles, 13K images (now superseded by v2.0).

## Quick Links

- **[Data Format](./data-format)** — Full schema, COCO annotations (per‑run + global merged), and field tables.
- **[Sensors](./sensors)** — Camera specs, instance segmentation, and multi‑view samples.
- **[Download](./download)** — Data packs and train/val/test split (70/15/15 by run).
- **[DevKit](./devkit)** — Python snippets for reading, validating, and merging COCO.
- **[Tasks](./tasks)** — Behavior Cloning, 2D detection, instance segmentation benchmarks.

## Citation

```bibtex
@misc{tat2026,
  title        = {{TunnelAutopilot-Tunnel (TAT) Dataset}},
  author       = {{TAT Dataset Contributors}},
  year         = {2026},
  howpublished = {\url{https://your-website-url}},
  note         = {61‑vehicle multi‑view tunnel driving dataset with 24K+ instance masks and 27K+ COCO 2D annotations.},
}
```

<br />

---

<div style="text-align:center;color:var(--tat-text-light);font-size:.9rem;margin-top:2rem">
  <em>Inspired by the KITTI Vision Benchmark Suite — clarity, reproducibility, and academic rigor.</em>
</div>
