#!/usr/bin/env python
"""Generate sample visualizations for the TAT dataset website (v2.0 batch).

Output images (in website/docs/public/):
  - instance-comparison.png     : ego + front + front_right, RGB vs Instance Mask
  - coco-detection-front.png    : front camera with COCO bboxes
  - coco-detection-front_right.png : front_right with most bboxes
  - multi-view-grid.png         : all 7 cameras at one frame in a 4x2 grid
"""

import json
from pathlib import Path
from collections import defaultdict
from PIL import Image, ImageDraw

# ── Config ──────────────────────────────────────────────────────
RUN_DIR = Path(r"dataset\proxy_1796\run_20260506_163611")
OUT_DIR = Path(r"website\docs\public")
CAMERAS = ["ego", "front", "front_left", "front_right", "left", "right", "rear"]
FRAME_MAIN   = 14253298   # first frame, 38 bboxes across 7 cameras
FRAME_MIDDLE = 14253598   # middle frame for variety

OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Load COCO ───────────────────────────────────────────────────
with open(RUN_DIR / "labels_2d" / "coco_instances.json") as f:
    coco = json.load(f)

ann_by_image = defaultdict(list)
for ann in coco["annotations"]:
    ann_by_image[ann["image_id"]].append(ann)

key_to_meta = {(img["camera"], img["frame"]): img for img in coco["images"]}


def get_image_path(camera, frame):
    return RUN_DIR / "images" / camera / f"{int(frame)}.png"


def get_instance_path(camera, frame):
    return RUN_DIR / "images" / f"{camera}_instance" / f"{int(frame)}.png"


def draw_bboxes(pil_img, image_id):
    anns = ann_by_image.get(image_id, [])
    if not anns:
        return pil_img
    img = pil_img.copy()
    draw = ImageDraw.Draw(img)
    for ann in anns:
        x, y, w, h = ann["bbox"]
        draw.rectangle([x, y, x + w, y + h], outline="#00FF00", width=2)
    return img


def make_label_strip(text, width, height=28, bg=(42, 82, 118), fg=(255, 255, 255)):
    strip = Image.new("RGB", (width, height), bg)
    draw = ImageDraw.Draw(strip)
    draw.text((8, 6), text, fill=fg)
    return strip


# ═══════════════════════════════════════════════════════════════
#  1. Instance Segmentation Comparison
# ═══════════════════════════════════════════════════════════════
print("1. Generating instance-comparison.png ...")
comp_cameras = ["ego", "front", "front_right"]
frame = FRAME_MIDDLE

thumb_w = 400
gap = 4
label_h = 28

rgb_imgs = []
ins_imgs = []
for cam in comp_cameras:
    rgb = Image.open(get_image_path(cam, frame)).convert("RGB")
    ins = Image.open(get_instance_path(cam, frame)).convert("RGB")
    w, h = rgb.size
    th = int(h * thumb_w / w)
    rgb_imgs.append(rgb.resize((thumb_w, th), Image.LANCZOS))
    ins_imgs.append(ins.resize((thumb_w, th), Image.LANCZOS))

actual_h = rgb_imgs[0].size[1]
cols = len(comp_cameras)
canvas_w = cols * thumb_w + (cols - 1) * gap
canvas_h = label_h + actual_h + gap + label_h + actual_h
canvas = Image.new("RGB", (canvas_w, canvas_h), (255, 255, 255))

for col in range(cols):
    x = col * (thumb_w + gap)
    label_rgb = make_label_strip(f"{comp_cameras[col]} (RGB)", thumb_w)
    canvas.paste(label_rgb, (x, 0))
    canvas.paste(rgb_imgs[col], (x, label_h))
    label_ins = make_label_strip(f"{comp_cameras[col]} (Instance Mask)", thumb_w, bg=(100, 120, 60))
    y_ins_label = label_h + actual_h + gap
    canvas.paste(label_ins, (x, y_ins_label))
    canvas.paste(ins_imgs[col], (x, y_ins_label + label_h))

canvas.save(OUT_DIR / "instance-comparison.png", "PNG")
print(f"  -> {OUT_DIR / 'instance-comparison.png'}")


# ═══════════════════════════════════════════════════════════════
#  2. COCO Detection Visualization
# ═══════════════════════════════════════════════════════════════
print("2. Generating coco-detection-*.png ...")
frame = FRAME_MAIN

# Check which cameras have the most bboxes
cam_counts = {}
for cam in ["front", "front_right", "front_left"]:
    key = (cam, frame)
    if key in key_to_meta:
        n = len(ann_by_image.get(key_to_meta[key]["id"], []))
        cam_counts[cam] = n
        print(f"  {cam}: {n} bboxes at frame {frame}")

# Pick top 2
top_cams = sorted(cam_counts, key=lambda c: -cam_counts[c])[:2]

for cam in top_cams:
    key = (cam, frame)
    img_meta = key_to_meta[key]
    n_bboxes = cam_counts[cam]
    img = Image.open(get_image_path(cam, frame)).convert("RGB")
    img = draw_bboxes(img, img_meta["id"])
    display_w = 800
    w, h = img.size
    display_h = int(h * display_w / w)
    img = img.resize((display_w, display_h), Image.LANCZOS)
    out_name = f"coco-detection-{cam}.png"
    img.save(OUT_DIR / out_name, "PNG")
    print(f"  -> {OUT_DIR / out_name} ({n_bboxes} bboxes)")


# ═══════════════════════════════════════════════════════════════
#  3. Multi-View Grid (all 7 cameras)
# ═══════════════════════════════════════════════════════════════
print("3. Generating multi-view-grid.png ...")
frame = FRAME_MIDDLE

thumb_w = 260
gap = 4
label_h = 26

loaded = []
for cam in CAMERAS:
    p = get_image_path(cam, frame)
    if p.exists():
        img = Image.open(p).convert("RGB")
        w, h = img.size
        th = int(h * thumb_w / w)
        loaded.append((cam, img.resize((thumb_w, th), Image.LANCZOS)))

if loaded:
    cols = 4
    rows = 2
    actual_h = loaded[0][1].size[1]
    canvas_w = cols * thumb_w + (cols - 1) * gap
    canvas_h = rows * (label_h + actual_h) + (rows - 1) * gap
    canvas = Image.new("RGB", (canvas_w, canvas_h), (255, 255, 255))

    for idx, (cam, img) in enumerate(loaded):
        row = idx // cols
        col = idx % cols
        if row >= rows:
            break
        x = col * (thumb_w + gap)
        y = row * (label_h + actual_h + gap) + label_h
        label = make_label_strip(f"{cam} (frame {frame})", thumb_w)
        canvas.paste(label, (x, y - label_h))
        canvas.paste(img, (x, y))

    out_path = OUT_DIR / "multi-view-grid.png"
    canvas.save(out_path, "PNG")
    print(f"  -> {out_path} ({len(loaded)} cameras)")

print("\n=== Done ===")
