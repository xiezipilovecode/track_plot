from __future__ import annotations

"""Check COCO/instance outputs for a dataset run.

Usage (CMD):
    python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\\proxy_<id>\\run_YYYYmmdd_HHMMSS

Optional:
    --sample-cameras ego,front
    --sample-limit 3
"""

import argparse
import json
from pathlib import Path
"""(Pillow is optional; only needed for overlay rendering.)"""


def _parse_run_dir(run_dir: str) -> Path:
    p = Path(run_dir)
    if not p.exists():
        raise FileNotFoundError(str(p))
    return p


def _load_coco(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _group_annotations(annotations: list[dict]) -> dict[int, list[dict]]:
    grouped: dict[int, list[dict]] = {}
    for ann in annotations:
        image_id = ann.get("image_id")
        if image_id is None:
            continue
        grouped.setdefault(int(image_id), []).append(ann)
    return grouped


def _normalize_image_path(run_dir: Path, file_name: str) -> Path:
    path = Path(file_name)
    if path.is_absolute():
        run_str = str(run_dir)
        path_str = str(path)
        if run_str in path_str:
            remainder = path_str.split(run_str, 1)[-1]
            if run_str in remainder:
                remainder = remainder.split(run_str, 1)[-1]
            remainder = remainder.lstrip("\\/")
            if remainder:
                return Path(remainder)
        try:
            return path.relative_to(run_dir)
        except Exception:
            return path
    if str(run_dir) in str(path):
        try:
            return Path(str(path).split(str(run_dir), maxsplit=1)[-1].lstrip("\\/"))
        except Exception:
            return path
    return path


def _resolve_image_path(run_dir: Path, file_name: str) -> Path:
    normalized = _normalize_image_path(run_dir, file_name)
    if normalized.is_absolute():
        return normalized
    return run_dir / normalized


def _validate_instance_images(run_dir: Path, images: list[dict]) -> list[str]:
    missing = []
    for img in images:
        file_name = img.get("file_name")
        if not file_name:
            continue
        p = _resolve_image_path(run_dir, file_name)
        if not p.exists():
            missing.append(str(p))
    return missing


def _validate_instance_pairs(run_dir: Path, images: list[dict], instance_suffix: str) -> list[str]:
    missing_pairs = []
    images_root = run_dir / "images"
    existing_instance_dirs = {
        p.name for p in images_root.iterdir() if p.is_dir() and p.name.endswith(instance_suffix)
    } if images_root.exists() else set()
    for img in images:
        camera = img.get("camera")
        if not camera or camera.endswith(instance_suffix):
            continue
        frame = img.get("frame")
        if frame is None:
            continue
        instance_camera = f"{camera}{instance_suffix}"
        if instance_camera not in existing_instance_dirs:
            continue
        inst_path = run_dir / "images" / instance_camera / f"{int(frame):06d}.png"
        if not inst_path.exists():
            missing_pairs.append(str(inst_path))
    return missing_pairs


def _write_sample_json(
    run_dir: Path,
    images: list[dict],
    annotations_by_image: dict[int, list[dict]],
    cameras: list[str],
    limit: int,
) -> Path:
    counts: dict[str, int] = {c: 0 for c in cameras}
    samples: list[dict] = []
    for img in images:
        camera = img.get("camera")
        if camera not in cameras:
            continue
        if counts[camera] >= limit:
            continue
        file_name = img.get("file_name")
        if not file_name:
            continue
        image_path = _resolve_image_path(run_dir, file_name)
        if not image_path.exists():
            continue
        entry = {
            "image": str(image_path).replace("\\", "/"),
            "frame": int(img.get("frame", 0)),
            "camera": camera,
            "bboxes": [ann.get("bbox") for ann in annotations_by_image.get(int(img.get("id", -1)), [])],
        }
        samples.append(entry)
        counts[camera] += 1
    out_path = run_dir / "labels_2d" / "inspect_samples.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(samples, f, ensure_ascii=False, indent=2)
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser(description="Validate COCO and instance segmentation outputs for one run.")
    ap.add_argument("--run-dir", required=True, help="Run directory path, e.g. dataset/proxy_123/run_YYYYmmdd_HHMMSS")
    ap.add_argument(
        "--sample-cameras",
        default="",
        help="Comma-separated camera names to export sample bboxes (e.g. ego,front).",
    )
    ap.add_argument("--sample-limit", type=int, default=3, help="Max samples per camera.")
    ap.add_argument(
        "--instance-suffix",
        default="_instance",
        help="Instance camera suffix (default: _instance).",
    )
    args = ap.parse_args()

    run_dir = _parse_run_dir(args.run_dir)
    coco_path = run_dir / "labels_2d" / "coco_instances.json"
    if not coco_path.exists():
        print(f"ERROR: COCO not found: {coco_path}")
        return 2

    coco = _load_coco(coco_path)
    images = coco.get("images") or []
    annotations = coco.get("annotations") or []
    categories = coco.get("categories") or []
    print(f"COCO images: {len(images)}, annotations: {len(annotations)}, categories: {len(categories)}")
    if images and annotations:
        avg = float(len(annotations)) / float(len(images))
        print(f"Average annotations per image: {avg:.3f}")

    annotations_by_image = _group_annotations(annotations)

    missing_images = _validate_instance_images(run_dir, images)
    if missing_images:
        print(f"Missing image files: {len(missing_images)}")
        print("Example:", missing_images[:5])
        return 3

    missing_pairs = _validate_instance_pairs(run_dir, images, args.instance_suffix)
    if missing_pairs:
        print(f"Missing instance images: {len(missing_pairs)}")
        print("Example:", missing_pairs[:5])
        return 4

    if args.sample_cameras:
        cameras = [c.strip() for c in args.sample_cameras.split(",") if c.strip()]
        if cameras:
            sample_path = _write_sample_json(run_dir, images, annotations_by_image, cameras, args.sample_limit)
            print(f"Sample file written: {sample_path}")

    print("OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
