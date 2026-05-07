from __future__ import annotations

"""Merge per-run COCO files into a single unified dataset-level COCO file.

Usage:
    python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset

Output:
    dataset/coco_annotations.json
"""

import argparse
import json
from pathlib import Path


def _collect_coco_files(dataset_dir: Path) -> list[Path]:
    """Find all per-run coco_instances.json files."""
    candidates = sorted(dataset_dir.glob("proxy_*/run_*/labels_2d/coco_instances.json"))
    return [c for c in candidates if c.is_file()]


def _make_dataset_relative(dataset_dir: Path, abs_path: Path) -> str:
    """Convert an absolute path to dataset-relative."""
    try:
        rel = abs_path.relative_to(dataset_dir)
    except ValueError:
        return str(abs_path).replace("\\", "/")
    return str(rel).replace("\\", "/")


def _build_file_path_map(
    dataset_dir: Path,
    coco_files: list[Path],
) -> dict[str, str]:
    """Build a canonical mapping of absolute path -> dataset-relative path.

    Keys are all possible forms of the image paths we might encounter:
    - Absolute paths
    - Run-relative paths (e.g. images/ego/000123.png)
    """
    mapping: dict[str, str] = {}
    for cf in coco_files:
        run_dir = cf.parent.parent  # .../run_*/labels_2d -> .../run_*/
        with open(cf, "r", encoding="utf-8") as f:
            data = json.load(f)
        for img in data.get("images", []):
            file_name = img.get("file_name", "")
            if not file_name:
                continue
            # Try multiple resolution strategies
            candidates: list[Path] = []
            fp = Path(file_name)
            if fp.is_absolute():
                candidates.append(fp)
            candidates.append(run_dir / fp)
            candidates.append(dataset_dir / fp)
            candidates.append(dataset_dir / run_dir.name / fp)
            resolved = None
            for p in candidates:
                if p.exists():
                    resolved = p
                    break
            if resolved is None:
                # Best effort: resolve from run_dir
                resolved = run_dir / fp
            rel = _make_dataset_relative(dataset_dir, resolved)
            # Store under the original file_name key for fast lookup
            mapping[file_name] = rel
    return mapping


def merge_coco(dataset_dir: Path, output_path: Path) -> dict:
    coco_files = _collect_coco_files(dataset_dir)
    if not coco_files:
        print("No per-run COCO files found.")
        return {}

    print(f"Found {len(coco_files)} per-run COCO files")

    merged = {
        "images": [],
        "annotations": [],
        "categories": [
            {"id": 1, "name": "vehicle"},
        ],
    }
    image_id_offset = 0
    annotation_id_offset = 0

    for cf in coco_files:
        run_dir = cf.parent.parent
        with open(cf, "r", encoding="utf-8") as f:
            data = json.load(f)

        run_images = data.get("images", [])
        run_annotations = data.get("annotations", [])

        # Build old-id -> new-id map for this run
        old_to_new_image: dict[int, int] = {}

        for img in run_images:
            old_id = int(img.get("id", 0))
            image_id_offset += 1
            new_id = image_id_offset
            old_to_new_image[old_id] = new_id

            file_name = img.get("file_name", "")
            fp = Path(file_name)
            if fp.is_absolute():
                real_path = fp
            else:
                real_path = run_dir / fp
            if not real_path.exists():
                real_path = run_dir / fp

            rel_path = _make_dataset_relative(dataset_dir, real_path)

            merged["images"].append({
                "id": new_id,
                "file_name": rel_path,
                "width": int(img.get("width", 0)),
                "height": int(img.get("height", 0)),
                "frame": int(img.get("frame", 0)),
                "camera": str(img.get("camera", "")),
            })

        for ann in run_annotations:
            old_image_id = int(ann.get("image_id", 0))
            if old_image_id not in old_to_new_image:
                continue
            annotation_id_offset += 1
            merged["annotations"].append({
                "id": annotation_id_offset,
                "image_id": old_to_new_image[old_image_id],
                "category_id": int(ann.get("category_id", 1)),
                "bbox": list(ann.get("bbox", [])),
                "area": float(ann.get("area", 0)),
                "iscrowd": int(ann.get("iscrowd", 0)),
                "actor_id": int(ann.get("actor_id", -1)),
            })

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(merged, f, ensure_ascii=False)

    print(f"Merged: {len(merged['images'])} images, {len(merged['annotations'])} annotations")
    print(f"Output: {output_path}")
    return merged


def main():
    ap = argparse.ArgumentParser(description="Merge per-run COCO files into one dataset-level file.")
    ap.add_argument(
        "--dataset-dir",
        default="dataset",
        help="Path to dataset directory (default: dataset).",
    )
    ap.add_argument(
        "--output",
        default=None,
        help="Output path (default: <dataset-dir>/coco_annotations.json).",
    )
    args = ap.parse_args()

    dataset_dir = Path(args.dataset_dir).resolve()
    if not dataset_dir.is_dir():
        print(f"ERROR: Not a directory: {dataset_dir}")
        return

    output_path = Path(args.output) if args.output else (dataset_dir / "coco_annotations.json")
    merge_coco(dataset_dir, output_path)


if __name__ == "__main__":
    main()
