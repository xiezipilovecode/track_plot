from __future__ import annotations

import argparse
import json
from pathlib import Path


def _iter_jsonl(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            s = line.strip()
            if not s:
                continue
            try:
                yield line_no, json.loads(s)
            except Exception as e:
                raise ValueError(f"Invalid JSONL at {path} line {line_no}: {e}")


def _validate_run_dir(run_dir: Path) -> int:
    if not run_dir.exists():
        raise FileNotFoundError(str(run_dir))

    labels_path = run_dir / "labels.jsonl"
    meta_path = run_dir / "metadata.json"
    cfg_path = run_dir / "effective_config.json"
    images_dir = run_dir / "images"

    if not meta_path.exists():
        raise FileNotFoundError(str(meta_path))
    if not images_dir.exists():
        raise FileNotFoundError(str(images_dir))

    if not cfg_path.exists():
        print(f"WARN: effective_config.json missing: {cfg_path}")
    else:
        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            if not isinstance(cfg, dict):
                raise ValueError("effective_config.json must be a JSON object")
        except Exception as e:
            raise ValueError(f"Invalid effective_config.json: {cfg_path}: {e}")

    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)
    cameras = meta.get("cameras") or []
    cam_names = [str(c.get("name")) for c in cameras if isinstance(c, dict) and c.get("name")]
    cam_names = [n for n in cam_names if n]
    if not cam_names:
        raise ValueError(f"No cameras listed in metadata: {meta_path}")

    if not labels_path.exists():
        print(f"WARN: labels.jsonl missing: {labels_path}")
        # Still validate that image folders exist
        missing_folders = [n for n in cam_names if not (images_dir / n).exists()]
        if missing_folders:
            raise FileNotFoundError(f"Missing camera folders: {missing_folders}")
        print("OK: metadata + image folders exist (no labels to validate)")
        return 0

    label_count = 0
    missing_images = 0
    total_expected = 0
    actor_ids = set()
    frames = []

    for _, label in _iter_jsonl(labels_path):
        if not isinstance(label, dict):
            continue
        label_count += 1
        frame = label.get("world_frame", label.get("frame"))
        if frame is None:
            continue
        try:
            frame_i = int(frame)
        except Exception:
            continue
        frames.append(frame_i)
        try:
            actor_ids.add(int(label.get("actor_id", -1)))
        except Exception:
            pass

        filename = f"{frame_i:06d}.png"
        for cam in cam_names:
            total_expected += 1
            p = images_dir / cam / filename
            if not p.exists():
                missing_images += 1

    if frames:
        frames_sorted = sorted(frames)
        span = frames_sorted[-1] - frames_sorted[0]
        print(f"Frames: {frames_sorted[0]}..{frames_sorted[-1]} span={span} n={len(frames_sorted)}")
    print(f"Labels: {label_count}")
    if actor_ids:
        print(f"Actor IDs in labels: {sorted(actor_ids)[:8]}{' ...' if len(actor_ids) > 8 else ''}")

    missing_rate = (missing_images / total_expected) if total_expected > 0 else 0.0
    print(f"Expected images: {total_expected}, missing: {missing_images}, missing_rate={missing_rate:.4%}")

    if missing_images > 0:
        return 2
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Validate one dataset run directory (metadata/labels/images alignment).")
    ap.add_argument("--run-dir", required=True, help="Run directory path, e.g. dataset/proxy_123/run_YYYYmmdd_HHMMSS")
    args = ap.parse_args()

    run_dir = Path(args.run_dir)
    try:
        code = _validate_run_dir(run_dir)
        if code == 0:
            print("OK")
        else:
            print("FAILED")
        return int(code)
    except Exception as e:
        print(f"ERROR: {e}")
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
