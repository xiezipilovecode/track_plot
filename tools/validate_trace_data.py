import argparse
import csv
import logging
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime


logger = logging.getLogger(__name__)


def _get_int_from_env(name: str, default: int) -> int:
    v = os.getenv(name)
    if v is None or v == "":
        return default
    try:
        return int(v)
    except ValueError:
        logger.warning("Invalid env %s=%r; using default=%r", name, v, default)
        return default


def _get_float_from_env(name: str, default: float) -> float:
    v = os.getenv(name)
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        logger.warning("Invalid env %s=%r; using default=%r", name, v, default)
        return default


def _get_bool_from_env(name: str, default: bool) -> bool:
    v = os.getenv(name)
    if v is None or v == "":
        return default
    s = str(v).strip().lower()
    if s in ("1", "true", "yes", "y", "on"):
        return True
    if s in ("0", "false", "no", "n", "off"):
        return False
    logger.warning("Invalid env %s=%r; using default=%r", name, v, default)
    return default


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _setup_logging(output_dir: str, log_level: str) -> str:
    _ensure_dir(output_dir)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = os.path.join(output_dir, f"validate_summary-{ts}-{os.getpid()}.log")

    level = getattr(logging, str(log_level).upper(), logging.INFO)
    root = logging.getLogger()
    root.setLevel(level)

    if not root.handlers:
        logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(message)s")

    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setLevel(level)
    fh.setFormatter(fmt)
    root.addHandler(fh)
    return log_path


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return None
    mid = n // 2
    if n % 2 == 1:
        return xs[mid]
    return 0.5 * (xs[mid - 1] + xs[mid])


def _mean(xs):
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    if not xs:
        return float("nan")
    return sum(xs) / float(len(xs))


def _std(xs):
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    if len(xs) < 2:
        return float("nan")
    m = sum(xs) / float(len(xs))
    var = sum((x - m) ** 2 for x in xs) / float(len(xs) - 1)
    return math.sqrt(var)


@dataclass
class TrackMetrics:
    track_id: int
    n_frames: int
    duration_seconds: float
    start_ts_ms: float
    end_ts_ms: float
    mean_dt_ms: float
    std_dt_ms: float
    missing_timestamps: int
    negative_time_jumps: int
    mean_speed: float
    speed_zero_fraction: float
    displacement_m: float
    path_length_m: float
    median_step_m: float
    max_step_m: float
    stationary_flag: int
    outlier_position_count: int
    invalid_type_flag: int
    passes_basic_checks: int


def _parse_track_tokens(tokens, data_scale: float, keep_small_x: bool):
    """Parse one line tokens into time series.

    Expected format per 6-tuple:
        ts x y v accel_or_placeholder type

    Returns dict containing: ts_raw_ms(list[float]), x_m(list[float]), y_m(list[float]), v(list[float|None]), types(set[str])
    """
    ts_raw_ms = []
    xs_m = []
    ys_m = []
    vs = []
    types = set()
    last_valid_v = None

    for i in range(0, len(tokens), 6):
        if i + 2 >= len(tokens):
            break
        try:
            ts = float(tokens[i])
            x = float(tokens[i + 1])
            y = float(tokens[i + 2])
        except (ValueError, IndexError):
            continue

        if (not keep_small_x) and abs(x) < 1.0:
            # Match auto_control_main.py behavior: drop obviously invalid points
            continue

        v = None
        if i + 3 < len(tokens):
            raw_v = str(tokens[i + 3]).strip()
            if raw_v != "" and raw_v.lower() != "null":
                try:
                    v = float(raw_v)
                    last_valid_v = v
                except ValueError:
                    v = last_valid_v
            else:
                v = last_valid_v

        if i + 5 < len(tokens):
            t = str(tokens[i + 5]).strip()
            if t:
                types.add(t)

        ts_raw_ms.append(ts)
        xs_m.append(x * data_scale)
        ys_m.append(y * data_scale)
        vs.append(v)

    return {
        "ts_raw_ms": ts_raw_ms,
        "x_m": xs_m,
        "y_m": ys_m,
        "v": vs,
        "types": types,
    }


def compute_track_metrics(
    *,
    track_id: int,
    ts_raw_ms,
    x_m,
    y_m,
    v,
    types,
    min_frames: int,
    min_duration_seconds: float,
    allow_zero_speed: bool,
    position_outlier_dist_m: float,
    allowed_types,
):
    n = len(ts_raw_ms)
    if n == 0:
        return TrackMetrics(
            track_id=track_id,
            n_frames=0,
            duration_seconds=0.0,
            start_ts_ms=float("nan"),
            end_ts_ms=float("nan"),
            mean_dt_ms=float("nan"),
            std_dt_ms=float("nan"),
            missing_timestamps=0,
            negative_time_jumps=0,
            mean_speed=float("nan"),
            speed_zero_fraction=float("nan"),
            displacement_m=float("nan"),
            path_length_m=float("nan"),
            median_step_m=float("nan"),
            max_step_m=float("nan"),
            stationary_flag=0,
            outlier_position_count=0,
            invalid_type_flag=0,
            passes_basic_checks=0,
        )

    dts_ms = []
    negative_jumps = 0
    for i in range(1, n):
        dt = ts_raw_ms[i] - ts_raw_ms[i - 1]
        if dt < 0:
            negative_jumps += 1
        else:
            dts_ms.append(dt)

    med_dt = _median(dts_ms) if dts_ms else None
    missing = 0
    if med_dt is not None and med_dt > 0:
        for dt in dts_ms:
            if dt > 3.0 * med_dt:
                missing += 1

    duration_s = max(0.0, (ts_raw_ms[-1] - ts_raw_ms[0]) / 1000.0)

    # Speed metrics
    v_clean = [vv for vv in v if vv is not None]
    mean_speed = _mean(v_clean) if v_clean else float("nan")

    if v_clean:
        zero_cnt = sum(1 for vv in v_clean if abs(float(vv)) < 1e-6)
        speed_zero_fraction = zero_cnt / float(len(v_clean))
    else:
        speed_zero_fraction = float("nan")

    # Position jump outliers (simple and robust)
    outlier_pos = 0
    steps = []
    for i in range(1, len(x_m)):
        dx = x_m[i] - x_m[i - 1]
        dy = y_m[i] - y_m[i - 1]
        dist = math.hypot(dx, dy)
        steps.append(dist)
        if dist > float(position_outlier_dist_m):
            outlier_pos += 1

    displacement_m = math.hypot(x_m[-1] - x_m[0], y_m[-1] - y_m[0]) if n >= 2 else 0.0
    path_length_m = sum(steps) if steps else 0.0
    median_step_m = _median(steps) if steps else 0.0
    max_step_m = max(steps) if steps else 0.0

    # Stationary heuristic: either speed is all ~0 (when available) OR position barely changes.
    stationary_by_speed = bool(v_clean) and all(abs(float(vv)) < 1e-6 for vv in v_clean)
    stationary_by_pos = displacement_m < 0.5 and path_length_m < 1.0
    stationary_flag = 1 if (stationary_by_speed or stationary_by_pos) else 0

    invalid_type = 0
    if allowed_types:
        for t in types:
            if t not in allowed_types:
                invalid_type = 1
                break

    passes = 1
    if n < int(min_frames) or duration_s < float(min_duration_seconds):
        passes = 0
    if (not allow_zero_speed) and stationary_by_speed:
        passes = 0

    return TrackMetrics(
        track_id=track_id,
        n_frames=n,
        duration_seconds=duration_s,
        start_ts_ms=float(ts_raw_ms[0]),
        end_ts_ms=float(ts_raw_ms[-1]),
        mean_dt_ms=_mean(dts_ms) if dts_ms else float("nan"),
        std_dt_ms=_std(dts_ms) if dts_ms else float("nan"),
        missing_timestamps=missing,
        negative_time_jumps=negative_jumps,
        mean_speed=mean_speed,
        speed_zero_fraction=speed_zero_fraction,
        displacement_m=float(displacement_m),
        path_length_m=float(path_length_m),
        median_step_m=float(median_step_m),
        max_step_m=float(max_step_m),
        stationary_flag=int(stationary_flag),
        outlier_position_count=outlier_pos,
        invalid_type_flag=invalid_type,
        passes_basic_checks=passes,
    )


def _write_metrics_csv(path: str, rows):
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "track_id",
                "n_frames",
                "duration_seconds",
                "start_ts_ms",
                "end_ts_ms",
                "mean_dt_ms",
                "std_dt_ms",
                "missing_timestamps",
                "negative_time_jumps",
                "mean_speed",
                "speed_zero_fraction",
                "displacement_m",
                "path_length_m",
                "median_step_m",
                "max_step_m",
                "stationary_flag",
                "outlier_position_count",
                "invalid_type_flag",
                "passes_basic_checks",
            ]
        )
        for r in rows:
            w.writerow(
                [
                    r.track_id,
                    r.n_frames,
                    f"{r.duration_seconds:.6f}",
                    "" if math.isnan(r.start_ts_ms) else f"{r.start_ts_ms:.3f}",
                    "" if math.isnan(r.end_ts_ms) else f"{r.end_ts_ms:.3f}",
                    "" if math.isnan(r.mean_dt_ms) else f"{r.mean_dt_ms:.6f}",
                    "" if math.isnan(r.std_dt_ms) else f"{r.std_dt_ms:.6f}",
                    r.missing_timestamps,
                    r.negative_time_jumps,
                    "" if math.isnan(r.mean_speed) else f"{r.mean_speed:.6f}",
                    "" if math.isnan(r.speed_zero_fraction) else f"{r.speed_zero_fraction:.6f}",
                    "" if math.isnan(r.displacement_m) else f"{r.displacement_m:.6f}",
                    "" if math.isnan(r.path_length_m) else f"{r.path_length_m:.6f}",
                    "" if math.isnan(r.median_step_m) else f"{r.median_step_m:.6f}",
                    "" if math.isnan(r.max_step_m) else f"{r.max_step_m:.6f}",
                    r.stationary_flag,
                    r.outlier_position_count,
                    r.invalid_type_flag,
                    r.passes_basic_checks,
                ]
            )


def _percentile(xs, p: float):
    """Return p-th percentile (0-100) using linear interpolation."""
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    if not xs:
        return float("nan")
    xs.sort()
    if p <= 0:
        return float(xs[0])
    if p >= 100:
        return float(xs[-1])
    k = (len(xs) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return float(xs[int(k)])
    d0 = xs[int(f)] * (c - k)
    d1 = xs[int(c)] * (k - f)
    return float(d0 + d1)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Validate trajectory txt file (one track per line, 6-tuple repeated): "
            "ts x y v accel_or_placeholder type ..."
        )
    )
    parser.add_argument(
        "--data-file",
        required=True,
        help="Path to trajectory txt file (e.g. data_6lu2.txt)",
    )
    parser.add_argument(
        "--output-dir",
        default=os.getenv("VALIDATOR_OUTPUT_DIR") or os.path.join("run_logs", "validator"),
        help="Output directory for logs and metrics CSV (env: VALIDATOR_OUTPUT_DIR)",
    )
    parser.add_argument(
        "--min-frames",
        type=int,
        default=_get_int_from_env("VALIDATOR_MIN_FRAMES", 6),
        help="Minimum frames required per track (env: VALIDATOR_MIN_FRAMES)",
    )
    parser.add_argument(
        "--min-duration",
        type=float,
        default=_get_float_from_env("VALIDATOR_MIN_DURATION_SECONDS", 1.0),
        help="Minimum duration seconds required per track (env: VALIDATOR_MIN_DURATION_SECONDS)",
    )
    parser.add_argument(
        "--allow-zero-speed",
        action="store_true",
        default=_get_bool_from_env("VALIDATOR_ALLOW_ZERO_SPEED", False),
        help="Allow tracks with all zero speeds (env: VALIDATOR_ALLOW_ZERO_SPEED)",
    )
    parser.add_argument(
        "--data-scale",
        type=float,
        default=_get_float_from_env("VALIDATOR_DATA_SCALE", 0.01),
        help="Scale applied to x/y to convert to meters (default 0.01 like cm->m). env: VALIDATOR_DATA_SCALE",
    )
    parser.add_argument(
        "--position-outlier-dist-m",
        type=float,
        default=_get_float_from_env("VALIDATOR_POSITION_OUTLIER_DIST_M", 50.0),
        help="Step distance above which a point is counted as position outlier (env: VALIDATOR_POSITION_OUTLIER_DIST_M)",
    )
    parser.add_argument(
        "--keep-small-x",
        action="store_true",
        default=_get_bool_from_env("VALIDATOR_KEEP_SMALL_X", False),
        help="Keep points even if abs(x)<1.0 (auto_control_main.py drops them). env: VALIDATOR_KEEP_SMALL_X",
    )
    parser.add_argument(
        "--log-level",
        default=os.getenv("VALIDATOR_LOG_LEVEL") or "INFO",
        help="Logging level (env: VALIDATOR_LOG_LEVEL)",
    )
    args = parser.parse_args()

    try:
        log_path = _setup_logging(args.output_dir, args.log_level)
    except OSError as e:
        print(f"Fatal: failed to setup logging output_dir={args.output_dir!r}: {e}", file=sys.stderr)
        return 3

    logger.info("Validator log: %s", log_path)
    logger.info(
        "Run config: data_file=%s output_dir=%s min_frames=%s min_duration=%s allow_zero_speed=%s data_scale=%s "
        "position_outlier_dist_m=%s keep_small_x=%s",
        args.data_file,
        args.output_dir,
        args.min_frames,
        args.min_duration,
        args.allow_zero_speed,
        args.data_scale,
        args.position_outlier_dist_m,
        args.keep_small_x,
    )

    if not os.path.exists(args.data_file):
        logger.error("Fatal: data file not found: %s", args.data_file)
        return 4

    allowed_types = {"car", "truck", "bus"}

    rows = []
    fatal_parse_errors = 0
    with open(args.data_file, "r", encoding="utf-8") as f:
        for line_idx, line in enumerate(f):
            tokens = line.replace("\n", " ").split()
            if len(tokens) < 6:
                continue
            parsed = _parse_track_tokens(tokens, float(args.data_scale), bool(args.keep_small_x))
            try:
                m = compute_track_metrics(
                    track_id=line_idx,
                    ts_raw_ms=parsed["ts_raw_ms"],
                    x_m=parsed["x_m"],
                    y_m=parsed["y_m"],
                    v=parsed["v"],
                    types=parsed["types"],
                    min_frames=int(args.min_frames),
                    min_duration_seconds=float(args.min_duration),
                    allow_zero_speed=bool(args.allow_zero_speed),
                    position_outlier_dist_m=float(args.position_outlier_dist_m),
                    allowed_types=allowed_types,
                )
            except Exception as e:
                fatal_parse_errors += 1
                if fatal_parse_errors <= 5:
                    logger.exception("Track metrics compute failed line=%d: %s", line_idx, e)
                continue
            rows.append(m)

    _ensure_dir(args.output_dir)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = os.path.join(args.output_dir, f"validate_metrics-{ts}-{os.getpid()}.csv")
    try:
        _write_metrics_csv(csv_path, rows)
    except OSError as e:
        logger.error("Fatal: failed to write metrics CSV: %s", e)
        return 5

    total_tracks = len(rows)
    failed_basic = sum(1 for r in rows if r.passes_basic_checks == 0)
    tracks_with_time_anom = sum(1 for r in rows if r.negative_time_jumps > 0 or r.missing_timestamps > 0)
    tracks_with_pos_outliers = sum(1 for r in rows if r.outlier_position_count > 0)
    invalid_types = sum(1 for r in rows if r.invalid_type_flag)

    durations = [r.duration_seconds for r in rows]
    n_frames = [r.n_frames for r in rows]

    median_duration = _median(durations) or 0.0
    median_frames = _median(n_frames) or 0.0

    max_duration = max(durations) if durations else 0.0
    p95_duration = _percentile(durations, 95.0)
    p99_duration = _percentile(durations, 99.0)
    long_30s = sum(1 for d in durations if d >= 30.0)
    long_60s = sum(1 for d in durations if d >= 60.0)
    long_120s = sum(1 for d in durations if d >= 120.0)

    stationary_tracks = sum(1 for r in rows if r.stationary_flag)

    logger.info("Wrote metrics CSV: %s", csv_path)
    logger.info(
        "Summary: total_tracks=%d failed_basic_checks=%d median_duration_seconds=%.3f median_n_frames=%.1f "
        "max_duration_seconds=%.3f p95_duration_seconds=%.3f p99_duration_seconds=%.3f long30s=%d long60s=%d long120s=%d "
        "stationary_tracks=%d tracks_with_time_anomalies=%d tracks_with_position_outliers=%d invalid_types=%d fatal_parse_errors=%d",
        total_tracks,
        failed_basic,
        float(median_duration),
        float(median_frames),
        float(max_duration),
        float(p95_duration),
        float(p99_duration),
        long_30s,
        long_60s,
        long_120s,
        stationary_tracks,
        tracks_with_time_anom,
        tracks_with_pos_outliers,
        invalid_types,
        fatal_parse_errors,
    )

    print(
        "\n".join(
            [
                f"Validator log: {log_path}",
                f"Metrics CSV  : {csv_path}",
                "Summary:",
                f"  total_tracks               : {total_tracks}",
                f"  failed_basic_checks        : {failed_basic}",
                f"  median_duration_seconds    : {float(median_duration):.3f}",
                f"  max_duration_seconds       : {float(max_duration):.3f}",
                f"  p95_duration_seconds       : {float(p95_duration):.3f}",
                f"  p99_duration_seconds       : {float(p99_duration):.3f}",
                f"  long_tracks_ge_30s         : {long_30s}",
                f"  long_tracks_ge_60s         : {long_60s}",
                f"  long_tracks_ge_120s        : {long_120s}",
                f"  stationary_tracks          : {stationary_tracks}",
                f"  median_n_frames            : {float(median_frames):.1f}",
                f"  tracks_with_time_anomalies : {tracks_with_time_anom}",
                f"  tracks_with_position_outliers: {tracks_with_pos_outliers}",
                f"  invalid_types              : {invalid_types}",
                f"  fatal_parse_errors         : {fatal_parse_errors}",
            ]
        )
    )

    # Exit code policy:
    # 0 = clean (all tracks pass + no anomalies/outliers/invalid types)
    # 1 = suspicious (non-fatal issues exist)
    # >1 = fatal errors
    suspicious = (
        failed_basic > 0
        or tracks_with_time_anom > 0
        or tracks_with_pos_outliers > 0
        or invalid_types > 0
    )
    return 1 if suspicious else 0


if __name__ == "__main__":
    raise SystemExit(main())
