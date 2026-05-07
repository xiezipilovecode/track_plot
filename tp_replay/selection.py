import logging
import math
import os
import random

from . import config
from .env_utils import _get_bool_from_env, _get_float_from_env, _get_int_from_env, _get_str_from_env


logger = logging.getLogger(__name__)


def _duration_s(track):
    try:
        return max(0.0, float(track.end_time) - float(track.start_time))
    except Exception:
        return 0.0


def _select_tracks_for_replay_window(tracks):
    """Select tracks to best cover a replay window."""

    mode = (os.getenv("TP_SELECT_MODE") or "none").strip().lower()
    if mode != "cover_window":
        return None

    win_s = _get_float_from_env(
        "TP_COVER_WINDOW_SECONDS",
        float(config.MAX_TRAJ_TIME_SECONDS) if float(config.MAX_TRAJ_TIME_SECONDS or 0.0) > 0.0 else 60.0,
    )
    min_dur = _get_float_from_env("TP_SELECT_MIN_DURATION", max(8.0, win_s * 0.6))
    n = _get_int_from_env("TP_SELECT_NUM_TRACKS", 20)

    min_frames_raw = os.getenv("TP_SELECT_MIN_FRAMES")
    min_frames = None
    if min_frames_raw is not None and min_frames_raw != "":
        min_frames = _get_int_from_env("TP_SELECT_MIN_FRAMES", 20)

    cand = [t for t in tracks if _duration_s(t) >= min_dur]
    if min_frames is not None:
        cand = [t for t in cand if len(getattr(t, "frames", [])) >= min_frames]

    if not cand:
        logger.warning(
            "Track selection cover_window kept 0 candidates (min_dur=%.3f min_frames=%s); falling back",
            min_dur,
            (str(min_frames) if min_frames is not None else "none"),
        )
        return []

    def score(t):
        st = float(getattr(t, "start_time", 0.0))
        dur = _duration_s(t)
        late_pen = max(0.0, st - (win_s * 0.2))
        return dur - 0.5 * late_pen

    chosen = sorted(cand, key=score, reverse=True)[: max(1, n)]
    chosen = sorted(chosen, key=lambda t: float(getattr(t, "start_time", 0.0)))
    if chosen:
        durs = [_duration_s(t) for t in chosen]
        logger.info(
            "Track selection cover_window: window_s=%.1f min_dur=%.1f min_frames=%s cand=%d kept=%d dur_s[min=%.3f max=%.3f]",
            win_s,
            min_dur,
            (str(min_frames) if min_frames is not None else "none"),
            len(cand),
            len(chosen),
            min(durs),
            max(durs),
        )
    return chosen


def _select_tracks(tracks):
    """Select a small subset for testing while preserving spawn order."""

    mode = (os.getenv("TP_SELECT_MODE") or "none").strip().lower()
    if mode in ("", "none"):
        return tracks

    cover = _select_tracks_for_replay_window(tracks)
    if cover is not None:
        if cover:
            return cover
        return tracks

    _debug_err_counts = {"disp": 0, "path": 0}

    def _track_displacement_m(t):
        if not t.frames:
            return 0.0
        a = t.frames[0].loc
        b = t.frames[-1].loc
        try:
            return a.distance(b)
        except Exception as e:
            if _debug_err_counts["disp"] < 3:
                logger.debug("track displacement distance() failed: %s", e)
                _debug_err_counts["disp"] += 1
            return 0.0

    def _track_path_length_m(t):
        if len(t.frames) < 2:
            return 0.0
        total = 0.0
        prev = t.frames[0].loc
        for f in t.frames[1:]:
            try:
                total += prev.distance(f.loc)
            except Exception as e:
                if _debug_err_counts["path"] < 3:
                    logger.debug("track path length distance() failed: %s", e)
                    _debug_err_counts["path"] += 1
            prev = f.loc
        return total

    def _track_speed_stats(t):
        if not t.frames:
            return 0.0, 1.0
        speeds = []
        zeros = 0
        for f in t.frames:
            try:
                v = float(f.v)
            except Exception:
                v = 0.0
            speeds.append(v)
            if abs(v) <= 1e-6:
                zeros += 1
        mean_v = sum(speeds) / max(1, len(speeds))
        zero_frac = zeros / max(1, len(speeds))
        return mean_v, zero_frac

    def _track_entry_dist_m(t):
        if not t.frames:
            return float("inf")
        try:
            a = t.frames[0].loc
            dx = float(a.x) - float(config.TUNNEL_ENTRY_X)
            dy = float(a.y) - float(config.TUNNEL_ENTRY_Y)
            return float(math.hypot(dx, dy))
        except Exception:
            return float("inf")

    min_disp = _get_float_from_env("TP_FILTER_MIN_DISPLACEMENT_M", 0.0)
    min_path = _get_float_from_env("TP_FILTER_MIN_PATH_LENGTH_M", 0.0)
    min_mean_speed = _get_float_from_env("TP_FILTER_MIN_MEAN_SPEED_MPS", 0.0)
    max_zero_frac = _get_float_from_env("TP_FILTER_MAX_SPEED_ZERO_FRACTION", 1.0)
    max_entry_dist = _get_float_from_env("TP_FILTER_MAX_ENTRY_DIST_M", 0.0)
    if (
        min_disp > 0.0
        or min_path > 0.0
        or min_mean_speed > 0.0
        or max_zero_frac < 1.0
        or max_entry_dist > 0.0
    ):
        before = len(tracks)
        kept2 = []
        dropped = 0
        for t in tracks:
            disp = _track_displacement_m(t)
            path = _track_path_length_m(t)
            mean_v, zf = _track_speed_stats(t)
            if disp < min_disp:
                dropped += 1
                continue
            if path < min_path:
                dropped += 1
                continue
            if mean_v < min_mean_speed:
                dropped += 1
                continue
            if zf > max_zero_frac:
                dropped += 1
                continue
            if max_entry_dist > 0.0:
                if _track_entry_dist_m(t) > max_entry_dist:
                    dropped += 1
                    continue
            kept2.append(t)
        if kept2:
            tracks = kept2
            logger.info(
                "Track prefilter by motion/speed/entry: kept=%d/%d dropped=%d min_disp=%.3f min_path=%.3f min_mean_v=%.3f max_zero_frac=%.3f max_entry_dist=%.3f",
                len(tracks),
                before,
                dropped,
                min_disp,
                min_path,
                min_mean_speed,
                max_zero_frac,
                max_entry_dist,
            )
        else:
            logger.warning(
                "Track prefilter by motion/speed dropped all tracks (before=%d); ignoring filters",
                before,
            )

    min_start = _get_float_from_env("TP_SELECT_MIN_START", float("-inf"))
    max_start = _get_float_from_env("TP_SELECT_MAX_START", float("inf"))
    if min_start != float("-inf") or max_start != float("inf"):
        before = len(tracks)
        start_windowed = [t for t in tracks if min_start <= float(t.start_time) <= max_start]
        if start_windowed:
            tracks = start_windowed
            logger.info(
                "Track prefilter by start_time: kept=%d/%d window=[%.3f, %.3f]",
                len(tracks),
                before,
                min_start,
                max_start,
            )
        else:
            logger.warning(
                "Track prefilter by start_time kept 0 tracks for window=[%.3f, %.3f]; ignoring window",
                min_start,
                max_start,
            )

    def duration_s(t):
        return max(0.0, float(t.end_time) - float(t.start_time))

    kept = tracks
    if mode == "min_duration":
        min_dur = _get_float_from_env("TP_SELECT_MIN_DURATION", 8.0)
        kept = [t for t in tracks if duration_s(t) >= min_dur]
        min_frames_raw = os.getenv("TP_SELECT_MIN_FRAMES")
        if min_frames_raw is not None and min_frames_raw != "":
            min_frames = _get_int_from_env("TP_SELECT_MIN_FRAMES", 12)
            kept = [t for t in kept if len(t.frames) >= min_frames]
    elif mode == "min_frames":
        min_frames = _get_int_from_env("TP_SELECT_MIN_FRAMES", 12)
        kept = [t for t in tracks if len(t.frames) >= min_frames]
    elif mode == "duration_range":
        lo = _get_float_from_env("TP_SELECT_MIN_DURATION", 3.0)
        hi = _get_float_from_env("TP_SELECT_MAX_DURATION", 20.0)
        kept = [t for t in tracks if lo <= duration_s(t) <= hi]
    elif mode == "top_n_duration":
        n = _get_int_from_env("TP_SELECT_NUM_TRACKS", 5)
        top = sorted(tracks, key=duration_s, reverse=True)[: max(0, n)]
        kept = sorted(top, key=lambda t: t.start_time)
    elif mode == "dense_window":
        win_s = _get_float_from_env("TP_DENSE_WINDOW_SECONDS", 60.0)
        n = _get_int_from_env("TP_SELECT_NUM_TRACKS", 10)
        min_dur = _get_float_from_env("TP_SELECT_MIN_DURATION", 8.0)

        min_frames_raw = os.getenv("TP_SELECT_MIN_FRAMES")
        min_frames = None
        if min_frames_raw is not None and min_frames_raw != "":
            min_frames = _get_int_from_env("TP_SELECT_MIN_FRAMES", 12)

        cand = [t for t in tracks if duration_s(t) >= min_dur]
        if min_frames is not None:
            cand = [t for t in cand if len(t.frames) >= min_frames]

        cand.sort(key=lambda t: t.start_time)
        if not cand or win_s <= 0 or n <= 0:
            kept = cand
        else:
            durs = [duration_s(t) for t in cand]
            dur_prefix = [0.0]
            for d in durs:
                dur_prefix.append(dur_prefix[-1] + float(d))

            best_i = 0
            best_j = 0
            best_cnt = -1
            best_sum_dur = -1.0
            j = 0
            for i in range(len(cand)):
                if j < i:
                    j = i
                end_t = cand[i].start_time + win_s
                while j < len(cand) and cand[j].start_time < end_t:
                    j += 1
                cnt = j - i
                sum_dur = dur_prefix[j] - dur_prefix[i]
                if cnt > best_cnt or (cnt == best_cnt and sum_dur > best_sum_dur):
                    best_cnt = cnt
                    best_sum_dur = sum_dur
                    best_i = i
                    best_j = j

            windowed = cand[best_i:best_j]
            chosen = sorted(windowed, key=duration_s, reverse=True)[:n]
            kept = sorted(chosen, key=lambda t: t.start_time)

            if kept:
                logger.info(
                    "Track selection dense_window: window_s=%.1f min_dur=%.1f min_frames=%s window=[%.3f, %.3f) cand=%d kept=%d",
                    win_s,
                    min_dur,
                    (str(min_frames) if min_frames is not None else "none"),
                    cand[best_i].start_time,
                    cand[best_i].start_time + win_s,
                    len(cand),
                    len(kept),
                )
    else:
        logger.warning("Unknown TP_SELECT_MODE=%r; selection disabled", mode)
        kept = tracks

    if not kept:
        logger.warning("Track selection kept 0 tracks (mode=%s). Falling back to original list.", mode)
        return tracks

    durs = [duration_s(t) for t in kept]
    logger.info(
        "Track selection: mode=%s kept=%d/%d dur_s[min=%.3f max=%.3f]",
        mode,
        len(kept),
        len(tracks),
        min(durs),
        max(durs),
    )

    preview_n = _get_int_from_env("TP_SELECT_PREVIEW_N", 10)
    for t in kept[: max(0, preview_n)]:
        try:
            logger.info(
                "Selected track: id=%s frames=%d start=%.3fs end=%.3fs dur=%.3fs",
                getattr(t, "id", "?"),
                len(getattr(t, "frames", [])),
                float(t.start_time),
                float(t.end_time),
                duration_s(t),
            )
        except Exception:
            continue
    return kept


def _rebase_tracks_to_zero(tracks):
    """Shift all selected tracks so the earliest start_time becomes 0.0s (test convenience)."""

    if not tracks:
        return tracks

    base = min((getattr(t, "global_start_time", lambda: t.start_time)() for t in tracks), default=0.0)
    if base <= 0:
        return tracks

    for t in tracks:
        for f in t.frames:
            f.ts -= base
        t.start_time -= base
        t.end_time -= base
        t.time_offset = 0.0

    logger.info(
        "Rebased selected tracks by %.3fs (earliest start_time -> 0.0s), tracks=%d",
        base,
        len(tracks),
    )
    return tracks


def _compress_tracks_spawn_schedule(tracks):
    """Optionally compress track global start times into an early window."""

    if not tracks:
        return tracks

    enabled = _get_bool_from_env("TP_SPAWN_SCHEDULE_COMPRESS", False)
    if not enabled:
        return tracks

    mode = _get_str_from_env("TP_SPAWN_SCHEDULE_MODE", "uniform").strip().lower()
    base_s = _get_float_from_env("TP_SPAWN_SCHEDULE_BASE_SECONDS", 0.0)
    spread_s = _get_float_from_env("TP_SPAWN_SCHEDULE_SPREAD_SECONDS", 10.0)
    jitter_s = _get_float_from_env("TP_SPAWN_SCHEDULE_JITTER_SECONDS", 0.0)
    seed_raw = os.getenv("TP_SPAWN_SCHEDULE_SEED")

    if spread_s <= 0.0:
        logger.warning(
            "Spawn schedule compression enabled but spread<=0 (base=%.3f spread=%.3f); ignored",
            base_s,
            spread_s,
        )
        return tracks

    rng = random.Random(0)
    if seed_raw is not None and str(seed_raw).strip() != "":
        try:
            rng = random.Random(int(seed_raw))
        except ValueError:
            rng = random.Random(str(seed_raw))

    ordered = sorted(tracks, key=lambda t: float(getattr(t, "start_time", 0.0)))

    before_starts = []
    for t in ordered:
        try:
            before_starts.append(float(t.global_start_time()))
        except Exception:
            before_starts.append(float(getattr(t, "start_time", 0.0)))

    n = len(ordered)
    if mode in ("uniform", "even"):
        denom = max(1, n - 1)
        targets = [base_s + spread_s * (i / denom) for i in range(n)]
    elif mode in ("clamp", "cap"):
        targets = []
        for t in ordered:
            st = float(getattr(t, "start_time", 0.0))
            targets.append(min(base_s + spread_s, max(base_s, st)))
    else:
        logger.warning("Unknown TP_SPAWN_SCHEDULE_MODE=%r; using uniform", mode)
        denom = max(1, n - 1)
        targets = [base_s + spread_s * (i / denom) for i in range(n)]

    if jitter_s > 0.0:
        lo = base_s
        hi = base_s + spread_s
        jittered = []
        for x in targets:
            j = rng.uniform(-abs(jitter_s), abs(jitter_s))
            jittered.append(min(hi, max(lo, x + j)))
        targets = jittered

    for t, new_global_start in zip(ordered, targets):
        try:
            t.time_offset = float(new_global_start) - float(t.start_time)
        except Exception:
            t.time_offset = 0.0

    after_starts = [float(t.global_start_time()) for t in ordered]
    logger.info(
        "Spawn schedule compressed: mode=%s tracks=%d window=[%.3f, %.3f] jitter=%.3f seed=%r global_start[min=%.3f max=%.3f] (was [%.3f, %.3f])",
        mode,
        n,
        base_s,
        base_s + spread_s,
        jitter_s,
        seed_raw,
        min(after_starts),
        max(after_starts),
        min(before_starts),
        max(before_starts),
    )

    return tracks
