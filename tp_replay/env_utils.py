import logging
import os
from datetime import datetime

from . import config


logger = logging.getLogger(__name__)


def _get_int_from_env(name, default):
    v = os.getenv(name)
    if v is None or v == "":
        return default
    try:
        return int(v)
    except ValueError:
        logger.warning("Invalid env %s=%r; using default=%s", name, v, default)
        return default


def _get_float_from_env(name, default):
    v = os.getenv(name)
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        logger.warning("Invalid env %s=%r; using default=%s", name, v, default)
        return default


def _get_bool_from_env(name, default=False):
    v = os.getenv(name)
    if v is None or v == "":
        return default
    s = str(v).strip().lower()
    if s in ("1", "true", "yes", "y", "on"):
        return True
    if s in ("0", "false", "no", "n", "off"):
        return False
    logger.warning("Invalid env %s=%r; using default=%s", name, v, default)
    return default


def _get_float_list_from_env(name, default_list):
    v = os.getenv(name)
    if v is None or v == "":
        return list(default_list)
    parts = [p.strip() for p in str(v).replace(";", ",").split(",")]
    out = []
    for p in parts:
        if p == "":
            continue
        try:
            out.append(float(p))
        except ValueError:
            logger.warning("Invalid float in env %s part=%r; ignored", name, p)
    if not out:
        logger.warning("Env %s parsed empty; using default=%s", name, default_list)
        return list(default_list)
    return out


def _get_str_from_env(name, default):
    v = os.getenv(name)
    if v is None:
        return default
    s = str(v).strip()
    return s if s != "" else default


def _get_speed_factor_from_env(default_factor: float) -> float:
    """Return speed conversion factor to m/s.

    Source of truth:
    - TP_DATA_SPEED_IN_KMH: bool; if true => divide by 3.6
    - TP_DATA_SPEED_FACTOR: float; optional explicit factor (overrides bool)
    """

    raw = os.getenv("TP_DATA_SPEED_FACTOR")
    if raw is not None and str(raw).strip() != "":
        try:
            return float(raw)
        except ValueError:
            logger.warning(
                "Invalid env TP_DATA_SPEED_FACTOR=%r; using default_factor=%s",
                raw,
                default_factor,
            )
            return float(default_factor)

    in_kmh = _get_bool_from_env("TP_DATA_SPEED_IN_KMH", config.DATA_SPEED_IN_KMH)
    return (1.0 / 3.6) if in_kmh else 1.0


def _ensure_logging_to_file():
    """Add a FileHandler once per run, while keeping console logs."""

    raw = os.getenv("TP_LOG_DIR")
    if raw is not None:
        s = str(raw).strip()
        if s == "" or s.lower() in (
            "0",
            "false",
            "off",
            "none",
            "null",
            "disable",
            "disabled",
        ):
            return ""
        log_dir = s
    else:
        log_dir = config.LOG_DIR

    os.makedirs(log_dir, exist_ok=True)

    file_prefix = os.getenv("TP_LOG_FILE_PREFIX") or config.LOG_FILE_PREFIX
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = os.path.join(log_dir, f"{file_prefix}-{ts}-{os.getpid()}.log")

    root = logging.getLogger()
    for h in root.handlers:
        if isinstance(h, logging.FileHandler):
            return getattr(h, "baseFilename", log_path)

    level_str = (os.getenv("TP_LOG_LEVEL") or config.LOG_LEVEL).upper()
    level = getattr(logging, level_str, logging.INFO)
    root.setLevel(level)
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setLevel(level)
    fh.setFormatter(fmt)
    root.addHandler(fh)
    return log_path
