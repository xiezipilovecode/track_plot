from __future__ import annotations

import os
from dataclasses import dataclass


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name)
    if v is None or v == "":
        return default
    return str(v).strip().lower() in {"1", "true", "yes", "y", "on"}


def _env_int(name: str, default: int) -> int:
    v = os.getenv(name)
    if v is None or v == "":
        return default
    try:
        return int(v)
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    v = os.getenv(name)
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def _env_str(name: str, default: str) -> str:
    v = os.getenv(name)
    return default if v is None or v == "" else v


@dataclass
class TunnelTrafficConfig:
    seed: int = _env_int("TT_SEED", 42)
    carla_host: str = _env_str("TP_CARLA_HOST", "localhost")
    carla_port: int = _env_int("TP_CARLA_PORT", 2000)
    carla_timeout: float = _env_float("TP_CARLA_TIMEOUT_SECONDS", 10.0)
    preserve_world: bool = _env_bool("TP_PRESERVE_EXISTING_WORLD", True)
    xodr_path: str = _env_str("TP_XODR_PATH", r"E:\carla\Unreal\CarlaUE4\Content\Carla\OpenDrive\QingShiLing.xodr")

    # 采样步长过大会导致曲率段转向抖动/起步偏航放大
    step_m: float = _env_float("TT_STEP_M", 2.0)
    spawn_clearance_m: float = _env_float("TT_SPAWN_CLEARANCE_M", 3.0)
    clear_existing_vehicles: bool = _env_bool("TT_CLEAR_EXISTING_VEHICLES", True)

    sync_mode: bool = _env_bool("TT_SYNC_MODE", True)
    fixed_delta_seconds: float = _env_float("TT_FIXED_DELTA_SECONDS", 0.05)
    run_seconds: float = _env_float("TT_RUN_SECONDS", 0.0)

    blueprint_filter: str = _env_str("TT_BLUEPRINT_FILTER", "vehicle.*")
    blueprints_deny: str = _env_str("TT_BLUEPRINT_DENY", "")

    debug_points: bool = _env_bool("TT_DEBUG_POINTS", False)
    debug_draw: bool = _env_bool("TT_DEBUG_DRAW", False)
    debug_limit: int = _env_int("TT_DEBUG_LIMIT", 25)
    camera_mode: str = _env_str("TT_CAMERA_MODE", "ego")

    # --- 单车中车道(-2)跟随控制调参 ---
    # 目标点前视距离（米），用于避免“追最近点”造成来回修正
    lookahead_m: float = _env_float("TT_LOOKAHEAD_M", 18.0)
    # 转向低通滤波系数：steer = (1-a)*prev + a*raw（越小越平滑，响应越慢）
    steer_lpf_alpha: float = _env_float("TT_STEER_LPF_ALPHA", 0.12)
    # 转向变化率限制（每秒最大变化量，单位 steer/s），用于抑制振荡
    steer_max_rate: float = _env_float("TT_STEER_MAX_RATE", 1.6)
    # 起步速度爬升时间（秒），降低起步打转/甩尾
    speed_ramp_seconds: float = _env_float("TT_SPEED_RAMP_SECONDS", 2.5)

    # 车内视角微调文件（运行时可编辑；main 会每帧读取）
    camera_tune_json: str = _env_str("TT_CAMERA_TUNE_JSON", r"tp_tunnel_traffic\camera_offsets.json")

    # --- 数据集采集（Phase A）---
    gui_enable: bool = _env_bool("TT_GUI_ENABLE", False)
    collect_enable: bool = _env_bool("TT_COLLECT_ENABLE", False)
    # 默认输出到项目根目录下的 dataset/
    collect_output_dir: str = _env_str("TT_COLLECT_OUTPUT_DIR", r"dataset")
    collect_output_by_target: bool = _env_bool("TT_COLLECT_OUTPUT_BY_TARGET", True)
    collect_cameras_json: str = _env_str("TT_COLLECT_CAMERAS_JSON", r"tp_tunnel_traffic\dataset_cameras.json")
    # 0.5s 采样一次（默认同步步长 0.05s，则约等于每 10 帧采样一次）
    collect_frame_stride: int = _env_int("TT_COLLECT_FRAME_STRIDE", 10)
    collect_record_labels: bool = _env_bool("TT_COLLECT_RECORD_LABELS", True)
    collect_show_overview: bool = _env_bool("TT_COLLECT_SHOW_OVERVIEW", True)
    collect_enable_instance_segmentation: bool = _env_bool("TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION", False)
    collect_instance_suffix: str = _env_str("TT_COLLECT_INSTANCE_SUFFIX", "_instance")
    collect_write_coco: bool = _env_bool("TT_COLLECT_WRITE_COCO", True)
    collect_coco_min_area_px2: float = _env_float("TT_COLLECT_COCO_MIN_AREA_PX2", 200.0)
    collect_coco_max_height_ratio: float = _env_float("TT_COLLECT_COCO_MAX_HEIGHT_RATIO", 0.9)

    # --- 自动化批量采集 ---
    auto_collect_enable: bool = _env_bool("TT_AUTO_COLLECT_ENABLE", False)
    auto_collect_seconds_per_run: float = _env_float("TT_AUTO_COLLECT_SECONDS_PER_RUN", 60.0)
    auto_collect_max_runs: int = _env_int("TT_AUTO_COLLECT_MAX_RUNS", 20)
    auto_collect_min_speed_mps: float = _env_float("TT_AUTO_COLLECT_MIN_SPEED_MPS", 5.0)
    auto_collect_mode: str = _env_str("TT_AUTO_COLLECT_MODE", "balanced")
    auto_collect_cooldown_s: float = _env_float("TT_AUTO_COLLECT_COOLDOWN_S", 5.0)
    auto_collect_target_images_per_camera: int = _env_int("TT_AUTO_COLLECT_TARGET_IMAGES_PER_CAMERA", 0)
    auto_collect_resume: bool = _env_bool("TT_AUTO_COLLECT_RESUME", True)
    auto_collect_output_batch_dir: str = _env_str("TT_AUTO_COLLECT_OUTPUT_BATCH_DIR", "dataset")

    # --- 阶段 B：代理车辆（受约束随机车流）---
    proxy_enable: bool = _env_bool("TT_PROXY_ENABLE", False)
    # 代理车基准速度（m/s），实际每辆车会叠加车道速度分层差异
    proxy_base_speed_mps: float = _env_float("TT_PROXY_BASE_SPEED_MPS", 12.0)
    proxy_min_per_lane: int = _env_int("TT_PROXY_MIN_PER_LANE", 4)
    proxy_max_per_lane: int = _env_int("TT_PROXY_MAX_PER_LANE", 8)
    # 隧道内巡航一般更稳定：默认让代理车比目标基准速度慢 5%~12% 左右
    proxy_speed_diff_percent: float = _env_float("TT_PROXY_SPEED_DIFF_PERCENT", 8.0)
    proxy_follow_distance_m: float = _env_float("TT_PROXY_FOLLOW_DISTANCE_M", 10.0)
    proxy_spawn_clearance_m: float = _env_float("TT_PROXY_SPAWN_CLEARANCE_M", 12.0)
    # 代理车生成净空判定是否按车道方向分解（推荐三车道密集生成时开启）
    # - 同车道：按纵向距离约束，避免追尾/扎堆
    # - 跨车道：仅使用更小的径向净空，降低相邻车道互相阻塞导致大量 spawn 失败
    proxy_spawn_lane_aware: bool = _env_bool("TT_PROXY_SPAWN_LANE_AWARE", True)
    # lane-aware 参数（单位：米）
    proxy_spawn_same_lane_lateral_m: float = _env_float("TT_PROXY_SPAWN_SAME_LANE_LATERAL_M", 1.6)
    proxy_spawn_other_lane_clearance_m: float = _env_float("TT_PROXY_SPAWN_OTHER_LANE_CLEARANCE_M", 2.5)
    proxy_spawn_other_lane_lateral_m: float = _env_float("TT_PROXY_SPAWN_OTHER_LANE_LATERAL_M", 6.0)
    proxy_warmup_seconds: float = _env_float("TT_PROXY_WARMUP_SECONDS", 8.0)
    proxy_spawn_start_ratio: float = _env_float("TT_PROXY_SPAWN_START_RATIO", 0.18)
    proxy_use_tm: bool = _env_bool("TT_PROXY_USE_TM", False)
    # 运行中每条车道的目标车辆数（持续补车维持该流量）；0 表示自动计算
    proxy_target_per_lane: int = _env_int("TT_PROXY_TARGET_PER_LANE", 0)
    # 全量代理车状态日志间隔（秒）
    proxy_detail_log_interval_s: float = _env_float("TT_PROXY_DETAIL_LOG_INTERVAL_S", 5.0)
    # 非 TM 模式下，按车道生成速度差分层（单位：百分比；正值=慢于基准，负值=快于基准）
    proxy_speed_diff_left_min: float = _env_float("TT_PROXY_SPEED_DIFF_LEFT_MIN", -8.0)
    proxy_speed_diff_left_max: float = _env_float("TT_PROXY_SPEED_DIFF_LEFT_MAX", -4.0)
    proxy_speed_diff_mid_min: float = _env_float("TT_PROXY_SPEED_DIFF_MID_MIN", -2.0)
    proxy_speed_diff_mid_max: float = _env_float("TT_PROXY_SPEED_DIFF_MID_MAX", 2.0)
    proxy_speed_diff_right_min: float = _env_float("TT_PROXY_SPEED_DIFF_RIGHT_MIN", 4.0)
    proxy_speed_diff_right_max: float = _env_float("TT_PROXY_SPEED_DIFF_RIGHT_MAX", 9.0)

    # 兼容 vehicle_plan / Traffic Manager 约定
    min_vehicles_per_lane: int = _env_int("TT_MIN_VEHICLES_PER_LANE", 1)
    max_vehicles_per_lane: int = _env_int("TT_MAX_VEHICLES_PER_LANE", 2)
    tm_port: int = _env_int("TT_TM_PORT", 8000)
    tm_ignore_lights: bool = _env_bool("TT_TM_IGNORE_LIGHTS", True)
    tm_ignore_signs: bool = _env_bool("TT_TM_IGNORE_SIGNS", True)
    tm_auto_lane_change: bool = _env_bool("TT_TM_AUTO_LANE_CHANGE", False)
    tm_follow_distance: float = _env_float("TT_TM_FOLLOW_DISTANCE", 8.0)
    tm_speed_diff_percent: float = _env_float("TT_TM_SPEED_DIFF_PERCENT", 10.0)
