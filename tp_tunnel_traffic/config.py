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
    camera_mode: str = _env_str("TT_CAMERA_MODE", "overview")

    # --- 单车中车道(-2)跟随控制调参 ---
    # 目标点前视距离（米），用于避免“追最近点”造成来回修正
    lookahead_m: float = _env_float("TT_LOOKAHEAD_M", 18.0)
    # 转向低通滤波系数：steer = (1-a)*prev + a*raw（越大越灵敏，过大会抖动）
    steer_lpf_alpha: float = _env_float("TT_STEER_LPF_ALPHA", 0.35)
    # 转向变化率限制（每秒最大变化量，单位 steer/s），用于抑制振荡
    steer_max_rate: float = _env_float("TT_STEER_MAX_RATE", 1.6)
    # 起步速度爬升时间（秒），降低起步打转/甩尾
    speed_ramp_seconds: float = _env_float("TT_SPEED_RAMP_SECONDS", 2.5)

    # 车内视角微调文件（运行时可编辑；main 会每帧读取）
    camera_tune_json: str = _env_str("TT_CAMERA_TUNE_JSON", r"tp_tunnel_traffic\camera_offsets.json")

    # --- 数据集采集（Phase A）---
    gui_enable: bool = _env_bool("TT_GUI_ENABLE", True)
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

    # --- 视频录制（驾驶人实时仿真）---
    video_enable: bool = _env_bool("TT_VIDEO_ENABLE", True)
    video_output_dir: str = _env_str("TT_VIDEO_OUTPUT_DIR", r"dataset_video")
    video_frame_stride: int = _env_int("TT_VIDEO_FRAME_STRIDE", 1)
    video_cameras_json: str = _env_str("TT_VIDEO_CAMERAS_JSON", r"tp_tunnel_traffic\dataset_cameras.json")

    # --- HoloLens 2 WebRTC 推流 ---
    hololens_enable: bool = _env_bool("TT_HOLOLENS_ENABLE", True)
    hololens_port: int = _env_int("TT_HOLOLENS_PORT", 8765)
    hololens_res_w: int = _env_int("TT_HOLOLENS_RES_W", 1280)
    hololens_res_h: int = _env_int("TT_HOLOLENS_RES_H", 720)
    hololens_fps: int = _env_int("TT_HOLOLENS_FPS", 30)

    # --- IDM（智能驾驶员模型）跟车参数 ---
    idm_max_accel: float = _env_float("TT_IDM_MAX_ACCEL", 2.0)          # 最大加速度 (m/s²)
    idm_comfort_decel: float = _env_float("TT_IDM_COMFORT_DECEL", 1.5)  # 舒适减速度 (m/s²)
    idm_min_gap: float = _env_float("TT_IDM_MIN_GAP", 2.0)              # 最小停车距离 (m)
    idm_time_headway: float = _env_float("TT_IDM_TIME_HEADWAY", 1.5)    # 期望时距 (s)
    idm_delta: float = _env_float("TT_IDM_DELTA", 4.0)                  # 加速度指数

    # --- 隧道代理车流（真实隧道参数）---
    # 中国高速公路隧道：限速 60~80 km/h，车距 2~3 秒，高峰流量 1500~2000 辆/车道/小时
    proxy_enable: bool = _env_bool("TT_PROXY_ENABLE", True)
    proxy_base_speed_mps: float = _env_float("TT_PROXY_BASE_SPEED_MPS", 18.0)       # ~65 km/h
    proxy_min_per_lane: int = _env_int("TT_PROXY_MIN_PER_LANE", 7)                  # 高密度
    proxy_max_per_lane: int = _env_int("TT_PROXY_MAX_PER_LANE", 14)                 # 高峰流量
    proxy_speed_diff_percent: float = _env_float("TT_PROXY_SPEED_DIFF_PERCENT", 8.0)
    proxy_follow_distance_m: float = _env_float("TT_PROXY_FOLLOW_DISTANCE_M", 16.0)
    proxy_spawn_clearance_m: float = _env_float("TT_PROXY_SPAWN_CLEARANCE_M", 13.0)
    proxy_spawn_lane_aware: bool = _env_bool("TT_PROXY_SPAWN_LANE_AWARE", True)
    proxy_spawn_same_lane_lateral_m: float = _env_float("TT_PROXY_SPAWN_SAME_LANE_LATERAL_M", 1.6)
    proxy_spawn_other_lane_clearance_m: float = _env_float("TT_PROXY_SPAWN_OTHER_LANE_CLEARANCE_M", 2.2)
    proxy_spawn_other_lane_lateral_m: float = _env_float("TT_PROXY_SPAWN_OTHER_LANE_LATERAL_M", 7.0)
    proxy_warmup_seconds: float = _env_float("TT_PROXY_WARMUP_SECONDS", 15.0)       # 高密度需更长预热
    proxy_spawn_start_ratio: float = _env_float("TT_PROXY_SPAWN_START_RATIO", 0.12)
    proxy_use_tm: bool = _env_bool("TT_PROXY_USE_TM", False)
    proxy_target_per_lane: int = _env_int("TT_PROXY_TARGET_PER_LANE", 0)
    proxy_detail_log_interval_s: float = _env_float("TT_PROXY_DETAIL_LOG_INTERVAL_S", 0.0)
    # 车道速度分层（百分比；负值=更快，正值=更慢）
    # 左车道：快 3%~8%（稳定超车道）
    proxy_speed_diff_left_min: float = _env_float("TT_PROXY_SPEED_DIFF_LEFT_MIN", -8.0)
    proxy_speed_diff_left_max: float = _env_float("TT_PROXY_SPEED_DIFF_LEFT_MAX", -3.0)
    # 中车道：基准速度 ±2%
    proxy_speed_diff_mid_min: float = _env_float("TT_PROXY_SPEED_DIFF_MID_MIN", -2.0)
    proxy_speed_diff_mid_max: float = _env_float("TT_PROXY_SPEED_DIFF_MID_MAX", 2.0)
    # 右车道：慢 3%~8%（稳定行车道）
    proxy_speed_diff_right_min: float = _env_float("TT_PROXY_SPEED_DIFF_RIGHT_MIN", 3.0)
    proxy_speed_diff_right_max: float = _env_float("TT_PROXY_SPEED_DIFF_RIGHT_MAX", 8.0)

    # 兼容 vehicle_plan / Traffic Manager 约定
    min_vehicles_per_lane: int = _env_int("TT_MIN_VEHICLES_PER_LANE", 1)
    max_vehicles_per_lane: int = _env_int("TT_MAX_VEHICLES_PER_LANE", 2)
    tm_port: int = _env_int("TT_TM_PORT", 8000)
    tm_ignore_lights: bool = _env_bool("TT_TM_IGNORE_LIGHTS", True)
    tm_ignore_signs: bool = _env_bool("TT_TM_IGNORE_SIGNS", True)
    tm_auto_lane_change: bool = _env_bool("TT_TM_AUTO_LANE_CHANGE", False)
    tm_follow_distance: float = _env_float("TT_TM_FOLLOW_DISTANCE", 8.0)
    tm_speed_diff_percent: float = _env_float("TT_TM_SPEED_DIFF_PERCENT", 10.0)
