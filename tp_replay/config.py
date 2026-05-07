"""回放配置（默认值 + env 覆盖后会被 main() 写入）。

注意：这里使用模块级变量以尽量保持原脚本行为（可被环境变量覆盖）。
"""

import math


# ==============================================================================
# 1. 核心配置区域
# ==============================================================================

# OpenDRIVE 文件路径：隧道地图
XODR_PATH = r"E:\carla\Unreal\CarlaUE4\Content\Carla\OpenDrive\QingShiLing.xodr"

# NOTE: 这里的默认数据文件会在 main() 中被环境变量 TP_DATA_FILE_PATH 覆盖。
# 默认使用 data_6lu1.txt
DATA_FILE_PATH = r"E:\code\track_plot\trace_data\track24.7.25-24.8.5_79_6lu\K615+703TV023_2024-07-25-14-40-19.txt"

# 调试/测试开关
# - TRACK_LIMIT: 限制回放车辆数（None 表示不限制）
# - PRINT_EVERY_N_TICKS: 降低控制台刷屏
TRACK_LIMIT = None
PRINT_EVERY_N_TICKS = 10
STATUS_LOG_EVERY_N_TICKS = 200  # 写入文件日志的状态频率（避免每次都刷文件）
LOG_LEVEL = "INFO"  # "DEBUG" / "INFO" / "WARNING"

# 日志文件输出
LOG_DIR = "run_logs"
LOG_FILE_PREFIX = "auto_control_main"

# 锚点
TUNNEL_ENTRY_X = -213549.140625 / 100.0
TUNNEL_ENTRY_Y = 55603.199219 / 100.0
TUNNEL_ENTRY_Z = 100.000000 / 100.0

# 变换参数
YAW_CORRECTION = 180.0
LATERAL_OFFSET = 0.0
REVERSE_DIRECTION = True
DATA_SCALE = 0.01
SCALE_Y = 1.0
SNAP_THRESHOLD = 15.0

# 对齐/吸附鲁棒性参数（面向三车道隧道）
ROTATION_BIAS_DEG = 0.0
DATA_ANGLE_BASELINE_M = 30.0
SNAP_STRICT_DIST = 4.0
LANE_CHANGE_PENALTY_M = 2.0
LANE_SWITCH_MIN_IMPROVEMENT_M = 2.0
ENFORCE_SAME_ROAD_ID = True

# 【速度与回放设置】
PLAYBACK_SPEED = 1.0  # 建议实时
ENABLE_PHYSICS = False

# 速度单位配置：如果原始数据是 km/h，则设为 True；如果是 m/s，则设为 False
DATA_SPEED_IN_KMH = True

# 速度转换系数：按上面配置自动决定（在 main() 里也会根据 env 重新计算）
SPEED_FACTOR = (1.0 / 3.6) if DATA_SPEED_IN_KMH else 1.0

# 物理参数
PHYSICS_SYNC_DIST = 5.0
CATCH_UP_GAIN = 0.0
LOOKAHEAD_TIME = 0.8
LOOKAHEAD_SPEED_GAIN = 0.03
ANGULAR_SMOOTH = 0.1

# 回放模式：physics_follow / kinematic
REPLAY_CONTROL_MODE = "kinematic"

# Teleport 迟滞
TELEPORT_CONSECUTIVE_TICKS = 3
TELEPORT_COOLDOWN_TICKS = 10
TELEPORT_YAW_GATE_DEG = 12.0

# 速度/角速度指令平滑与限幅（仅对 physics_follow 生效）
CMD_LPF_ALPHA_V = 0.3
CMD_LPF_ALPHA_W = 0.3
CMD_LPF_ALPHA_SPEED = 0.35
MAX_ACCEL_MPS2 = 2.5
MAX_DECEL_MPS2 = 4.0
CATCH_UP_ERROR_ALPHA = 0.2
CATCH_UP_DEADBAND_M = 0.5
MAX_YAW_RATE_RAD_S = math.radians(75.0)
MAX_YAW_ACCEL_RAD_S2 = math.radians(180.0)

# 销毁延迟 (秒)
DESTROY_DELAY = 2.0

# 轨迹终点处理策略
FINISH_BEHAVIOR = "teleport_away"
FINISH_TELEPORT_Z = -200.0

# 回放结束后保留世界运行一段时间（便于观察最终状态；0 表示不保留）
POST_PLAYBACK_HOLD_SECONDS = 0.0

# 生成失败重试
SPAWN_RETRY_DELAY_SECONDS = 0.3
SPAWN_MAX_ATTEMPTS = 60

# 生成候选点策略
SPAWN_CANDIDATE_OFFSETS_M = [0.0, 2.0, 4.0, 6.0]
SPAWN_TRY_ADJACENT_LANES = False
SPAWN_OVERLAP_BLOCK_DIST_M = 12.0
SPAWN_RETRY_MIN_REMAINING_SECONDS = 0.5
SPAWN_Z_OFFSET_M = 0.2
SPAWN_MAX_CANDIDATES = 12
SPAWN_RETRY_MIN_DELAY_SECONDS = 0.02

# 回放负载保护
MAX_ACTIVE_VEHICLES = 18
SPAWN_DEFER_SECONDS = 0.8

# 便于小规模测试（环境变量覆盖）：达到该轨迹时间后强制停止（默认 0=不限制）
MAX_TRAJ_TIME_SECONDS = 0.0
