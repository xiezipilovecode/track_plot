# tp_replay：CARLA 轨迹复现（回放）引擎

这是 `track_plot` 中专门负责**将外部车辆轨迹数据在 CARLA 仿真世界中回放**的核心模块。

## 核心能力

- 解析自定义轨迹数据文件（每行一条轨迹，空格分隔，6 字段一组节点）
- 自动对齐数据坐标系与 CARLA 地图坐标系（角度旋转 + 位置平移 + 缩放）
- 按轨迹时间轴自动调度车辆生成（spawn）和销毁（destroy）
- 支持物理跟随（physics_follow）和运动学直控（kinematic）两种回放模式
- 丰富的轨迹筛选机制（按时长/帧数/密度窗口/位移等条件过滤）
- 完整的回放统计与日志输出

## 快速开始

### 前提

- CARLA Server 已启动（默认 `localhost:2000`）
- Python 环境中已安装 `carla` 模块 + `numpy`
- 准备好轨迹数据文件（如 `data_6lu1.txt`）和 OpenDRIVE 地图（`QingShiLing.xodr`）

### 一键运行

```powershell
conda activate carla
cd E:\code\track_plot

# 推荐入口（仓库根目录）
set TP_PRESERVE_EXISTING_WORLD=1
python .\auto_control_main.py

# 或通过模块
set TP_PRESERVE_EXISTING_WORLD=1
python -m tp_replay.main
```

运行后会看到：
```
锚点锁定. 地图方向: -89.50
解析数据...
开始回放 (mode=kinematic)...
TrajTime: 12.50s | Active: 5 | Teleports: 42 | SpawnOK: 8 | ...
```

## 模块结构

```text
tp_replay/
├── __init__.py         # 包入口，延迟导入，避免无 CARLA 环境时 import 报错
├── main.py             # 主入口：env 覆盖、CARLA 连接、回放主循环、清理
├── engine.py           # 核心引擎：轨迹解析、坐标对齐、车辆调度、逐帧控制
├── models.py           # 数据模型：TrackFrame（帧）、VehicleTrack（轨迹）
├── config.py           # 默认配置（模块级变量，运行时被 env 覆盖）
├── selection.py        # 轨迹筛选：按时长/帧数/密度/位移等条件过滤
├── geometry.py         # 几何工具：角度计算、方向估计、点数提取
├── env_utils.py        # 环境变量读取工具函数
├── world_utils.py      # CARLA 世界工具：OpenDRIVE 生成/保留
└── carla_compat.py     # CARLA 兼容层：安全的 carla 导入
```

## 各模块详解

### `main.py` — 主入口（570 行）

**职责**：薄入口层，串联 env → config → CARLA 连接 → engine 初始化 → 主循环 → 清理。

核心流程：
1. `_apply_env_overrides_to_config()` — 用环境变量覆盖所有 config 默认值
2. `main()` — 连接 CARLA、设置同步模式、创建 `ReplayEngine`、进入 `while world.tick()` 主循环
3. 主循环中每 tick 调用 `engine.tick(fixed_dt)` 推进回放
4. 周期性打印控制台进度（`TrajTime / Active / SpawnOK / SpawnFail / ...`）
5. 支持 `TP_MAX_TRAJ_TIME` 提前终止、`POST_PLAYBACK_HOLD_SECONDS` 结束后保持
6. 异常安全：`KeyboardInterrupt` 友好退出，`finally` 中销毁所有车辆并恢复 world settings

**关键子功能**：
- `_try_set_spectator_view()` — 从 JSON 文件加载观众视角，或自动计算隧道入口最佳观察角度
- `_maybe_apply_weather_override()` — 支持 `TP_WEATHER_PRESET` / `TP_FORCE_WEATHER` 覆盖天气

### `engine.py` — 核心引擎（1393 行）

**职责**：轨迹回放的核心逻辑，是本模块最复杂的文件。

**主要方法**：

| 方法 | 说明 |
|------|------|
| `__init__()` | 加载 XODR 地图、确定入口锚点和地图方向 |
| `process_data(file_path)` | 解析轨迹文件、计算坐标对齐变换、生成 VehicleTrack 列表 |
| `tick(fixed_dt)` | 每帧更新：spawn 新车辆 → 推进 active tracks → 销毁完成的车辆 |
| `_spawn_vehicle(track)` | 为一条轨迹在 CARLA 中生成代理车辆（含碰撞检测和重试） |
| `_update_track(track, traj_time)` | 更新单条轨迹：计算目标位置、应用控制（teleport 或 physics_control） |
| `_find_best_waypoint(loc)` | 在地图上找到最近的可行车道点（lane snapping） |

**坐标对齐算法**（`process_data` 中）：
1. 扫描前 N 行数据，选择同时包含多节点且地图角度偏差最小的行作为锚点
2. 计算 `map_angle - data_angle` 作为旋转角度（`YAW_CORRECTION`）
3. 将锚点数据坐标旋转平移对齐到 `TUNNEL_ENTRY` 锚点
4. 后续所有轨迹点按相同变换处理（缩放 × DATA_SCALE、旋转、平移）

**回放控制模式**（详见下文"控制模式"章节）。

### `models.py` — 数据模型（103 行）

**`TrackFrame`**：单帧数据点
- `ts` — 相对时间戳（秒，已经过 rebase）
- `loc` — CARLA `Location`（对齐后的世界坐标）
- `rot` — CARLA `Rotation`（对齐后的朝向）
- `v` — 速度（m/s）

**`VehicleTrack`**：一条完整轨迹
- `frames` — TrackFrame 列表（按时间排序）
- `actor` — 对应的 CARLA `Vehicle` actor（spawn 后赋值）
- `spawned / finished / spawn_attempts` — 生命周期状态
- `next_spawn_time / start_time / end_time` — 时间调度
- `time_offset / loop_count` — 支持轨迹复用（extend replay）
- `teleport_over_ticks / teleport_cooldown_ticks` — 瞬移迟滞控制
- `cmd_vx / cmd_vy / cmd_speed / cmd_wz` — 控制指令缓存

关键方法：
- `add_frame(frame)` — 追加帧（自动去重，按 ts 排序）
- `get_state_at_time(rel_time)` — 在相邻帧间线性插值获取任意时刻状态

### `config.py` — 默认配置（117 行）

模块级变量，运行时被 `main.py` 中的 `_apply_env_overrides_to_config()` 通过环境变量覆盖。

**核心配置项**：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `XODR_PATH` | `QingShiLing.xodr` | OpenDRIVE 地图路径 |
| `DATA_FILE_PATH` | TV023 样本文件 | 轨迹数据文件 |
| `TRACK_LIMIT` | `None` | 限制回放车辆数 |
| `DATA_SCALE` | `0.01` | 数据坐标→CARLA 米缩放因子 |
| `REVERSE_DIRECTION` | `True` | 隧道逆向行驶 |
| `SNAP_THRESHOLD` | `15.0` | 车道吸附距离（米） |
| `PLAYBACK_SPEED` | `1.0` | 回放速度倍率 |
| `ENABLE_PHYSICS` | `False` | 是否启用物理模拟 |
| `REPLAY_CONTROL_MODE` | `kinematic` | 回放控制模式 |
| `FINISH_BEHAVIOR` | `teleport_away` | 轨迹结束处理 |
| `MAX_ACTIVE_VEHICLES` | `18` | 最大同时活跃车辆数 |

### `selection.py` — 轨迹筛选（443 行）

提供多种轨迹选择策略，通过 `TP_SELECT_MODE` 控制：

| 模式 | 说明 | 适用场景 |
|------|------|----------|
| `none`（默认） | 不过滤，使用全部轨迹 | 完整回放 |
| `min_duration` | 按最小时长过滤 | 排除短暂闪烁的检测 |
| `min_frames` | 按最少帧数过滤 | 排除低质量轨迹 |
| `duration_range` | 按时长区间过滤 | 选取特定行程长度的轨迹 |
| `top_n_duration` | 取时长最长的 N 条 | 展示代表性长轨迹 |
| `dense_window` | 在指定时间窗口内选最密集的 N 条 | 高密度场景测试 |
| `cover_window` | 选 N 条轨迹覆盖整个时间窗口 | 均匀分布测试 |

辅助功能：
- `_rebase_tracks_to_zero()` — 将所有轨迹起始时间归零（测试便利）
- `_compress_tracks_spawn_schedule()` — 将分散的 spawn 时间压缩到指定窗口（支持 uniform/clamp 模式 + jitter）

### `geometry.py` — 几何工具（56 行）

| 函数 | 说明 |
|------|------|
| `get_vector_angle_degrees(dx, dy)` | 计算向量角度 |
| `is_in_front(point, entry, forward)` | 判断点是否在入口前方 |
| `_extract_xy_points_from_tokens(tokens)` | 从 6 元组 token 序列提取有效 (x,y) 点 |
| `_calculate_stable_data_angle(points)` | 用长基线（~30m）估计方向，降低短基线噪声 |

### `world_utils.py` — 世界工具（94 行）

- `_maybe_generate_opendrive_world()` — 按需从 XODR 文件生成 CARLA 世界
  - 支持 `TP_PRESERVE_EXISTING_WORLD=1` 保留当前世界（推荐）
  - 支持 `TP_USE_GENERATE_OPENDRIVE_WORLD=1` 强制重新生成
  - 可配置 vertex_distance、max_road_length、wall_height 等生成参数

### `env_utils.py` — 环境变量工具（136 行）

类型安全的 env 读取函数：`_get_int_from_env` / `_get_float_from_env` / `_get_bool_from_env` / `_get_str_from_env` / `_get_float_list_from_env` / `_get_speed_factor_from_env`。

附加：`_ensure_logging_to_file()` — 创建带时间戳的日志文件输出。

### `carla_compat.py` — CARLA 兼容层（27 行）

- 在模块顶层尝试 `import carla`，失败时设 `_carla = None`
- `require_carla()` — 获取 carla 模块或抛出友好的错误提示
- 这使得 `compileall` 等工具在无 CARLA 环境下仍可正常运行

---

## 控制模式

### `kinematic` 模式（默认）

**直接 teleport**：每帧将车辆瞬间移动到目标位置和朝向。

- ✅ 轨迹精确复现，无物理偏差
- ✅ 适合轨迹数据质量验证
- ❌ 无车辆交互、无碰撞响应

**teleport 迟滞机制**：
- `TELEPORT_CONSECUTIVE_TICKS=3` — 连续 N tick 偏差过大才触发 teleport
- `TELEPORT_COOLDOWN_TICKS=10` — teleport 后冷却，避免抖动
- `TELEPORT_YAW_GATE_DEG=12` — 朝向偏差阈值

### `physics_follow` 模式

**物理控制**：通过 CARLA 的 `AckermannControl` 或 `VehicleControl` 驱动车辆跟随目标轨迹。

- ✅ 车辆有物理交互、可碰撞
- ✅ 轨迹更自然
- ❌ 存在跟随偏差（catch-up error）
- ❌ 需要调参（PID/LPF 参数）

通过 `TP_REPLAY_CONTROL_MODE=physics_follow` 启用。

**物理模式特有参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CMD_LPF_ALPHA_V` | 0.3 | 速度指令低通 |
| `CMD_LPF_ALPHA_W` | 0.3 | 角速度指令低通 |
| `MAX_ACCEL_MPS2` | 2.5 | 最大加速度 |
| `MAX_YAW_RATE_RAD_S` | 75°/s | 最大转向角速度 |
| `CATCH_UP_DEADBAND_M` | 0.5 | catch-up 死区 |

---

## 轨迹数据格式

```
# 每行一条轨迹，空格分隔，6 字段一组节点
时间戳 x像素 y世界坐标 速度km/h 加速度 车辆类型 [重复...]
```

**坐标变换链**（数据处理 → CARLA 世界）：
```
原始数据(cm) → ×DATA_SCALE(0.01) → 旋转(YAW_CORRECTION) → 平移(TUNNEL_ENTRY) → CARLA 世界坐标(m)
```

**速度变换链**：
```
原始速度(km/h) → ×SPEED_FACTOR(1/3.6) → 平移+旋转不变 → CARLA 速度(m/s)
```

---

## 完整环境变量参考

### 核心配置

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `TP_CARLA_HOST` | `localhost` | CARLA 服务器地址 |
| `TP_CARLA_PORT` | `2000` | CARLA 服务器端口 |
| `TP_CARLA_TIMEOUT_SECONDS` | `10.0` | 连接超时 |
| `TP_XODR_PATH` | `QingShiLing.xodr` | OpenDRIVE 地图路径 |
| `TP_DATA_FILE_PATH` | — | 轨迹数据文件（必须指定） |
| `TP_PRESERVE_EXISTING_WORLD` | `1` | 保留当前 CARLA 世界 |
| `TP_FIXED_DELTA_SECONDS` | `0.05` | 同步模式固定步长 |

### 回放控制

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `TP_REPLAY_CONTROL_MODE` | `kinematic` | `kinematic` / `physics_follow` |
| `TP_PLAYBACK_SPEED` | `1.0` | 回放速度倍率 |
| `TP_ENABLE_PHYSICS` | `0` | 启用物理模拟 |
| `TP_DATA_SPEED_IN_KMH` | `1` | 原始速度单位是否为 km/h |
| `TP_DATA_SPEED_FACTOR` | — | 显式速度缩放因子（覆盖自动计算） |
| `TP_MAX_TRAJ_TIME` | `0`（不限） | 最大回放时间（秒） |

### 对齐与吸附

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `TP_AUTO_ANCHOR` | `1` | 自动选择最优锚点 |
| `TP_ROTATION_BIAS_DEG` | `0` | 旋转偏置（度） |
| `TP_SNAP_THRESHOLD` | `15.0` | 车道吸附距离（米） |
| `TP_SNAP_STRICT_DIST` | `4.0` | 严格吸附距离 |
| `TP_ENFORCE_SAME_ROAD_ID` | `1` | 强制吸附到同一 road_id |

### 车辆生成

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `TP_SPAWN_MAX_ATTEMPTS` | `60` | 单辆车最大生成尝试次数 |
| `TP_SPAWN_RETRY_DELAY_SECONDS` | `0.3` | 生成重试间隔 |
| `TP_SPAWN_OVERLAP_BLOCK_DIST_M` | `12.0` | 重叠检测距离 |
| `TP_SPAWN_TRY_ADJACENT_LANES` | `0` | 跨车道生成尝试 |
| `TP_MAX_ACTIVE_VEHICLES` | `18` | 最大同时活跃车辆数 |
| `TP_SPAWN_DEFER_SECONDS` | `0.8` | 生成延迟（避免瞬时大量 spawn） |

### 轨迹筛选

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `TP_SELECT_MODE` | `none` | 选择模式 |
| `TP_SELECT_MIN_DURATION` | `8.0` | 最小时长（秒） |
| `TP_SELECT_NUM_TRACKS` | `5` | top_n / dense 模式选取数量 |
| `TP_TRACK_LIMIT` | — | 限制总轨迹数 |
| `TP_DENSE_WINDOW_SECONDS` | `60.0` | dense_window 窗口大小 |

### 生成调度压缩

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `TP_SPAWN_SCHEDULE_COMPRESS` | `0` | 压缩 spawn 时间窗口 |
| `TP_SPAWN_SCHEDULE_MODE` | `uniform` | `uniform` / `clamp` |
| `TP_SPAWN_SCHEDULE_BASE_SECONDS` | `0` | 窗口起始 |
| `TP_SPAWN_SCHEDULE_SPREAD_SECONDS` | `10.0` | 窗口跨度 |
| `TP_SPAWN_SCHEDULE_JITTER_SECONDS` | `0` | 随机抖动 |

### 其他

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `TP_FINISH_BEHAVIOR` | `teleport_away` | 轨迹结束处理 |
| `TP_POST_PLAYBACK_HOLD_SECONDS` | `0` | 回放结束后保持秒数 |
| `TP_WEATHER_PRESET` | — | 天气预设名（如 `ClearNoon`） |
| `TP_FORCE_WEATHER` | `0` | 启用自定义天气 |
| `TP_SPECTATOR_VIEW_JSON` | `camera_view.json` | 观众视角配置文件 |
| `TP_ENABLE_SUBSTEPPING` | `0` | 启用物理子步 |

---

## 入口文件

| 文件 | 用途 |
|------|------|
| `auto_control_main.py` | **推荐入口**，位于仓库根目录，延迟导入 `tp_replay.main` |
| `replay_main.py` | 兼容性别名（已移除，使用 `auto_control_main.py`） |

---

## 日志

- 控制台：实时进度条（`PRINT_EVERY_N_TICKS=10` 刷新间隔）
- 文件：`run_logs/auto_control_main-{timestamp}-{pid}.log`（可通过 `TP_LOG_DIR` / `TP_LOG_FILE_PREFIX` 配置）

日志级别：`TP_LOG_LEVEL=INFO`（默认）/ `DEBUG` / `WARNING`

---

## 常见问题

| 问题 | 排查 |
|------|------|
| `ModuleNotFoundError: carla` | 确认已 `conda activate carla`，CARLA PythonAPI 在 PYTHONPATH |
| 地图加载失败 | 检查 `TP_XODR_PATH` 是否正确；尝试 `TP_PRESERVE_EXISTING_WORLD=1` |
| 无车辆生成 | 检查 `TP_DATA_FILE_PATH`；确认数据文件格式正确；查看 `SpawnFail` 统计 |
| 车辆位置偏移 | 调整 `TP_AUTO_ANCHOR=1`；检查 `YAW_CORRECTION` 和 `TUNNEL_ENTRY` 锚点 |
| 控制台乱码 | Windows GBK 问题，引擎已内置 UTF-8 reconfigure |

---

## 架构要点

1. **薄入口 + 厚引擎**：`auto_control_main.py` → `main.py`（env 配置）→ `engine.py`（核心逻辑）
2. **延迟导入**：`carla_compat.py` 确保无 CARLA 环境时仍可 `import tp_replay`
3. **模块级 config**：运行时被 env 覆盖，保持与历史单文件脚本的行为兼容
4. **坐标对齐自动锚点**：`TP_AUTO_ANCHOR=1` 自动扫描数据选择最优对齐参数
5. **Spawning 策略**：重叠检测 + 候选点偏移 + 跨车道尝试 + 重试冷却 = 高成功率生成

---

## 拼接轨迹回放系统

从 `trace_data/track_stitch/` 输出的拼接 JSON 数据，直接在 CARLA 隧道中回放。

> **坐标系统**：拼接 JSON 中的 y 坐标与原始数据一致（世界 cm），通过 `DATA_SCALE=0.01 → 旋转 → 平移` 变换到 CARLA 坐标。此变换链与 `engine.process_data()` 完全一致。

> **前提**：stitch 模式需要 `TP_DATA_FILE_PATH` 指向一个锚点文件，用于计算坐标变换参数。

### 架构

```
stitched_trajectories.json (70K 轨迹)
        │
        ▼ StitchAdapter.load_for_kinematic()
        │   ├── JSON → 坐标变换（复用 engine 变换参数）
        │   ├── 逐节点路点吸附
        │   └── 构建全帧 VehicleTrack
        │
        ▼ ReplayEngine.tick()（engine 原生 kinematic 驱动）
            ├── 帧间插值平滑行驶
            ├── yaw gating 防闪烁
            └── 自动 spawn/despawn
```

### 启动方式

**方式 1：stitch_kinematic 模式（推荐）**

```bat
set TP_REPLAY_MODE=stitch_kinematic
set TP_STITCH_JSON_PATH=E:\code\track_plot\trace_data\track_stitch\output\stitched_trajectories.json
set TP_STITCH_MIN_QUALITY=0.7
set TP_STITCH_MIN_CAMERAS=4
set TP_TRACK_LIMIT=20
set TP_STITCH_MAX_START_S=600
python auto_control_main.py
```

**方式 2：测试脚本（单条轨迹验证）**

```bat
set TP_STITCH_JSON_PATH=E:\code\track_plot\trace_data\track_stitch\output\stitched_trajectories.json
set TP_STITCH_NTH=1
python -m tp_replay.tests.test_stitch_autopilot
```

**方式 3：stitch_autopilot 模式（TM 自动驾驶）**

```bat
set TP_REPLAY_MODE=stitch_autopilot
set TP_STITCH_JSON_PATH=E:\code\track_plot\trace_data\track_stitch\output\stitched_trajectories.json
set TP_STITCH_TM_MAX_ACTIVE=20
python auto_control_main.py
```

### 核心模块

| 文件 | 功能 |
|------|------|
| `stitch_adapter.py` | JSON→VehicleTrack，含全帧模式和速度曲线模式 |
| `stitch_autopilot.py` | TM 自动驾驶编排器（spawn/speed/destroy） |
| `tests/test_stitch_autopilot.py` | 单条轨迹测试脚本 |
