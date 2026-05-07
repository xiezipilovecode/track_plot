# track_plot（CARLA 轨迹复现与处理系统）

本仓库围绕 CARLA 仿真引擎的隧道场景，提供从**轨迹数据解析→质量校验→地图对齐可视化→自动回放**的完整闭环能力。

> **核心目标**：把外部采集的车辆轨迹数据（txt/csv/json）准确复现在 CARLA 世界中，保持车道吸附、对齐和逆向行驶等语义。

---

## 1. 项目架构概览

### 1.1 目录结构

```text
track_plot/
├─ auto_control_main.py        # 推荐回放入口（延迟导入，避免 import 时强依赖 CARLA）
├─ replay_main.py          # 兼容入口（保留旧脚本用法，可选使用）
├─ tp_replay/             # 回放核心包（模块化后的引擎与配置）
│  ├─ __init__.py
│  ├─ main.py             # 主入口：环境变量覆盖 → ReplayEngine 初始化 → 回放循环
│  ├─ engine.py           # ReplayEngine：轨迹解析、变换、对齐、回放控制
│  ├─ config.py          # 全局配置（默认锚点、缩放、吸附阈值等）
│  ├─ carla_compat.py     # CARLA 延迟导入兼容
│  ├─ models.py          # 数据模型：TrackFrame、VehicleTrack
│  ├─ geometry.py        # 几何计算：角度、旋转、XY 提取
│  ├─ selection.py      # 轨迹挑选与调度策略
│  ├─ world_utils.py    # 地图生成/天气覆盖等辅助
│  └─ env_utils.py     # 环境变量解析工具
├─ tools/                 # 独立工具脚本（推荐路径）
│  ├─ trace_display.py    # 地图对齐可视化（蓝点原始 / 绿点吸附）
│  ├─ trace_plot.py    # 轨迹绘图（matplotlib）
│  ├─ scan_trace_time_ranges.py # 时间范围扫描
│  ├─ get_position.py  # 实时坐标采集
│  ├─ get_start.py    # 地图起点获取
│  ├─ debug_line_width.py  # 线宽调试
│  ├─ validate_trace_data.py   # 轨迹质量校验
│  └─ record_spectator_view.py # 视角记录
├─ trace_data/            # 轨迹数据目录
│  ├─ test_data/        # 测试/示例轨迹（data_6lu*.txt）
│  └─ archive/         # 历史脚本归档
├─ run_logs/            # 运行日志（可再生）
├─ plot_data/          # 绘图导出（可再生）
└─ README.md
```

### 1.2 核心模块职责

| 模块 | 职责 | 关键 API |
|---|---|---|
| `tp_replay/main.py` | 程序入口、环境变量注入、日志初始化、世界设置 | `main()` |
| `tp_replay/engine.py` | 轨迹解析、自动锚点选择、坐标变换、车道吸附、回放循环 | `ReplayEngine.process_data()`, `tick()` |
| `tp_replay/config.py` | 全部默认值（锚点坐标、缩放、旋转、对齐阈值） | 环境变量覆盖（TP_*） |
| `tp_replay/carla_compat.py` | CARLA 延迟导入，conda 环境检测 | `require_carla()` |
| `tp_replay/geometry.py` | 角度计算、稳定基线选择、XY 提取 | `_calculate_stable_data_angle()` |
| `tp_replay/models.py` | 数据结构：每帧位置/速度/类型 | `TrackFrame`, `VehicleTrack` |
| `tools/trace_display.py` | 可视化检验：原始（蓝）vs 吸附（绿） | `main()` |
| `tools/validate_trace_data.py` | 轨迹质量统计（帧数、时长、跳跃、离群点） | CLI 参数 `--data-file` |
| `tools/record_spectator_view.py` | 视角保存与恢复 | 按 SPACE 保存 |

---

## 2. 数据格式

### 2.1 原始轨迹输入格式

每行一条轨迹（按 6 元组重复）：

```text
ts  x  y  v  accel_or_placeholder  type  ts  x  y  v ...
```

| 字段 | 含义 | 单位 | 说明 |
|---|---|---|---|
| `ts` | 时间戳 | ms | 整数 |
| `x, y` | 坐标 | **厘米**（默认） | 需 `DATA_SCALE=0.01` 转换为米 |
| `v` | 速度 | km/h 或 m/s | 由 `DATA_SPEED_IN_KMH` 决定 |
| `type` | 车型 | string | `car` / `truck` / `bus` |

示例：

```text
1719235200000  -213549.14  55603.19  45.6  null  car  1719235200050  -213540.00  55620.00  46.2  null  car
```

---

## 3. 关键配置项（环境变量）

> 所有配置均可在 `config.py` 中找到默认值，也可通过环境变量覆盖。

### 3.1 锚点与变换

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `TP_XODR_PATH` | `E:\carla\...\QingShiLing.xodr` | OpenDRIVE 地图文件 |
| `TP_DATA_FILE_PATH` | （见 config.py） | 轨迹数据文件路径 |
| `TP_TUNNEL_ENTRY_X / _Y / _Z` | -2135.49 / 556.03 / 1.0 | 隧道入口锚点（米） |
| `TP_DATA_SCALE` | 0.01 | 厘米→米缩放（必设） |
| `TP_REVERSE_DIRECTION` | True | 数据是否反向（隧道逆向行驶） |
| `TP_ROTATION_BIAS_DEG` | 0.0 | 手动角度微调（调试用） |
| `TP_DATA_ANGLE_BASELINE_M` | 30.0 | 计算数据朝向的基线长度 |

### 3.2 吸附与对齐

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `TP_SNAP_THRESHOLD` | 15.0 | 车道吸附距离（米） |
| `TP_SNAP_STRICT_DIST` | 4.0 | 严格模式吸附阈值 |
| `TP_ENFORCE_SAME_ROAD_ID` | True | 是否强制同道路 ID |
| `TP_LANE_CHANGE_PENALTY_M` | 2.0 | 换道惩罚距离 |

### 3.3 回放控制

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `TP_PLAYBACK_SPEED` | 1.0 | 回放倍率（1.0=实时） |
| `TP_ENABLE_PHYSICS` | False | 启用 CARLA 物理（默认运动学） |
| `TP_MAX_TRAJ_TIME` | 0.0（不限制） | 最大轨迹时长（秒） |
| `TP_TRACK_LIMIT` | None（不限制） | 最大并发车辆数 |
| `TT_GUI_ENABLE` | False | 启用 Pygame 可视化和操作界面 (需安装 pygame 和 numpy) |

### 3.4 日志与输出

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `TP_PRESERVE_EXISTING_WORLD` | （未设） | 保留现有 CARLA 世界，避免覆盖 |
| `TP_SPECTATOR_VIEW_JSON` | `camera_view.json` | 视角文件路径 |
| `TP_PRINT_EVERY_N_TICKS` | 10 | 控制台打印频率 |

---

## 4. 轨迹复现完整流程

### Step 0：环境准备

```powershell
# 1. 激活 CARLA 环境
conda activate carla

# 2. 进入项目目录
cd E:\code\track_plot
```

### Step 1：数据质量校验（推荐）

在正式回放前先用 `validate_trace_data.py` 检验轨迹质量：

```powershell
python .\tools\validate_trace_data.py --data-file .\trace_data\test_data\data_6lu2.txt --output-dir run_logs/validator
```

关键指标：
- `n_frames`：轨迹帧数（太少=无效）
- `duration_seconds`：轨迹时长
- `negative_time_jumps`：时间倒流（数据异常）
- `outlier_position_count`：位置跳变（离群点）
- `passes_basic_checks`：是否通过基础校验

### Step 2：地图对齐可视化（关键）

使用 `trace_display.py` 验证**锚点对齐**：

```powershell
python .\tools\trace_display.py
```

画面说明：
- **蓝色点**：原始轨迹坐标（变换后）
- **绿色点**：吸附到 CARLA 车道后的坐标
- **白色线**：`原始↔吸附` 偏差线

调试技巧：
- 如果蓝点整体偏向**左侧** → 调小 `MANUAL_ROTATION_FIX`（如 `-1.0`）
- 如果蓝点整体偏向**右侧** → 调大 `MANUAL_ROTATION_FIX`（如 `+1.0`）
- 每次调整 **0.5 度**，直到蓝点压在车道线上

### Step 3：正式回放

```powershell
# 方式一：推荐入口（延迟导入，避免 import 时强依赖 CARLA）
$env:TP_PRESERVE_EXISTING_WORLD='1'
python .\auto_control_main.py

# 方式二：直接调用模块（等效）
python -m tp_replay.main

# 方式三：兼容旧入口（保留）
python .\replay_main.py
```

回放控制：
- **空格**：暂停/继续
- **ESC**：停止并销毁所有车辆
- 回放完成后会自动保留世界（如需观察）

---

## 5. 常见问题与排查

### 5.1 车辆未出现在隧道入口

**可能原因**：
1. 锚点坐标 (`TUNNEL_ENTRY_X/Y/Z`) 与 XODR 地图不匹配
2. 数据未成功吸附（`SNAP_THRESHOLD` 太小）

**排查**：
```powershell
# 启用详细日志
$env:TP_LOG_LEVEL='DEBUG'
python .\auto_control_main.py 2>&1 | Select-String -Pattern "Auto anchor|snap|waypoint"
```

### 5.2 车辆逆向行驶/方向错误

**可能原因**：`REVERSE_DIRECTION` 参数未设

**修复**：
```powershell
# 隧道逆向设为 True
$env:TP_REVERSE_DIRECTION='True'
python .\auto_control_main.py
```

### 5.3 轨迹与车道偏差大

**可能原因**：
1. `DATA_SCALE` 错误（默认 0.01=厘米→米）
2. `MANUAL_ROTATION_FIX` 需要微调

**修复**：
1. 确认原始数据单位：如果是**厘米**，保持 `DATA_SCALE=0.01`；如果是**米**，设为 `1.0`
2. 打开 `trace_display.py` 观察蓝点偏移方向，手动微调 `MANUAL_ROTATION_FIX`

### 5.4 车道吸附失败/大量车辆未生成

**可能原因**：`SNAP_THRESHOLD` 或 `SNAP_STRICT_DIST` 太小，导致匹配失败

**修复**：
```powershell
$env:TP_SNAP_THRESHOLD='20.0'
$env:TP_SNAP_STRICT_DIST='6.0'
python .\auto_control_main.py
```

---

## 6. 进阶用法

### 6.1 限制并发车辆数

```powershell
$env:TP_TRACK_LIMIT='10'
python .\auto_control_main.py
```

### 6.2 仅回放前 N 秒

```powershell
$env:TP_MAX_TRAJ_TIME='30'
python .\auto_control_main.py
```

### 6.3 指定轨迹文件（覆盖默认）

```powershell
$env:TP_DATA_FILE_PATH='.\trace_data\test_data\data_6lu3.txt'
python .\auto_control_main.py
```

### 6.4 自定义天气

```powershell
# 使用预设
$env:TP_WEATHER_PRESET='ClearNoon'
python .\auto_control_main.py

# 自定义参数
$env:TP_FORCE_WEATHER='1'
$env:TP_WEATHER_CLOUDINESS='80'
$env:TP_WEATHER_RAIN='50'
python .\auto_control_main.py
```

### 6.5 视角保存与恢复

```powershell
# 1. 运行回放后，在 CARLA 中手动调整视角
# 2. 按 SPACE 保存当前视角到 camera_view.json

# 3. 下次运行时自动恢复
python .\auto_control_main.py
```

---

## 7. 隧道三车道与自动驾驶功能

### 7.1 功能目标

- 从 `QingShiLing.xodr` 提取完整三车道轨迹
- 在隧道场景中绘制左/中/右三条车道线
- 车辆在中间车道起点生成，并沿中间车道自动驾驶到终点
- 车辆视角保持第一人称驾驶人视角，并支持运行时热键旋转观察周围环境

### 7.2 主要实现思路

1. `tp_tunnel_traffic/lane_sampling.py`
   - 读取 OpenDRIVE 的 `planView` 和 `laneSection`
   - 根据车道宽度与 laneOffset 计算三条车道中心线
   - 将采样结果转换为 CARLA 坐标系中的 `PathPoint`

2. `tp_tunnel_traffic/control.py`
   - 使用路径前视点作为目标，避免只追最近点带来的抖动
   - 对转向进行低通和平滑限制，减少左右摆动
   - 根据目标速度控制油门，在隧道中保持稳定巡航

3. `tp_tunnel_traffic/main.py`
   - 生成车辆并设置起始朝向
   - 持续读取车道采样点，沿中间车道自动驾驶
   - 使用 spectator 跟随车辆，实现车内第一人称视角

### 7.3 测试入口

- `python -m tp_tunnel_traffic.tests.test_tunnel_lanes`
  - 只显示三车道车道线
  - 用于检查三车道是否完整、是否重合、是否从正确方向开始

- `python -m tp_tunnel_traffic.tests.test_tunnel_autodrive`
  - 运行隧道自动驾驶流程
  - 用于验证车辆是否能沿中间车道从起点行驶到终点

### 7.4 车内第一人称视角

- 默认视角由 `tp_tunnel_traffic/camera_offsets.json` 提供
- 运行时可通过热键旋转视角角度：
  - `[` / `]`：切换视角预设
  - `0`：恢复正前方
  - `,` / `.`：左右微调 1 度

---

## 8. 项目现状与后续建议

### 7.1 当前保留的功能模块

- `tp_replay/`：核心回放引擎
- `validate_trace_data.py` + `record_spectator_view.py`：校验与视角工具
- `tools/`：独立工具集
- `trace_data/test_data/`：示例轨迹数据

### 7.2 建议的后续改进

1. **配置外部化**：把硬编码路径 (`XODR_PATH`) 移到单独 `config.yaml` 或环境变量
2. **自动锚点选择**：目前依赖手动 `MANUAL_ROTATION_FIX`，可改进为自动最优角度搜索
3. **测试覆盖**：增加回归测试（检验吸附率、解析错误率）
4. **Docker 支持**：封装为容器，方便 CI/CD

---

## 9. 参考命令汇总

```powershell
# 激活 CARLA
conda activate carla
cd E:\code\track_plot

# 校验轨迹
python .\tools\validate_trace_data.py --data-file .\trace_data\test_data\data_6lu2.txt

# 可视化对齐
python .\tools\trace_display.py

# 回放
$env:TP_PRESERVE_EXISTING_WORLD='1'
python .\auto_control_main.py
```

> 如遇问题，先检查 `config.py` 中的默认值与环境变量是否与你的 CARLA 地图/数据匹配。
