# track_plot（CARLA 轨迹复现与隧道数据集采集系统）

本仓库围绕 CARLA 仿真引擎，提供两大核心能力：

- **轨迹复现**：外部车辆轨迹数据解析、质量校验、地图对齐可视化、CARLA 世界中自动回放
- **隧道仿真与数据集采集**：三车道代理车流生成、多相机图像采集、2D 检测框/实例分割标注、批量自动化导出

## 项目目录结构

```text
track_plot/
├─ auto_control_main.py           # 轨迹回放入口
├─ run_all_batches.bat            # 一键批量采集脚本
├─ tp_replay/                     # 轨迹复现核心包
├─ tp_tunnel_traffic/             # 隧道仿真 + 数据集采集
│  ├─ main.py                     # 隧道场景主入口
│  ├─ collector.py                # 帧级数据集采集器（RGB/COCO/instance）
│  ├─ video_collector.py          # 视频录制器（连续帧 + 压力参数）
│  ├─ hololens_server.py           # HoloLens 2 WebRTC 推流
│  ├─ config.py                   # 全部环境变量配置
│  ├─ control.py                  # 自动驾驶控制器
│  ├─ lane_sampling.py            # 三车道提取
│  ├─ gui.py                      # Pygame GUI 控制台
│  ├─ spawning.py                 # 车辆生成与净空判定
│  ├─ vehicle_plan.py             # 代理车计划
│  ├─ autopilot.py                # Traffic Manager 集成
│  ├─ merge_coco.py               # 全局 COCO 合并工具
│  ├─ validate_dataset_run.py     # run 级校验工具
│  ├─ dataset_cameras.json        # 相机布局配置
│  ├─ camera_offsets.json         # 视角微调配置
│  ├─ docs/                       # 设计文档
│  ├─ tests/                      # 测试入口
│  └─ README.md                   # 模块详细文档
├─ tools/                         # 独立工具脚本
├─ dataset/                       # 数据集输出目录
│  ├── README.md                  # 数据集说明文档
│  ├── coco_annotations.json      # 全局 COCO 标注
│  ├── batch_summary.json         # 批量采集统计
│  └── proxy_<id>/run_*/          # run 级数据
├─ trace_data/                    # 轨迹数据
└─ run_logs/                      # 运行日志
```

## 一、轨迹复现（tp_replay）

### 快速开始
```powershell
conda activate carla
cd E:\code\track_plot
$env:TP_PRESERVE_EXISTING_WORLD='1'
python .\auto_control_main.py
```

### 关键配置
| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `TP_DATA_FILE_PATH` | — | 轨迹数据文件 |
| `TP_DATA_SCALE` | 0.01 | 厘米→米缩放 |
| `TP_REVERSE_DIRECTION` | True | 数据方向（隧道逆向） |
| `TP_SNAP_THRESHOLD` | 15.0 | 车道吸附距离（米） |

详细文档见 §4 轨迹复现完整流程。

## 二、隧道仿真与数据集采集（tp_tunnel_traffic）

### 快速开始：GUI 手动采集
```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla
cd /d E:\code\track_plot

set TT_GUI_ENABLE=1
set TT_PROXY_ENABLE=1
set TT_COLLECT_OUTPUT_DIR=dataset

python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

### 快速开始：批量自动化采集
```bat
E:\code\track_plot\run_all_batches.bat
```

### 核心能力
- 三车道代理车流（左快中稳右慢，持续补车）
- 7 相机 RGB 图像 + 7 相机实例分割 PNG
- COCO 2D 车辆检测框自动标注
- 控制量标签（steer/throttle/brake + ego state）
- 全局 COCO 合并（`merge_coco.py`）

### 校验命令
```bat
python -m tp_tunnel_traffic.validate_dataset_run --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset
```

详细文档见 `tp_tunnel_traffic/README.md` 和 `dataset/README.md`。

---

## 2. 数据格式（轨迹复现）

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

> 如遇问题，先检查 `config.py` 中的默认值与环境变量是否与你的 CARLA 地图/数据匹配。

---

## 参考命令汇总

**轨迹复现**：
```powershell
conda activate carla
cd E:\code\track_plot
$env:TP_PRESERVE_EXISTING_WORLD='1'
python .\auto_control_main.py
```

**隧道仿真 + GUI 采集**：
```bat
python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

**视频录制（GUI Record Video）**：
```bat
set TT_VIDEO_ENABLE=1
set TT_VIDEO_OUTPUT_DIR=dataset_video
python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

**HoloLens 推流**：
```bat
set TT_HOLOLENS_ENABLE=1
python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

**批量自动化采集**：
```bat
E:\code\track_plot\run_all_batches.bat
```

**数据集校验**：
```bat
python -m tp_tunnel_traffic.validate_dataset_run --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset
```
