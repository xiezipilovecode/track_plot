# tp_tunnel_traffic：隧道车流仿真 + GUI 选车采集

这是 `track_plot` 中专门针对 **CARLA 隧道 + OpenDRIVE(XODR)** 的子模块。

当前主能力：

- 从 `QingShiLing.xodr` 提取隧道三车道中心线，并可选 debug 绘制
- 在三车道上生成并维持**持续、稳定的代理车流**（左快中稳右慢，带安全跟车距离）
- 提供 GUI 控制台：
  - Overview 总览（WASD/QE 移动 + 鼠标右键拖拽旋转 + 滚轮缩放）
  - Ego View 第一人称（选中某辆代理车后跟随）
  - 选中代理车后按 `Collect Selected` 对该车进行数据采集（默认关闭，按需开启）

数据采集关键语义（非常重要）：

- **proxy-only**：不再生成旧的“主采集车(ego)”流程；采集对象就是你在 GUI 中选中的代理车
- **目录按需创建**：仅在 `Collect Selected -> ON` 时创建并写盘；仅选车/观察不会创建空目录
- **按 run 分段**：每次 ON 会在 `proxy_<id>/run_YYYYmmdd_HHMMSS/` 下创建独立 run 目录，避免多次 ON/OFF 混写
- **对齐主键统一为 world_frame**：标签与多相机图像使用同一个 `world.get_snapshot().frame` 对齐
- **持续采集策略**：采样点如果严格同帧凑不齐相机，会自动选“最近且完整”的帧写盘，避免采集中断

## 主要模块

### `main.py`
隧道场景的主入口：
- 连接 CARLA 世界
- 调用车道采样模块
- 绘制三车道线
- 生成车辆并执行自动驾驶
- 更新驾驶人第一人称视角

此外还包含：
- 代理车流的持续补车与全量状态日志输出
- GUI action 解析（选车、视角、Collect Selected）
- 将 `world_frame` 传给采集器，确保 label/image 对齐

### `lane_sampling.py`
从 XODR 中提取三车道轨迹：
- 解析 `planView` 中的几何线段
- 按 road / laneSection / sOffset 选择当前生效的 `laneOffset` 和车道宽度 `width`
- 先提取**中间车道（-2）**作为 source of truth
- 左/右车道由中间车道按常量车道宽度（默认 3.5m，可用环境变量 `TT_LANE_WIDTH_M` 覆盖）平移合成，使三车道始终平行且等间距
- 负责判断三车道是否有效，并跳过无效/重合段

### `collector.py`
数据集采集器（帧级）：
- 多相机 RGB + 实例分割传感器创建与管理
- labels.jsonl / COCO 2D 标注 / instance PNG 写盘
- 2D bbox 投影计算（3D→2D）

### `video_collector.py`
视频录制器（连续帧）：
- 7 相机连续帧录制 → `dataset_video/run_*/`
- FFmpeg 实时编码 MP4（背景线程异步）
- 预留驾驶人压力参数接口（driver_stress.jsonl）
- 通过 GUI `Record Video` 按钮控制启停

### `hololens_server.py`
HoloLens 2 WebRTC 推流：
- 单一世界相机（不绑定车辆），切车时无创建/销毁操作
- 将选中代理车驾驶员视角实时推流到 HoloLens 2（VideoStreamTrack → WebRTC）
- 接收 HoloLens 头部旋转（yaw/pitch）驱动 CARLA 相机（DataChannel）
- 独立后台线程运行 asyncio 事件循环，不影响仿真帧率
- 通过 GUI `HoloLens Stream` 按钮控制启停

### `config.py`
全部配置项（`TunnelTrafficConfig`），通过 `TT_*` / `TP_*` 环境变量覆盖。

### `gui.py`
Pygame GUI 控制台：
- Overview 自由视角 / Ego 第一人称
- 代理车列表与选中
- Collect Selected 采集开关

### `merge_coco.py`
全局 COCO 合并工具：`python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset`

### `validate_dataset_run.py`
run 级离线校验：检查 metadata/labels/images 对齐。


### `control.py`
自动驾驶控制器：
- 根据当前车辆位置和前视点计算方向
- 限制转向变化，减少左右抖动
- 根据目标速度控制油门

### `autopilot.py`
Traffic Manager 集成：自动导航配置与 TM 参数设置。

### `spawning.py`
车辆生成与净空判定：代理车在车道点上的约束生成，含 lane-aware 跨车道净空检测。

### `vehicle_plan.py`
代理车计划生成：按密度/速度/分布生成各车道的车辆蓝图与生成位置。

### 配置文件
- `dataset_cameras.json`：RGB 相机布局（位姿/分辨率/fov）
- `camera_offsets.json`：ego 视角微调参数

### `docs/`
- `DATASET_COLLECTION_DESIGN.md`：数据集采集模块设计文档
- `VIDEO_COLLECTION_DESIGN.md`：视频采集模块设计文档
- `HOLOLENS_INTEGRATION_DESIGN.md`：HoloLens 集成设计文档

### `tests/`
- `test_tunnel_lanes.py`：只显示三车道车道线
- `test_tunnel_autodrive.py`：GUI 手动采集入口
- `test_auto_collect.py`：无 GUI 批量自动化采集
- `test_dataset_capture.py`：数据集采集触发入口
- `test_dataset_vision_outputs.py`：COCO/instance 输出校验

当前“隧道代理车流 + GUI 选车采集”的推荐入口：
- `python -m tp_tunnel_traffic.tests.test_tunnel_autodrive`
 - （新增）检查 COCO/instance 输出：`python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS`
 - （新增）合并所有 run COCO：`python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset`

## 实现原理简述

1. **车道提取**
   - 先从 XODR 读取道路几何与车道信息
   - 再根据车道宽度和偏移计算出每条车道的中心轨迹

2. **车辆生成与起点方向**
   - 用中间车道的起点和下一个点计算起始朝向
   - 让车头对准中间车道的前进方向

3. **自动驾驶**
   - 采用前视点跟踪，不只是追最近点
   - 在同步 tick 中持续给车辆发送控制命令

4. **第一人称视角**
   - spectator 以车辆局部坐标系偏移的方式跟随车辆
   - 可通过热键在运行时调整 yaw 角度观察周围环境

运行时热键（控制台窗口焦点在该进程时生效）：
- `v`：切换 ego / overview
- `[` / `]`：切换 yaw 预设
- `0`：yaw 回正
- `,` / `.`：yaw 左右微调 1 度

## 运行（CMD）

### 0) 前提
- CARLA Server 已启动（默认 `localhost:2000`）
- 你用的是 Windows CMD（不是 PowerShell）

### 1) 安装 GUI 依赖（只需一次）
在 `carla` conda 环境中：

```bat
pip install pygame numpy
```

### 2) 一键启动：持续代理车流 + GUI + 选车采集

```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla

cd /d E:\code\track_plot

set TT_GUI_ENABLE=1
set TT_PROXY_ENABLE=1
set TP_PRESERVE_EXISTING_WORLD=1

rem 数据集输出在项目根 dataset/
set TT_COLLECT_OUTPUT_DIR=dataset
rem 采样间隔：0.05s*10≈0.5s
set TT_COLLECT_FRAME_STRIDE=10
set TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1
set TT_COLLECT_WRITE_COCO=1
set TT_COLLECT_INSTANCE_SUFFIX=_instance
set TT_COLLECT_COCO_MIN_AREA_PX2=200
set TT_COLLECT_COCO_MAX_HEIGHT_RATIO=0.9

rem 持续车流密度：每车道目标车辆数（0=自动）
set TT_PROXY_TARGET_PER_LANE=8
set TT_PROXY_DETAIL_LOG_INTERVAL_S=5

python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

新增采集输出：
- 实例分割 PNG：默认输出到 `images/<camera>_instance/<frame>.png`
- COCO 2D 标注：输出到 `labels_2d/coco_instances.json`（仅 vehicle 类）

HoloLens 推流：
- WebRTC VideoTrack 推流（VP8/H264），896×504 @ 30fps
- 头部姿态通过 WebRTC DataChannel 回传
- 单一世界相机架构，切车无崩溃
- GUI 按钮 "HoloLens Stream" 控制启停，跟随选中代理车视角
- 可通过 `set TT_HOLOLENS_ENABLE=1` 开启（默认端口 8765）

快速验证（COCO/instance 输出）：
```bat
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
```

如需导出抽检样本（含 bbox 列表）：
```bat
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS --sample-cameras ego,front --sample-limit 3
```

### 2.1) 一键启动（真实隧道车流 + 可直接采集，复制即用）

这套参数的目标是尽量贴近“隧道内稳定通勤车流”的感觉：
- 三车道**左快中稳右慢**，速度分布更像真实隧道（不追求极限速度）
- **中等偏高密度**但仍尽量稳定（不频繁碰撞/不疯狂变道）
- 跟车距离适中（不拥堵爬行，也不空旷）
- 采样频率默认 0.5s/帧（适合长时间跑）

```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla

cd /d E:\code\track_plot

rem --- 基础 ---
set TP_PRESERVE_EXISTING_WORLD=1
set TT_SYNC_MODE=1
set TT_FIXED_DELTA_SECONDS=0.05
set TT_SEED=42

rem --- GUI + 视角（推荐先用 overview 观察车流，再选车采集）---
set TT_GUI_ENABLE=1
set TT_CAMERA_MODE=overview

rem --- 数据集输出 ---
set TT_COLLECT_OUTPUT_DIR=dataset
rem 0.05s*10 ≈ 0.5s
set TT_COLLECT_FRAME_STRIDE=10
set TT_COLLECT_RECORD_LABELS=1
set TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1
set TT_COLLECT_WRITE_COCO=1
set TT_COLLECT_INSTANCE_SUFFIX=_instance
set TT_COLLECT_COCO_MIN_AREA_PX2=200
set TT_COLLECT_COCO_MAX_HEIGHT_RATIO=0.9

rem --- 代理车流（真实隧道车流：中等偏高密度 + 左快右慢）---
set TT_PROXY_ENABLE=1
set TT_PROXY_USE_TM=0

rem 持续流量：每车道目标车辆数
set TT_PROXY_TARGET_PER_LANE=9

rem 预热：先形成稳定流再开始你手动选车/采集
set TT_PROXY_WARMUP_SECONDS=10
set TT_PROXY_DETAIL_LOG_INTERVAL_S=6

rem 基准速度（m/s）：12.5≈45km/h（隧道较常见的稳定巡航）
set TT_PROXY_BASE_SPEED_MPS=12.5
rem 跟车距离（m）
set TT_PROXY_FOLLOW_DISTANCE_M=9.5

rem 车道速度分层（百分比；负值更快，正值更慢）
set TT_PROXY_SPEED_DIFF_LEFT_MIN=-10
set TT_PROXY_SPEED_DIFF_LEFT_MAX=-5
set TT_PROXY_SPEED_DIFF_MID_MIN=-2
set TT_PROXY_SPEED_DIFF_MID_MAX=2
set TT_PROXY_SPEED_DIFF_RIGHT_MIN=4
set TT_PROXY_SPEED_DIFF_RIGHT_MAX=10

rem 生成策略：减少相邻车道互相卡住导致的 spawn 失败
set TT_PROXY_SPAWN_LANE_AWARE=1
set TT_PROXY_SPAWN_CLEARANCE_M=13
set TT_PROXY_SPAWN_OTHER_LANE_CLEARANCE_M=2.5
set TT_PROXY_SPAWN_OTHER_LANE_LATERAL_M=6.5
set TT_PROXY_SPAWN_SAME_LANE_LATERAL_M=1.6

python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

COCO 2D 说明：
- `categories`: 仅包含 `vehicle`（id=1）
- `bbox`: [x, y, w, h] 像素坐标

采集方式（运行后）：
- 在 GUI 左侧 `Proxies` 列表里点选一辆代理车（会切到该车第一人称）
- 点击 `Collect Selected` -> ON 开始写盘（会创建 `dataset\proxy_<id>\run_...\`，并写 `effective_config.json`）
- 需要换车采集：先 OFF，再换车再 ON（切换目标时系统会自动 OFF，避免串数据）

## 多种车流模拟方式（让数据集更“多样”）

下面给出多套“车流配方”。你可以按 run 切换不同配方采集，得到更丰富的数据分布（密度/速度/跟车间距/规则等）。

使用方式：先按“2) 一键启动”那套基础命令运行，然后在启动前替换/追加对应 `set ...` 参数即可。

### 配方 1：稀疏车流（低遮挡，适合先打底）

```bat
set TT_PROXY_TARGET_PER_LANE=3
set TT_PROXY_FOLLOW_DISTANCE_M=12
set TT_PROXY_BASE_SPEED_MPS=11
set TT_PROXY_DETAIL_LOG_INTERVAL_S=5
```

### 配方 2：中等车流（默认推荐，兼顾稳定与多样性）

```bat
set TT_PROXY_TARGET_PER_LANE=8
set TT_PROXY_FOLLOW_DISTANCE_M=10
set TT_PROXY_BASE_SPEED_MPS=12
```

### 配方 3：高密度车流（高遮挡/高交互，压力测试写盘与齐帧）

```bat
set TT_PROXY_TARGET_PER_LANE=12
set TT_PROXY_FOLLOW_DISTANCE_M=7
set TT_PROXY_BASE_SPEED_MPS=10.5
rem 入口/生成失败多时可适当降低横向阻塞
set TT_PROXY_SPAWN_LANE_AWARE=1
set TT_PROXY_SPAWN_OTHER_LANE_CLEARANCE_M=2.2
set TT_PROXY_SPAWN_OTHER_LANE_LATERAL_M=7.0
```

### 配方 4：快车流（左快右慢更明显，提升速度分布多样性）

```bat
set TT_PROXY_TARGET_PER_LANE=8
set TT_PROXY_BASE_SPEED_MPS=14
set TT_PROXY_SPEED_DIFF_LEFT_MIN=-12
set TT_PROXY_SPEED_DIFF_LEFT_MAX=-6
set TT_PROXY_SPEED_DIFF_MID_MIN=-2
set TT_PROXY_SPEED_DIFF_MID_MAX=2
set TT_PROXY_SPEED_DIFF_RIGHT_MIN=6
set TT_PROXY_SPEED_DIFF_RIGHT_MAX=12
```

### 配方 5：慢车流（拥堵/爬行，更考验近距离跟车与遮挡）

```bat
set TT_PROXY_TARGET_PER_LANE=10
set TT_PROXY_BASE_SPEED_MPS=7.5
set TT_PROXY_FOLLOW_DISTANCE_M=6
set TT_PROXY_SPEED_DIFF_LEFT_MIN=-4
set TT_PROXY_SPEED_DIFF_LEFT_MAX=0
set TT_PROXY_SPEED_DIFF_RIGHT_MIN=0
set TT_PROXY_SPEED_DIFF_RIGHT_MAX=6
```

### 配方 6：Traffic Manager 模式（不同控制器分布，数据更“非手工规则”）

```bat
set TT_PROXY_USE_TM=1
set TT_TM_PORT=8000
set TT_TM_IGNORE_LIGHTS=1
set TT_TM_IGNORE_SIGNS=1
set TT_TM_AUTO_LANE_CHANGE=0
set TT_TM_FOLLOW_DISTANCE=8
set TT_TM_SPEED_DIFF_PERCENT=12
```

说明：TM 模式下代理车由 CARLA Traffic Manager 接管（而非本项目的 compute_control），适合做“不同控制策略”的数据多样性。

### 配方 7：随机性更强（多次采集换 seed，得到不同车辆分布/颜色/蓝图）

```bat
set TT_SEED=123
rem 每次 run 换一个 TT_SEED（例如 123/456/789）
```

### 配方 8：写盘压力更低（相机齐帧更容易，适合长时间稳定采集）

```bat
rem 降低采样频率（比如 1s 一次：0.05*20）
set TT_COLLECT_FRAME_STRIDE=20
set TT_PROXY_TARGET_PER_LANE=8
```

建议采集策略（最实用）：
- 按“配方 1/2/4/5/6”各采 5~10 分钟，形成多种分布；每次 ON 都会创建新的 run 目录，天然分段。
- 每次采集结束用离线校验工具检查：`python -m tp_tunnel_traffic.validate_dataset_run --run-dir ...`

可选：默认从 overview 开始（更方便观察全局车流）

```bat
set TT_CAMERA_MODE=overview
```

## 自动化批量采集

全自动无人值守采集（无需 GUI 手动点选）：

```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla
cd /d E:\code\track_plot

set TP_PRESERVE_EXISTING_WORLD=1
set TT_SYNC_MODE=1
set TT_FIXED_DELTA_SECONDS=0.05

set TT_PROXY_ENABLE=1
set TT_PROXY_TARGET_PER_LANE=9
set TT_PROXY_WARMUP_SECONDS=10

set TT_COLLECT_OUTPUT_DIR=dataset
set TT_COLLECT_FRAME_STRIDE=10
set TT_COLLECT_RECORD_LABELS=1
set TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1
set TT_COLLECT_WRITE_COCO=1
set TT_COLLECT_COCO_MIN_AREA_PX2=200
set TT_COLLECT_COCO_MAX_HEIGHT_RATIO=0.9

rem 自动采集配置
set TT_AUTO_COLLECT_ENABLE=1
set TT_AUTO_COLLECT_SECONDS_PER_RUN=60
set TT_AUTO_COLLECT_MAX_RUNS=84
set TT_AUTO_COLLECT_MODE=balanced
set TT_AUTO_COLLECT_TARGET_IMAGES_PER_CAMERA=10000
set TT_AUTO_COLLECT_COOLDOWN_S=5

python -m tp_tunnel_traffic.tests.test_auto_collect
```

策略说明：
- `balanced`：优先采集量最少的目标车（保证分布均衡）
- `cycle`：按 actor_id 顺序循环
- `random`：随机选车

输出文件：
- `dataset/batch_summary.json`：全局进度统计
- `dataset/auto_collect_config.json`：断点续跑状态（可中断后继续）

## GUI 使用说明

- **Overview**：总览自由视角
  - WASD：平面移动
  - Q/E：上升/下降
  - Shift：加速移动
  - 鼠标右键按住拖拽：旋转视角
  - 滚轮：缩放/前后移动
- **Ego View**：第一人称视角（选中某辆代理车后跟随）
- **选择代理车**：点击 GUI 左侧 Proxies 列表中的某一行
- **Collect Selected**：对当前选中代理车开启/关闭采集
  - ON 时会打印 `output_dir=...run_YYYYmmdd_HHMMSS`
  - 采集中切换选中代理车会自动 OFF，避免串数据
- **Record Video**：对当前选中代理车开启/关闭视频录制
- **HoloLens Stream**：启动/停止 HoloLens 2 WebRTC 推流

## 数据集采集输出结构

采集输出默认在项目根目录的 `dataset/`：

```text
dataset/
  proxy_<actor_id>/
    run_YYYYmmdd_HHMMSS/
      images/
        ego/
        front/
        front_left/
        front_right/
        left/
        right/
        rear/
      labels.jsonl
      metadata.json
      effective_config.json
      actors.json
      target.json
```

说明：
- `labels.jsonl` 每行一条记录；其 `frame/world_frame` 与 `images/**/<frame>.png` 同名对齐
- `metadata.json` 写入相机参数、stride、目标 actor_id、输出路径等
- `effective_config.json` 写入本次 run 的**可复现配置快照**（resolved config + TT_/TP_ 环境变量 + runtime 信息），便于后续定位“这批数据到底是用什么参数跑出来的”
- `actors.json` 记录本次 run 的采集目标 actor_id / blueprint_id / lane_id（便于离线分析）

离线校验（推荐每次采集后跑一次）：

```bat
conda activate carla
python -m tp_tunnel_traffic.validate_dataset_run --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
```

运行时会周期打印采集统计（用于确认“持续采集质量”）：
- `ticks/stride_ticks/writes`
- `miss_no_complete_frame`（采样点时没有凑齐完整相机帧）
- `miss_missing_camera`（选中帧缺少某个相机）
- `label_fail/image_fail`

### 阶段 B 推荐参数（稳定三车道 / 更高生成成功率）

为了减少“相邻车道互相卡住导致 proxy 生成失败”的情况，当前版本默认启用 **lane-aware** 的代理车生成净空判定：

- **同车道**：按纵向间距约束（仍使用 `TT_PROXY_SPAWN_CLEARANCE_M` 作为同车道净空），保持追尾安全。
- **跨车道**：使用更小的径向净空（默认 `2.5m`）+ 横向窗口（默认 `6.0m`），避免相邻车道车辆在入口附近互相阻塞。

推荐设置（环境变量）：

```bat
set TT_PROXY_ENABLE=1
set TT_PROXY_MIN_PER_LANE=2
set TT_PROXY_MAX_PER_LANE=4
set TT_PROXY_FOLLOW_DISTANCE_M=8
set TT_PROXY_SPAWN_START_RATIO=0.18
set TT_PROXY_SPAWN_CLEARANCE_M=12   rem 同车道纵向净空

rem lane-aware 生成净空（默认已开启；如需回退旧逻辑可设为 0）
set TT_PROXY_SPAWN_LANE_AWARE=1
set TT_PROXY_SPAWN_OTHER_LANE_CLEARANCE_M=2.5
set TT_PROXY_SPAWN_OTHER_LANE_LATERAL_M=6.0
set TT_PROXY_SPAWN_SAME_LANE_LATERAL_M=1.6
```

说明：主车入口净空仍由 `main.py` 的 `reserved_entry_m = TT_SPAWN_CLEARANCE_M + TT_PROXY_FOLLOW_DISTANCE_M + 6` 控制，lane-aware 只影响代理车之间的跨车道阻塞判断，不会放松入口预留区。

3. **GUI 选车与采集**：
   - 当前为 **proxy-only** 方案：不再生成旧主采集车。
   - 通过 GUI 选择代理车后，可切到该车第一人称视角；`Collect Selected` 控制该车采集开关。
   - 采集默认关闭，开启后会输出明确日志（目标 actor_id、ON/OFF、输出目录）。

4. **无碰撞保障**：
   - 采用“预采样 + 距离约束”策略，最大程度避免车辆在隧道内发生碰撞，确保采集过程的稳定性。

5. **代理车日志排查**：
    - 运行时会持续打印 `代理车流状态`、`代理车计划总数`、`代理车生成成功/失败`、`入口清理后剩余代理车` 等信息。
    - 如果看不到代理车，优先查看这些日志，判断是未开启 `TT_PROXY_ENABLE`、生成被入口净空过滤，还是 spawn 失败。

6. **视角切换**：
   - 默认 `TT_CAMERA_MODE=overview`，便于自由观察车流；点击代理车可切到该车第一人称观察。
   - 启用 `TT_GUI_ENABLE=1` 后，可在自定义窗口里点击按钮切换视角。
   - GUI 里会显示当前代理车列表，可点击代理车作为当前观察目标（第一视角）。
    - `Collect Selected` 只对当前选中的代理车采集；输出会按 `target_<id>` 分目录，避免混数据。
    - `Overview` 默认对准隧道入口，但可在界面里用 WASD/QE 自由移动观察视角。

7. **速度与车型真实性（最新）**：
   - 代理车基准速度由 `TT_PROXY_BASE_SPEED_MPS` 控制（默认 `12.0 m/s` ≈ `43.2 km/h`）。
   - 速度差不再固定 8%，改为按车道分层随机：
     - 左车道（更快）：`TT_PROXY_SPEED_DIFF_LEFT_MIN/MAX`（默认 `-8% ~ -4%`）
     - 中车道（中速）：`TT_PROXY_SPEED_DIFF_MID_MIN/MAX`（默认 `-2% ~ +2%`）
     - 右车道（更慢）：`TT_PROXY_SPEED_DIFF_RIGHT_MIN/MAX`（默认 `+4% ~ +9%`）
   - 车型过滤加强：排除 bike/motorbike/bus/truck/van，以及 `volkswagen.t2*` 等不符合隧道常规乘用车流的车型。

8. **采集目录创建时机（最新）**：
   - 点击代理车仅切换观察目标，不会立即创建该车的数据目录。
   - 只有在该车被选中且点击 `Collect Selected -> ON` 后，才会创建对应目录并开始写盘。
   - 每次 `Collect Selected -> ON` 会在 `proxy_<id>/run_YYYYmmdd_HHMMSS/` 下创建独立 run 目录，避免多次 ON/OFF 混写。
   - 这样可避免“浏览多个代理车就产生一堆空目录”。

10. **采集一致性（最新）**：
   - 标签与图像统一使用同一个 `world_frame` 作为主键对齐。
   - stride 仅控制“是否落盘该 frame”，队列仍每 tick 消费，避免多帧图像挤到单条标签。
   - 采集中如果切换选中代理车，系统会自动 `Collect OFF`，防止跨车辆串数据。

9. **持续车流观测（最新）**：
   - 支持通过 `TT_PROXY_TARGET_PER_LANE` 指定每条车道目标流量（`0` 表示自动）。
   - 控制台会持续输出 `代理车流状态`，并按 `TT_PROXY_DETAIL_LOG_INTERVAL_S` 周期打印所有代理车的详细状态（id/车道/路径索引/速度）。

## 常见问题

- GUI 不弹窗：先确认已在 `carla` 环境中 `pip install pygame numpy`，并设置 `TT_GUI_ENABLE=1`
- 没有代理车：确认 `TT_PROXY_ENABLE=1`；提高 `TT_PROXY_TARGET_PER_LANE`；观察控制台 `代理车详情` 日志
- Collect ON 后不写盘：
  - 先看控制台是否打印 `采集状态: 持续采集已启动` 以及周期性的 `采集统计`
  - 若 `miss_no_complete_frame` 快速增长，说明相机帧不齐（可尝试降低相机数量/分辨率或降低车流密度）
