# TunnelAutopilot-Tunnel (TAT) Dataset

本目录为 **CARLA 隧道场景**下采集的自动驾驶数据集（由 `tp_tunnel_traffic` 生成）。

本 README 参考 nuScenes / KITTI 的组织方式，提供：
- 数据集概览（Dataset Card）
- 目录结构与文件格式（Schema）
- 传感器/对齐规则（Synchronization）
- 训练拆分建议（Splits）
- 质量校验方式（Validation）

> 本数据集实际产出：7 相机 × 3,504 张 RGB + 3,504 张实例分割 + COCO 2D 框（车辆），可直接用于检测/分割/BC 训练。

---

## 1. Dataset Card

- **Name**: TunnelAutopilot-Tunnel (TAT)
- **Version**: 2026-05-06
- **Domain**: CARLA tunnel (QingShiLing.xodr), 3-lane traffic
- **Collection mode**: proxy-only, balanced auto-collection (6 batches × varied density/speed)
- **Sensors**: 7× RGB cameras + 7× instance segmentation cameras
- **Supervision**: control + ego state + 2D bbox (COCO) + instance mask (PNG)
- **Primary key**: `world_frame` (CARLA world snapshot frame)
- **Statistics**:

| 指标 | 值 |
|---|---|
| 总 run 数 | 61 |
| 采集目标车辆数 | 61 辆（每车 1 run） |
| RGB 图像 / 相机 | 3,504 |
| RGB 总图像 | 24,528 |
| 实例分割 PNG | 24,528（100% 配对） |
| COCO images | 24,528 |
| COCO annotations | 27,763 |
| COCO categories | 1（vehicle） |
| 平均标注/图 | 1.13 |
| 每 run 平均时长 | ~180s |
| 采集时间跨度 | 2026-05-06 19:26 ~ 约 3 小时 |

- **Recommended tasks**:
  - Behavior Cloning (BC): image(s) → steer/throttle/brake
  - 2D Object Detection (COCO)
  - Instance Segmentation
  - Speed/yaw regression (aux)
  - Trajectory/control quality analysis

---

## 2. Directory Structure

数据按 **采集目标（代理车 actor_id）** 与 **run** 组织：

```text
dataset/
  coco_annotations.json              ← 全局 COCO（合并所有 run）
  batch_summary.json                 ← 批量采集统计
  auto_collect_config.json           ← 续跑状态
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
        ego_instance/
        front_instance/
        front_left_instance/
        front_right_instance/
        left_instance/
        right_instance/
        rear_instance/
      labels.jsonl
      labels_2d/
        coco_instances.json
      metadata.json
      target.json
      effective_config.json        (optional; new versions)
      actors.json                  (optional; new versions)
```

说明：
- **一个 run = 一段连续采集片段**（通过 GUI `Collect Selected` -> ON 创建）。
- 建议训练/验证/测试按 **run 粒度**切分，避免同一片段泄漏到不同集合。

---

## 3. Sensor Suite

### 3.1 Cameras
相机列表来自 `metadata.json -> cameras[]`。默认相机名（也就是 `images/` 下的子目录名）为：

- `ego`
- `front`, `front_left`, `front_right`
- `left`, `right`
- `rear`

每个相机在 `metadata.json` 中包含：
- `name`
- **安装位姿（相对车体）**：`x, y, z, pitch, yaw, roll`
- **成像参数**：`width, height, fov`

> 业界数据集通常会提供 `intrinsics (K)` / `distortion (D)` / `T_ego_cam` 矩阵。当前数据集中未以矩阵形式提供，仅提供安装位姿与 fov（足以做端到端控制，但不够做多相机几何任务）。

### 3.2 Instance Segmentation Cameras
当开启实例分割采集时，每个 RGB 相机自动生成对应的 instance segmentation 相机：
- 名称后缀由 `TT_COLLECT_INSTANCE_SUFFIX` 控制（默认 `_instance`）
- 例：`ego` → `ego_instance`，`front` → `front_instance`
- 输出目录：`images/<camera_name_with_suffix>/` 下保存 PNG
- **本数据集已为全部 7 个 RGB 相机生成了 instance PNG**，共 24,528 张。

---

## 4. Synchronization & Keying (Images ↔ Labels)

本数据集使用 `world_frame` 作为跨文件的对齐主键：

- `labels.jsonl` 每行包含 `world_frame`
- 对应图像路径：

```text
images/<camera_name>/<frame>.png
```

其中 `<frame>` 使用**零填充到至少 6 位**的文件名：
- 例如 `world_frame=17759` → `017759.png`
- 例如 `world_frame=1309488` → `1309488.png`

对齐伪代码（与校验工具一致）：

```python
frame = label.get("world_frame", label.get("frame"))
fname = f"{int(frame):06d}.png"
path = f"images/{cam}/{fname}"
```

### 4.1 Time (dt / timestamps)

- `world_frame` 是 CARLA 的 snapshot frame id，**不是 wall-clock timestamp**。
- 当 `sync_mode=true` 时，推荐用 `metadata.json` 中的 `fixed_delta_seconds` 作为离线时间步长（dt）。
- 本数据集**不保证**逐样本写入 `timestamp` 字段（目前未写盘）。如果你需要显式时间戳，建议后续在标签里补 `timestamp = world_frame * fixed_delta_seconds` 或写入 `world_snapshot.timestamp.elapsed_seconds`。

---

## 5. Data Schema

### 5.1 labels.jsonl
JSON Lines（**每行一个样本**）。字段（以实际写盘为准）：

| 字段 | 类型 | 含义 |
|---|---|---|
| `world_frame` | int | 样本主键（CARLA snapshot frame） |
| `frame` | int | 与 `world_frame` 同值（兼容） |
| `actor_id` | int | 采集目标车辆 actor id |
| `nearest_idx` | int | 目标车在车道点序列上的最近点索引（调试/分析） |
| `throttle` | float | 油门 |
| `steer` | float | 转向 |
| `brake` | float | 刹车 |
| `hand_brake` | bool | 手刹 |
| `reverse` | bool | 倒车标记 |
| `vehicle_x/y/z` | float | 车辆世界坐标 |
| `vehicle_yaw` | float | 车辆 yaw（度；可选，旧 run 可能缺失） |
| `speed_mps` | float | 车速（m/s；可选，旧 run 可能缺失） |

控制量范围（CARLA 约定）：
- `steer` ∈ [-1, 1]
- `throttle` ∈ [0, 1]
- `brake` ∈ [0, 1]

示例（单行，来自旧 run，字段较少）：

```json
{"frame": 17759, "world_frame": 17759, "nearest_idx": 1414, "actor_id": 15, "throttle": 0.5280, "steer": -0.0057, "brake": 0.0, "hand_brake": false, "reverse": false, "vehicle_x": 663.7662, "vehicle_y": -667.2275, "vehicle_z": -0.0456}
```

### 5.2 metadata.json
run 级元信息（常见字段）：

| 字段 | 类型 | 含义 |
|---|---|---|
| `xodr_path` | str | OpenDRIVE 路径 |
| `map_name` | str | CARLA 地图名（新版本） |
| `carla_client_version` | str | CARLA client 版本（新版本） |
| `platform` | object | python/OS 信息（新版本） |
| `created_at_unix` | float | run 创建时间（秒，Unix time，新版本） |
| `output_dir` | str | run 输出目录 |
| `cameras` | list | 相机列表与安装/成像参数 |
| `instance_segmentation` | object | 是否开启实例分割与后缀 |
| `coco_2d` | object | COCO 2D 输出开关与类别 id |
| `step_m` | float | 车道采样步长 |
| `collect_frame_stride` | int | 采样 stride（每 N tick 写一次） |
| `fixed_delta_seconds` | float | 同步模式 dt（新版本） |
| `sync_mode` | bool | 是否同步模式（新版本） |
| `target_actor_id` | int | 采集目标 actor |

> 兼容性说明：不同 run 可能来自不同版本的采集逻辑；metadata 字段可能会随版本演进而增减。

### 5.3 target.json
run 级轻量标记文件：

| 字段 | 类型 | 含义 |
|---|---|---|
| `target_actor_id` | int | 采集目标 actor |
| `output_dir` | str | run 输出目录 |

### 5.4 effective_config.json（可选）
用于可复现实验的配置快照（新版本 run 可能包含）：
- `effective_config`：resolved 的 `TunnelTrafficConfig`（等价于 dataclass asdict）
- `env`：采集时刻 `TT_*/TP_*` 环境变量快照
- `runtime`：argv/python/conda 环境等运行时信息

典型字段（顶层）：
- `created_at_unix` (float)
- `output_dir` (str)
- `effective_config` (object)
- `env` (object)
- `runtime` (object)

### 5.5 actors.json（可选）
run 级目标 actor 的补充信息（新版本 run 可能包含）。

---

## 6. Create the Dataset (采集流程)

### 6.1 环境准备（Windows CMD）

你可以用你当前习惯的方式激活环境，例如：

```bat
E:\programs\miniconda\Scripts\activate && conda activate carla
```

更推荐（更稳）的一键方式：

```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla
```

### 6.2 运行与采集（概述）
1. 启动 `tp_tunnel_traffic`（开启代理车流 + GUI）。
2. GUI 左侧 `Proxies` 列表选择一辆代理车作为采集目标。
3. 点击 `Collect Selected` -> ON 创建新的 `run_*/` 并开始写盘。
4. 需要换车采集：先 OFF，再换车，再 ON（系统也会在切换目标时自动 OFF，避免串数据）。

### 6.3 推荐的“隧道车流 + 可采集”配置要点

采集相关：
- `TT_GUI_ENABLE=1`（开启 GUI）
- `TT_COLLECT_ENABLE=1`（启用采集模块）
- `TT_COLLECT_FRAME_STRIDE=10`（默认每 10 tick 采样一次；与 `TT_FIXED_DELTA_SECONDS` 联动）

车流相关（proxy-only 隧道更稳定、分布更像真实车流）：
- `TT_PROXY_ENABLE=1`
- `TT_PROXY_BASE_SPEED_MPS` / `TT_PROXY_FOLLOW_DISTANCE_M` 控制速度与跟车距离

更完整的“一键启动命令/配方”请参考：`tp_tunnel_traffic/README.md`。

---

## 7. Splits (训练/验证/测试切分建议)

nuScenes/KITTI 风格通常强调**按场景/片段**切分，避免帧级泄漏。建议：

- **按 run 切分**：同一个 `run_*` 只能属于 train 或 val 或 test。
- 若数据量足够：按 `proxy_<id>` 分组切分（不同 actor_id 更不易泄漏）。

示例（仅示意，不强制落盘文件）：
- train: 70%
- val: 15%
- test: 15%

---

## 8. Validation (质量校验)

使用内置离线校验工具检查 `metadata.json`、`labels.jsonl` 与图像对齐：

```bat
conda activate carla
cd /d E:\code\track_plot

python -m tp_tunnel_traffic.validate_dataset_run --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
```

校验逻辑：读取 `metadata.json` 的相机列表，并用 `labels.jsonl` 的 `world_frame` 检查每个相机是否存在同名 PNG。

COCO/instance 输出校验（新增）：
```bat
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
```

抽检样本导出（生成 inspect_samples.json）：
```bat
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS --sample-cameras ego,front --sample-limit 3
```

建议额外做的质量检查（nuScenes/KITTI 常见实践）：
- 帧间隔统计：`world_frame` 是否严格按 `collect_frame_stride` 递增（允许少量丢帧）
- 分布统计：速度/转向角/刹车事件比例，避免“全程直行”或“全程同油门”
- run 级去重：避免同一 actor_id 的极短 run 过多（训练会偏）

---

## 9. Auto Collection (批量自动化采集)

用于大规模无人工干预的数据采集：

```bat
python -m tp_tunnel_traffic.tests.test_auto_collect
```

关键配置：

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `TT_AUTO_COLLECT_ENABLE` | 0 | 开启自动采集模式 |
| `TT_AUTO_COLLECT_SECONDS_PER_RUN` | 60 | 每辆车采集时长（秒） |
| `TT_AUTO_COLLECT_MAX_RUNS` | 20 | 最多采集多少个 run |
| `TT_AUTO_COLLECT_MODE` | balanced | 选车策略：balanced/cycle/random |
| `TT_AUTO_COLLECT_MIN_SPEED_MPS` | 5.0 | 最低速度阈值（跳过静止车） |
| `TT_AUTO_COLLECT_TARGET_IMAGES_PER_CAMERA` | 0 | 每相机目标图像数（0=不限） |
| `TT_AUTO_COLLECT_COOLDOWN_S` | 5 | run 间冷却时间 |

自动化输出：
- `dataset/batch_summary.json`：全局统计（每相机图像数/run 数/目标车）
- `dataset/auto_collect_config.json`：断点续跑状态

---

## 10. Limitations (对标业界数据集的缺口)

对齐 nuScenes/KITTI 的“可迁移/可做几何/感知/定位”的标准，本数据集目前主要缺口为：

1. **时间戳与严格同步约定**：当前以 `world_frame` 为主键，未显式输出逐帧 timestamp。
2. **标定与坐标系字典**：未提供矩阵形式的内参/外参与坐标轴约定文档化。
3. **感知标注**：已有 2D bbox（COCO）与实例分割（PNG），但缺少 3D detection、深度、语义分割等。

这些缺口不会影响端到端控制/BC，但会限制几何一致性、多相机融合、感知任务训练。

---

## 11. Coordinate Frames & Units (建议写在论文/报告里)

### 10.1 CARLA / Unreal 坐标系（简述）

CARLA 基于 Unreal：
- 常见约定下，世界坐标轴可按 `+X` 前、`+Y` 右、`+Z` 上理解。
- 旋转角：`yaw/pitch/roll` 使用**度**。

> 注：不同版本/接口文档对“手性（左手/右手）”的表述容易引起混淆。对于几何/融合任务，请以你实际使用的 CARLA 版本官方文档与 `carla.Transform` 行为为准。

本数据集的 `vehicle_x/y/z` 为 CARLA world 坐标（单位：米）。

### 10.2 相机外参（当前提供方式）

`metadata.json -> cameras[]` 里给出每个相机相对车体的安装位姿（x/y/z + pitch/yaw/roll）。
这适合端到端学习与可视化复现。

### 10.3 相机内参（可由 fov/分辨率推导）

当前每个相机提供 `width/height/fov`，可按 pinhole 近似推导：

```text
fx = width  / (2 * tan(fov/2))
fy = height / (2 * tan(fov/2))    # 若假设水平/垂直 fov 相同
cx = width/2
cy = height/2
```

若你要做几何/重建任务，建议将 `K`、`T_ego_cam` 以矩阵形式显式写盘。

---

## 12. License & Citation

- **License**: 未声明（建议在发布/共享前补充）
- **Citation**: 若要在论文/报告引用，建议补充一个 BibTeX 条目（待定）
### 5.6 labels_2d/coco_instances.json（可选）
当开启 COCO 输出时，每个 run 会生成独立的 COCO 文件。
> `file_name` 使用 **相对路径**（相对于 run 目录），例如 `images/ego/000123.png`。
> 2D 框仅包含车辆（vehicle.*），这是本项目当前目标范围。

### 5.7 coco_annotations.json（全局合并）
将全部 run 的 COCO 合并为一个文件，image_id / annotation_id 全局唯一：
```bat
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset
```
输出 `dataset/coco_annotations.json`，可直接喂入检测框架。
2D bbox / 实例分割：
- `TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1`（生成 instance PNG）
- `TT_COLLECT_WRITE_COCO=1`（生成 COCO 2D JSON）
- `TT_COLLECT_INSTANCE_SUFFIX=_instance`（分割相机后缀）
- `TT_COLLECT_COCO_MIN_AREA_PX2=200`（过滤过小的 2D 框面积）
- `TT_COLLECT_COCO_MAX_HEIGHT_RATIO=0.9`（过滤过高的 2D 框，占图像高度比例）
