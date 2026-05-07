# tp_tunnel_traffic 数据集采集模块设计文档

> 版本：Draft 0.1  
> 目标：基于当前 `tp_tunnel_traffic/` 的隧道三车道可视化与自动驾驶能力，扩展出一套可用于构建自动驾驶数据集的采集模块。

---

## 1. 背景与现状

当前 `tp_tunnel_traffic/` 已具备以下能力：

- 从 `QingShiLing.xodr` 提取隧道三车道轨迹。
- 三车道模型已归一化：以中间车道为基准，左右车道按常量车道宽度平移合成（默认 3.5m，可用 `TT_LANE_WIDTH_M` 覆盖），保证三条车道平行且等间距。
- 在 CARLA 隧道场景中绘制三车道车道线。
- 在中间车道起点生成车辆，并沿中间车道自动驾驶到终点。
- 使用驾驶人第一视角跟随自动驾驶车辆。
- 支持运行时热键旋转视角，便于观察环境。
- 已有 `tests/` 下的测试入口，用于单独验证三车道显示与自动驾驶流程。

这意味着当前模块已经适合作为**数据采集仿真底座**，只需在现有自动驾驶循环之上增加“传感器采集、代理车流、数据落盘”能力即可。

---

## 2. 目标定义

本模块的目标不是单纯完成自动驾驶，而是进一步形成一个**隧道自动驾驶数据集采集系统**，采集内容包括：

1. **目标车（采集对象）**
   - 由 GUI 选择某辆代理车作为采集目标（proxy-only）。
   - 采集多视角 RGB 图像。
   - 可选：输出实例分割 PNG（每个 RGB 相机对应一个 instance 相机）。

2. **代理车辆（环境交通）**
   - 在左右车道 / 中间车道上随机生成。
   - 按各自车道行驶，不做复杂变道。
   - 保证与主车及彼此之间尽量不碰撞。

3. **数据输出**
   - 每帧图像（多视角 RGB）。
   - 可选：实例分割 PNG（`*_instance` 相机）。
   - 每帧控制量（throttle / steer / brake）。
   - 每帧车辆状态（位置、速度、朝向）。
   - 可选：COCO 2D bbox（仅车辆）。
   - 场景元信息（XODR、采样参数、视角参数、版本信息）。

---

## 3. 可行性分析

### 3.1 可行部分

当前模块已经实现：

- 三车道轨迹提取（`lane_sampling.py`，其中左右车道由中间车道归一化合成）。
- 车辆起点生成与方向对齐（`main.py`）。
- 中间车道自动驾驶（`control.py` + `main.py`）。
- 第一人称视角跟随（`main.py`）。

因此，增加数据集采集功能属于**顺延式扩展**，不需要推翻现有结构。

### 3.2 难点与约束

1. **代理车辆无碰撞**
   - 完全随机且零碰撞不现实。
   - 推荐采用“约束随机 + 安全距离 + 碰撞重采样”的策略。

2. **多视角同步**
   - 需要在同一 tick 内同步采集多个相机图像。
   - 需保证图像、控制量、状态在时间上对齐。

3. **数据量与存储成本**
   - 7 个视角同时存图像会产生较大磁盘占用。
   - 需要预先定义分辨率和采样频率。

### 3.3 结论

该方案**可行**，且适合先做一个“最小可用版本（MVP）”：

- 主车：中间车道自动驾驶。
- 视角：1 个车内 + 6 个车外。
- 代理车：少量、约束生成、固定车道行驶。
- 采集：按 stride 写盘，输出 jsonl + 图片目录 + COCO（可选）。

### 3.4 当前已落地的 GUI 代理车采集

当前代码已经具备 GUI 代理车采集的最小实现骨架：

- `main.py` 中可由 GUI 的 `Collect Selected` 按钮控制采集器启停。
- `collector.py` 负责创建多视角相机、缓存图像、记录标签并在退出时销毁传感器；同时支持 COCO 2D 与 instance PNG 输出。
- `dataset_cameras.json` 定义了默认的车内视角 + 周围视角布局。
- `tests/test_tunnel_autodrive.py` 提供了一个专门的代理车流 + GUI 采集测试入口。

该模式的目标是：

1. 先生成稳定的三车道代理车流。
2. 通过 GUI 选择任意代理车作为观察/采集目标。
3. 用 `Collect Selected` 按钮控制该目标车的数据采集开关。

补充（当前已落地的关键语义）：

- **proxy-only**：不再生成旧“主采集车(ego)”流程，采集对象就是 GUI 选中的代理车
- **目录按需创建**：仅在 `Collect Selected -> ON` 时创建并写盘；仅选车观察不创建空目录
- **按 run 分段**：每次 ON 会在 `proxy_<id>/run_YYYYmmdd_HHMMSS/` 下创建独立 run 目录，避免多次 ON/OFF 混写
- **对齐主键统一为 world_frame**：标签与图像以 `world.get_snapshot().frame` 对齐
- **持续采集策略**：采样点如果严格同帧凑不齐相机，会自动选“最近且完整”的帧写盘，避免采集中断

### 3.5 稳定三车道代理流

当前策略是**构建稳定且持续补充的三车道代理交通流，并让 GUI 在代理车流上直接进行观察与采集**。当前策略如下：

- **代理车流预热 (Warmup)**：在隧道内先在左/中/右三条车道上生成一批代理车，并让其运行 `TT_PROXY_WARMUP_SECONDS` 时间；随后在仿真过程中持续补车，直到形成分布均匀、速度稳定的隧道交通流。
- **稳定三车道代理流**：代理车沿各自车道中心线行驶，维持设定的目标速度与安全间距，不进行变道。
- **无碰撞约束**：通过 `follow_distance_m`、生成间距检测和基于前车距离的速度调节，确保代理车之间保持安全距离。
- **GUI 采集**：在 GUI 中选中某辆代理车后，可通过 `Collect Selected` 开关控制该目标车的数据采集。
- **车型过滤**：优先生成普通轿车 / SUV / 掀背类车辆，剔除自行车、摩托车、卡车、救护车、面包车（含 `volkswagen.t2*`）等不适合隧道车流的车型。
- **速度分层**：代理车目标速度不再固定同一比例，按车道分层随机（左快中稳右慢），并在每辆车生成时一次采样后固定。

阶段 B 的核心逻辑：**先预热形成稳定流，再持续补车，并在 GUI 中选车按按钮采集。**

### 3.6 调试与日志

如果运行后看不到代理车，请先检查控制台输出：

- `代理车计划总数`：本次计划生成的代理车数量，以及左/中/右三车道分布。
- `代理车生成成功` / `代理车生成失败`：单车生成结果。
- `代理车基准速度`：当前流量基准速度（m/s 与 km/h）及跟车/前视参数。
- `代理车跳过`：通常表示该位置太靠近主车入口，或该车道点为空。
- `入口清理后剩余代理车`：warmup 前入口净空后的有效代理车数量。
- `Collect Selected: ON/OFF`：采集开关变化、当前目标代理车 actor_id、输出目录。
- `代理车已选中/已取消选择`：GUI 选车行为与采集目录切换。
- `代理车详情`：周期打印所有活跃代理车的 id/车道/路径索引/速度，便于全量观察车流状态。
- 采集中若切换选中代理车，系统会自动 `Collect OFF`，避免跨目标串数据。
- `主车进入前剩余代理车`：已不再使用主车切入流程；若看到此日志，说明旧输出仍未完全清理。

如果这些日志里 `计划总数` 很小，通常是环境变量的 `TT_PROXY_MIN_PER_LANE / TT_PROXY_MAX_PER_LANE` 设置偏低；如果大量显示“靠近主车入口”，说明入口净空配置过大。

### 3.7 视角模式

- 默认 `TT_CAMERA_MODE=overview`，便于自由观察车流；点击代理车可切到该车第一人称观察。
- 若要查看代理车流，可临时设置 `TT_CAMERA_MODE=overview` 切换为总览视角。
- 运行时也可按 `v` 在 ego / overview 两种视角之间切换。
- 启用 `TT_GUI_ENABLE=1` 后，可在 GUI 按钮面板里直接切换视角。
- GUI 会显示当前代理车辆列表，可点击代理车进行第一视角观察。
- `Collect Selected` 仅对当前选中代理车采集，推荐按 `target_<id>` 分目录保存，避免与其他目标混在一起。
- `Overview` 默认对准隧道入口，并支持 WASD/QE 自由移动与升降，查看全局车流。
- 默认采样频率为 0.5s 一次（`TT_COLLECT_FRAME_STRIDE=10`，默认同步步长 0.05s）。

### 3.8 关键参数（速度真实性）

- `TT_PROXY_BASE_SPEED_MPS`：代理车基准速度（默认 `12.0 m/s`，约 `43.2 km/h`）。
- `TT_PROXY_SPEED_DIFF_LEFT_MIN/MAX`：左车道速度差百分比区间（默认 `-8 ~ -4`，更快）。
- `TT_PROXY_SPEED_DIFF_MID_MIN/MAX`：中车道速度差百分比区间（默认 `-2 ~ +2`，中速）。
- `TT_PROXY_SPEED_DIFF_RIGHT_MIN/MAX`：右车道速度差百分比区间（默认 `+4 ~ +9`，更慢）。

> 说明：速度差采用 `target = base * (1 - diff/100)`，因此负值表示比基准更快，正值表示更慢。

默认采集输出目录为项目根目录 `dataset/`（可通过 `TT_COLLECT_OUTPUT_DIR` 覆盖）。

目录创建策略：仅当选中某代理车并点击 `Collect Selected` 开启采集时，才创建该目标目录并写盘；仅选车观察不会创建空目录。

每次 `Collect ON` 使用独立 run 子目录（`proxy_<id>/run_YYYYmmdd_HHMMSS/`），避免同一目录多段采集混写。

采集对齐策略：标签与图像统一使用 `world_frame`；stride 仅控制是否落盘该帧，队列每 tick 都会消费，避免“多帧图像对应单条标签”的静默错配。

### 3.9 采集质量统计（已落地）

采集器会维护并周期输出统计（用于验收“持续采集质量”与定位瓶颈）：

- `ticks`：采集器收到的 tick 次数
- `stride_ticks`：满足 stride 的采样点次数
- `writes`：成功写盘的样本数
- `miss_no_complete_frame`：采样点时没有凑齐“所有相机齐全”的帧
- `miss_missing_camera`：选中的写盘帧缺少某个相机（理论上应接近 0）
- `label_write_fail` / `image_write_fail`：I/O 写入失败计数

默认每 200 ticks 打印一次 `采集统计`。

### 3.10 Run 元信息增强（已落地）

- `metadata.json` 现在包含：map_name / 客户端版本 / 平台信息 / created_at_unix / sync_mode / fixed_delta_seconds 等
- `metadata.json` 记录 COCO/instance 开关与过滤参数（min_area/max_height_ratio）
- `labels.jsonl` 现在包含：speed_mps / vehicle_yaw（并支持通过 `TT_COLLECT_RECORD_LABELS=0` 关闭写 label）
- `actors.json`：run 级目标信息（actor_id / blueprint_id / lane_id）
- `effective_config.json`：run 级可复现配置快照（resolved config + TT_/TP_ 环境变量 + runtime 信息）

### 3.11 自动化批量采集（已落地）

通过 `tests/test_auto_collect.py` 实现全自动采集：

- **入口**：`python -m tp_tunnel_traffic.tests.test_auto_collect`
- **无 GUI 模式**：程序化选车→采集→换车循环，无需人工干预
- **选车策略**：balanced（均衡）/ cycle（顺序）/ random（随机）
- **resume 能力**：通过 `auto_collect_config.json` 支持中断后断点续跑
- **全局统计**：`batch_summary.json` 记录每 run 的样本数/帧范围/目标车/时长
- **目标收敛**：通过 `TT_AUTO_COLLECT_TARGET_IMAGES_PER_CAMERA` 控制产出总量

关键配置：
- `TT_AUTO_COLLECT_ENABLE`：开启自动采集
- `TT_AUTO_COLLECT_SECONDS_PER_RUN`：每辆车采集时长
- `TT_AUTO_COLLECT_MAX_RUNS`：最大 run 数
- `TT_AUTO_COLLECT_MODE`：选车策略
- `TT_AUTO_COLLECT_TARGET_IMAGES_PER_CAMERA`：目标图像数（达到即停）

### 3.12 全局 COCO 合并（已落地）

通过 `tp_tunnel_traffic/merge_coco.py` 将全部 run 的独立 `coco_instances.json` 合并为一个 `coco_annotations.json`：

- **入口**：`python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset`
- **输出**：`dataset/coco_annotations.json`（image_id / annotation_id 全局唯一）
- **路径重写**：`file_name` 统一转为相对数据集根目录的路径


---

## 4. 目标数据集结构

当前按目标车辆与 run 组织数据：

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
        *_instance/            (optional)
      labels.jsonl
      labels_2d/
        coco_instances.json    (optional)
      metadata.json
      actors.json
      target.json
      effective_config.json    (optional)
```

### 4.1 图像内容

- `ego/`：车内第一人称视角。
- `front/`：车前方视角。
- `front_left/`：左前方视角。
- `front_right/`：右前方视角。
- `left/`：左侧视角。
- `right/`：右侧视角。
- `rear/`：后方视角。

### 4.2 标签内容（labels.jsonl）

建议每帧一行 JSON：

```json
{
  "frame": 120,
  "world_frame": 120,
  "throttle": 0.38,
  "steer": -0.04,
  "brake": 0.0,
  "speed_mps": 7.21,
  "vehicle_x": 123.45,
  "vehicle_y": 67.89,
  "vehicle_z": 1.2,
  "vehicle_yaw": 89.2
}
```

### 4.3 元信息（metadata.json）

建议记录：

- XODR 路径与地图名。
- 采样步长与同步信息（fixed_delta_seconds/sync_mode）。
- 相机外参/成像参数。
- COCO 与 instance 输出开关及过滤参数。
- CARLA 版本与运行环境信息。

---

## 5. 当前模块的职责拆分

### 5.1 `lane_sampling.py`

负责：

- 从 XODR 中读取道路几何。
- 读取 laneOffset / lane width。
- 生成三车道中心线：以中间车道为基准，左右车道按常量车道宽度平移合成（归一化三车道模型）。
- 识别起点 / 终点 / 有效三车道区段。

在数据集场景中，它还应提供：

- 主车的中间车道中心线。
- 代理车可用的左/中/右车道轨迹。

### 5.2 `control.py`

负责：

- 根据前视点计算转向。
- 控制速度与油门。
- 抑制抖动。

在采集任务中，它保持不变，作为**主车自动驾驶控制器**。

### 5.3 `main.py`

负责：

- CARLA 连接。
- 车辆生成。
- 自动驾驶循环。
- 第一人称视角跟随。

未来需要扩展为：

- 多摄像头挂载。
- 每 tick 采样并落盘。
- 代理车辆管理。

### 5.4 `tests/`

当前已有：

- `test_tunnel_lanes.py`：检查三车道是否完整。
- `test_tunnel_autodrive.py`：验证自动驾驶是否正常。

未来建议新增：

- `test_dataset_capture.py`：验证图像和标签落盘逻辑。

---

## 6. 采集方案设计

### 6.1 主车（数据采集车）

主车应满足：

- 从三车道起点生成。
- 按中间车道自动驾驶。
- 保持第一人称视角。
- 按 tick 保存图像和状态。

建议采集数据：

- 车内视角图像。
- 当前速度。
- 当前 steer / throttle / brake。
- 当前位姿（x, y, z, yaw）。
- 当前车道 id。

### 6.2 代理车辆

代理车建议分两层：

#### 第一层：最小可行代理车流

- 在左/中/右车道固定位置生成 0~N 辆。
- 只保持前进，不做复杂变道。
- 控制间距大于安全阈值。

#### 第二层：增强代理车流

- 按规则随机生成。
- 可以在不同车道上配置不同速度。
- 支持稀疏 / 中等 / 稠密三档流量。

### 6.3 安全策略

- 新车生成前检查与已有车辆最小距离。
- 若冲突则跳过或重采样。
- 主车优先，代理车不影响主车路径。
- 发生碰撞则记录事件并结束当前 episode。

---

## 7. 采集流程建议

### Step 1：初始化

- 读取 XODR。
- 生成三车道轨迹。
- 设置主车起点。
- 初始化相机参数。

### Step 2：生成代理车辆

- 在三车道中按规则随机生成。
- 设定安全距离。
- 预设初速度 / 目标速度。

### Step 3：启动主车自动驾驶

- 沿中间车道前进。
- 每 tick 更新控制。

### Step 4：同步采集

每个 tick：

- 采集 7 张图像。
- 记录主车控制量。
- 记录主车位姿和速度。
- 记录代理车状态。

### Step 5：结束与落盘

- 按终点 / 时间上限 / 碰撞条件结束。
- 生成 episode 文件夹。
- 保存 metadata 与 labels。

---

## 8. 推荐的实现顺序

### 阶段 A：最小可用版本

1. 主车自动驾驶保持不变。
2. 加 1 个车内 + 6 个外部相机。
3. 每 tick 落盘一帧图像 + 一行标签。
4. 不加代理车。

### 阶段 B：加入代理车辆

1. 固定车道生成少量代理车。
2. 加安全距离与重采样。
3. 保存代理车状态。

### 阶段 C：数据集完善

1. 支持多 episode。
2. 支持配置化采样频率。
3. 支持失败回滚与质量筛除。

---

## 9. 风险与注意事项

1. **图像同步**
   - 必须在同一个 tick 中保存图像和状态，避免错位。

2. **相机数量**
   - 7 个相机同时采集会加重性能压力，建议先从低分辨率开始。

3. **代理车安全**
   - 初期不要追求高密度车流，先确保无碰撞。

4. **数据一致性**
   - 每个 episode 必须保存相同字段格式。

5. **终点与方向**
   - 当前三车道轨迹已经做了方向修正，数据集采集应以“新的三车道起点”为起始点。

---

## 10. 结论

基于当前 `tp_tunnel_traffic/` 的实现，构建“隧道三车道多视角自动驾驶数据集”是可行的。

最推荐的落地路线是：

1. 保持现有三车道与自动驾驶逻辑。
2. 增加多相机采集。
3. 先不引入复杂车流，只做少量代理车。
4. 先产出稳定的 episode 数据目录。

如果后续要继续推进，建议先从**数据格式与采集入口设计**开始，再进入具体代码实现。
