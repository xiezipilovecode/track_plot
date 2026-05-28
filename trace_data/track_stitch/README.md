# track_stitch：六路摄像头隧道车辆轨迹拼接

将六路隧道摄像头（TV023-TV028）独立检测的车辆轨迹片段，通过时空约束 + 匈牙利算法拼接为完整的跨摄像头轨迹。

## 核心能力

- **数据解析**：解析 3,186 个 txt 文件（531 文件/摄像头 × 6 路），支持 7 种车辆类型
- **质量过滤**：(0,0)异常点过滤、静止轨迹检测、单节点轨迹排除
- **同摄像头合并**：跨 30 分钟时间窗口的轨迹片段合并（Step 0）
- **匈牙利匹配**：时间窗口分批 + 预筛选 + `scipy.optimize.linear_sum_assignment` 全局最优
- **盲区插值**：摄像头盲区匀速/线性插值，标记 `interpolated=True`
- **质量评分**：基于摄像头覆盖数、插值比、匹配置信度的综合评分（0-1）

## 快速开始

```powershell
cd E:\code\track_plot

# 完整运行（全量 3,186 文件，约 13 分钟）
python -m trace_data.track_stitch.main

# 快速测试（每摄像头 10 文件，约 30 秒）
python -m trace_data.track_stitch.main --quick

# 指定参数
python -m trace_data.track_stitch.main --data-root <path> --output <path> --verbose
```

## 模块结构

```text
trace_data/track_stitch/
├── __init__.py          # 包入口，导出 StitchingConfig / Stitcher
├── config.py            # 配置：StitchingConfig @dataclass，全部算法参数
├── models.py            # 数据模型：TrajectoryNode、Trajectory、CameraDataset、StitchedTrajectory
├── parser.py            # 解析器：文件扫描、节点解析、(0,0)过滤、同摄像头合并
├── matcher.py           # 匹配器：代价矩阵、预筛选、匈牙利算法、时间窗口分批
├── stitcher.py          # 编排引擎：解析→合并→匹配→插值→组装→导出 JSON
├── main.py              # CLI 入口：argparse，支持 --quick / --verbose
├── visualize.py         # 可视化：4 面板综合图（时距图+覆盖分布+质量评分+精选轨迹）
└── output/              # 输出目录（stitched_trajectories.json + stats + 可视化图片）
```

## 各模块详解

### `config.py` — 配置（~90 行）

`StitchingConfig` 为 frozen dataclass，所有参数集中管理：

| 类别 | 关键参数 | 默认值 |
|------|----------|--------|
| 数据 | `data_root` | `trace_data/track24.7.25-24.8.5_79_6lu/` |
| 摄像头 | `cameras` | `[TV023, TV024, TV025, TV026, TV027, TV028]` |
| 过滤 | `min_nodes_per_track` | 2（单节点排除） |
| 静止判定 | `static_y_std_cm` / `static_speed_kmh` | 100cm / 5km/h |
| 速度转换 | `speed_kmh_to_cm_ms` | 1/36.0 |
| 匹配 | `max_time_gap_ms` / `max_y_gap_cm` | 120000ms / 25000cm |
| 代价权重 | `weight_time/position/speed/type` | 1.0 / 1.0 / 0.5 / 10.0 |
| 插值 | `interp_max_gap_ms` / `interp_points_per_gap` | 30000ms / 10 |
| 性能 | `time_window_minutes` | 30（分批大小） |
| 输出 | `output_json` / `output_stats` | `stitched_trajectories.json` |

### `models.py` — 数据模型（~250 行）

**`TrajectoryNode`**：单个轨迹节点（6 字段）
- `from_tokens(tokens, idx)` 类方法：从空格分隔的 token 列表解析节点
- (0,0) 检测：`abs(x) < 1.0 and abs(y) < 1.0` → `is_valid=False`
- `to_dict()`：输出为 JSON 兼容格式，插值节点 `x=null`

**`Trajectory`**：单摄像头内的轨迹片段
- `majority_type`：多数投票车辆类型（解决同一轨迹内类型不恒定的问题）
- `is_static`：静止检测（y 标准差 < 100cm 且 平均速度 < 5km/h）
- `is_multi_node`：有效节点数 ≥ 2
- `first_valid()` / `last_valid()`：跳过无效节点获取首/末有效节点

**`CameraDataset`**：单摄像头数据集
- `get_active_trajs()`：获取可用于匹配的活跃轨迹（多节点 + 非静止）
- `to_summary()`：统计摘要

**`StitchedTrajectory`**：拼接后的完整轨迹
- `camera_count`：实际涉及的摄像头数（不含 INTERP）
- `interp_ratio`：插值节点占比
- `matched_pairs`：`[(前摄像头, 后摄像头, 代价), ...]`
- `to_dict()`：完整 JSON 序列化

### `parser.py` — 解析器（~230 行）

**文件扫描**：`scan_data_directory()` — 按摄像头 ID 分组，支持每摄像头 `max_files` 限制

**文件解析**：`parse_file()` — 逐行读取，每 6 个 token 一组调用 `TrajectoryNode.from_tokens()`

**批量解析**：`parse_all_files()` — 遍历所有文件，构建 `{camera_id: CameraDataset}` 字典

**同摄像头合并**（Step 0）：`merge_within_camera()`
- 按窗口时间排序，在相邻窗口边界附近搜索时空匹配的轨迹对
- 约束：时间差 ≤ 60s、y差 ≤ 5000cm、类型一致
- 合并后清除缓存重新计算 `majority_type`

### `matcher.py` — 匹配器（~270 行）

**代价函数**：`_compute_pair_cost()`
```
cost = time_diff/1000 + |y_diff|/100 + speed_consistency + type_mismatch_penalty
```
- 时间差归一化到秒，位置差归一化到米
- 速度一致性：实际时间差 vs 基于末速度的期望时间差
- 类型不匹配罚款：`10.0 × 50.0 = 500`

**预筛选**：`_prefilter()` — 宽松时空+类型约束，快速排除明显不匹配对

**时间窗口分批**：`_partition_by_time()`
- 30 分钟滑动窗口，相邻窗口重叠 2 分钟
- 解决 O(n³) 内存爆炸（全量 89687²×8B = 60GB）

**单批匹配**：`_match_one_batch()` — 5000×5000 安全阀

**去重**：每个 `traj_b` 仅保留代价最小的匹配

### `stitcher.py` — 编排引擎（~320 行）

**核心流程**：

```
Step 0+1: scan_data_directory() → parse_all_files() → 统计
Step 0b:  merge_within_camera() × 6 cameras
Step 2:   match_all_pairs() → 5 对摄像头匈牙利匹配
Step 3:   _assemble_trajectories() → BFS 链式拼接
Step 4:   _interpolate_gaps() → 盲区线性/匀速插值
Step 5:   _compute_quality() → 综合评分 + sort + export
```

**链式组装**：从 TV023 起始轨迹出发，沿匹配图 BFS 贪心追溯，选代价最小的路径

**质量评分**：
```
quality = 0.3×completeness + 0.3×continuity + 0.2×match_confidence + 0.2×camera_bonus
```

## 输出格式

```json
{
  "metadata": {
    "source": "trace_data/track24.7.25-24.8.5_79_6lu",
    "cameras": ["TV023", "TV024", "TV025", "TV026", "TV027", "TV028"],
    "stats": { "total_files": 3186, "stitched_trajectories": 70397, ... }
  },
  "trajectories": [
    {
      "trajectory_id": "TRAJ_000000",
      "vehicle_type": "car",
      "camera_count": 3,
      "interp_ratio": 0.4762,
      "quality_score": 0.6942,
      "nodes": [
        {
          "timestamp": 1721891126552,
          "x": 216.79,
          "y": 61571773.8,
          "speed": 27.420301,
          "acc": null,
          "type": "car",
          "camera": "TV023",
          "interpolated": false
        },
        {
          "timestamp": 1721891132193,
          "x": null,
          "y": 61582824.19,
          "speed": 72.385119,
          "acc": null,
          "type": "car",
          "camera": "INTERP",
          "interpolated": true
        }
      ],
      "matched_pairs": [
        ["TV023", "TV024", 122.51],
        ["TV024", "TV025", 89.34]
      ]
    }
  ]
}
```

**节点字段说明**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | int | Unix 毫秒时间戳 |
| `x` | float\|null | 像素 x 坐标，插值节点为 null |
| `y` | float | 世界 y 坐标（cm） |
| `speed` | float\|null | 瞬时速度（km/h） |
| `acc` | float\|null | 加速度 |
| `type` | str | 车辆类型（car/truck/tanker/bus/pika/van/motorbike） |
| `camera` | str | 来源摄像头 ID，插值节点为 "INTERP" |
| `interpolated` | bool | 是否为插值节点 |

## 数据特征

| 摄像头 | 公里桩 | 原始轨迹数 | y 坐标范围（cm） |
|--------|--------|-----------|-----------------|
| TV023 | K615+703 | 129,123 | 61,571,759 – 61,583,374 |
| TV024 | K615+833 | 105,004 | 61,585,033 – 61,594,533 |
| TV025 | K615+963 | 120,291 | 61,597,688 – 61,613,901 |
| TV026 | K616+093 | 96,711 | 61,610,798 – 61,632,611 |
| TV027 | K616+223 | 254,289 | 61,623,864 – 61,648,173 |
| TV028 | K616+323 | 202,896 | 61,633,315 – 61,654,109 |

## 全量运行结果

| 指标 | 数值 |
|------|------|
| 解析文件 | 3,186 个（531/摄像头 × 6） |
| 原始轨迹片段 | 908,314 条 |
| 同摄像头合并后 | 565,006 条 |
| 匹配对（去重后） | 355,082 对 |
| **拼接完整轨迹** | **70,397 条** |
| 插值节点 | 2,235,190 个 |
| 运行时间 | ~13.4 分钟 |

**摄像头覆盖分布**：

| 覆盖摄像头数 | 轨迹数 | 占比 |
|-------------|--------|------|
| 6（全覆盖） | 35,637 | 50.6% |
| 3 | 26,320 | 37.4% |
| 5 | 7,392 | 10.5% |
| 4 | 193 | 0.3% |
| 2 | 855 | 1.2% |

**匹配率**（5对相邻摄像头）：

| 摄像头对 | 匹配率 |
|----------|--------|
| TV023 → TV024 | 95.8% |
| TV024 → TV025 | 98.5% |
| TV025 → TV026 | 95.9% |
| TV026 → TV027 | 99.1% |
| TV027 → TV028 | 96.2% |

## 可视化

`visualize.py` 生成 4 面板综合可视化，直观展示拼接质量。

```powershell
python -m trace_data.track_stitch.visualize --sample 2000     # 中文标签
python -m trace_data.track_stitch.visualize --sample 2000 --english  # 英文标签（字体问题兜底）
```

输出：`output/stitching_visualization.png`（498KB，18"×12"）

### 面板 1：时距图（Time-Space Diagram）

横轴=时间（相对秒），纵轴=y 坐标（cm），每条彩色线=一辆车，6 条水平虚线标注各摄像头位置。

| 颜色 | 摄像头 | 位置 |
|:----:|:------:|:----:|
| 🔴 红 | TV023 | K615+703（入口） |
| 🔵 蓝 | TV024 | K615+833 |
| 🟢 绿 | TV025 | K615+963 |
| 🟣 紫 | TV026 | K616+093 |
| 🟠 橙 | TV027 | K616+223 |
| 🟤 棕 | TV028 | K616+323（出口） |
| ⬜ 灰 | 插值点 | 摄像头盲区 |

**好的结果**：线条平滑连续，摄像头过渡处颜色自然衔接，无断裂。

### 面板 2：摄像头覆盖分布

| 覆盖摄像头数 | 轨迹数 | 占比 | 图例颜色 |
|:-----------:|:------:|:----:|:--------:|
| 6（全覆盖） | 35,637 | **50.6%** | 绿色 |
| 5 | 7,392 | 10.5% | 蓝色 |
| 4 | 193 | 0.3% | 蓝色 |
| 3 | 26,320 | 37.4% | 蓝色 |
| 2 | 855 | 1.2% | 红色 |

### 面板 3：质量评分分布

| 指标 | 值 |
|:-----|:---|
| 最高分 | **0.952** |
| 最低分 | 0.497 |
| 均分 | 0.807 |
| 中位数 | 0.840 |

直方图集中在 0.75-0.92 区间，红色虚线=均值，蓝色虚线=中位数。

### 面板 4：精选轨迹放大

Top 5 最高质量轨迹（均覆盖 6 摄像头，实测点占 92%）：

| 轨迹 ID | 评分 | 节点数 | 实测 | 插值 | 穿越时长 | 类型 |
|:--------|:----:|:-----:|:----:|:----:|:--------:|:----:|
| TRAJ_022299 | **0.952** | 131 | 121 | 10 | 327s | truck |
| TRAJ_039769 | 0.951 | 123 | 113 | 10 | 330s | truck |
| TRAJ_039153 | 0.951 | 107 | 97 | 10 | 235s | truck |
| TRAJ_023284 | 0.950 | 103 | 93 | 10 | 257s | truck |
| TRAJ_048074 | 0.947 | 254 | 224 | 30 | 225s | truck |

---

## 车辆类型

支持 7 种车辆类型，拼接结果中实际分布：

| 类型 | 说明 | 轨迹数 | 占比 |
|------|------|:------:|:----:|
| `car` | 小汽车 | 39,548 | **56.2%** |
| `truck` | 卡车 | 30,509 | 43.3% |
| `tanker` | 罐车 | 171 | 0.2% |
| `bus` | 公共汽车 | 112 | 0.2% |
| `pika` | 皮卡 | 30 | <0.1% |
| `motorbike` | 摩托车 | 27 | <0.1% |

> **类型处理**：同一轨迹内不同节点可能被标注为不同类型（如 car→truck→car），采用多数投票（majority vote）确定轨迹代表类型。

## 关键算法

### 坐标与速度

- y 坐标单位：**厘米（cm）**，全局统一坐标系
- 速度单位：原始数据为 **km/h**
- 速度转换：`km/h → cm/ms = speed / 36.0`
- 逆转换：`cm/ms → km/h = speed × 36.0`

### (0,0) 异常点

跟踪丢失或短暂遮挡的标志，约 48% 轨迹包含至少一个 (0,0) 点。处理策略：
- 解析时标记 `is_valid=False`
- 获取首/末节点时自动跳过，取最近的有效节点
- 所有节点均为 (0,0) 的轨迹标记为无效

### 静止轨迹检测

- y 坐标标准差 < 100cm
- 平均速度 < 5km/h
- 满足以上条件标记为静止，不参与跨摄像头匹配
- 典型场景：隧道内停泊车辆

## 技术栈

| 组件 | 用途 |
|------|------|
| Python 3.9+ | 运行环境 |
| NumPy | 代价矩阵向量化计算 |
| scipy.optimize.linear_sum_assignment | 匈牙利算法 |
| argparse | CLI 参数解析 |

> **零 CARLA 依赖**：模块仅依赖 `numpy` 和 `scipy`，可在无 CARLA 环境的机器上运行。

## 相关文档

| 文档 | 路径 |
|------|------|
| 拼接方案报告（v2.1） | `trace_data/track24.7.25-24.8.5_79_6lu/docs/轨迹拼接方案报告.md` |
| 适配方案报告 | `tp_replay/docs/拼接轨迹适配方案报告.md` |
| 回放引擎 README | `tp_replay/README.md` |
| 项目总 README | `README.md` |
