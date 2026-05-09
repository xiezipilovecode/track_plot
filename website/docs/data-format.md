# 数据格式

<div class="tat-lead">
TAT 采用受 KITTI 和 nuScenes 启发的简洁、以 run 为粒度的组织方式。所有数据以 <code>world_frame</code> 为主键——一个单一的整数，用于对齐所有传感器的图像、标签、实例掩码和 COCO 标注。
</div>

---

## 1. 目录结构

```text
dataset/
  coco_annotations.json              ← 全局合并 COCO (所有 61 runs)
  batch_summary.json                 ← 批量采集统计
  auto_collect_config.json           ← 续跑状态
  proxy_<actor_id>/
    run_YYYYmmdd_HHMMSS/
      images/
        ego/              (65 PNG, 800×600)
        front/            (65 PNG)
        front_left/       (65 PNG)
        front_right/      (65 PNG)
        left/             (65 PNG)
        right/            (65 PNG)
        rear/             (65 PNG)
        ego_instance/     (65 PNG instance masks)
        front_instance/   (65 PNG)
        ... (7 total instance dirs)
      labels.jsonl                   (65 samples, JSON Lines)
      labels_2d/
        coco_instances.json          (per‑run COCO)
      metadata.json
      target.json
      effective_config.json          (optional)
```

<div class="tat-info-box">
  <div class="box-title">💡 Run 级组织</div>
  每个 <code>run_*</code> 代表来自一台代理车辆的连续约 180 秒的采集片段。每次 run 包含 65 帧，这为序列模型提供了足够的时间上下文，同时保持每次 run 自包含，便于清晰的训练/验证/测试划分。
</div>

---

## 2. 同步

所有数据通过 `world_frame`（CARLA 快照 ID）对齐：

```python
frame = label.get("world_frame", label.get("frame"))
fname = f"{int(frame)}.png"         # → "14253298.png"
path  = f"images/{camera}/{fname}"
```

> `world_frame` 是一个计数器，而非挂钟时间戳。离线时步请使用 `metadata.json → fixed_delta_seconds`。

---

## 3. `labels.jsonl`

JSON Lines 格式——每行一个 JSON 对象（每次 run 共 65 行）。每行记录一个 `world_frame` 下的完整自车状态。

| 字段 | 类型 | 说明 |
|-------|------|-------------|
| `world_frame` | `int` | CARLA 快照帧 ID（主键） |
| `frame` | `int` | 同义词（向后兼容） |
| `actor_id` | `int` | 被采集的代理车辆 |
| `throttle` | `float` | ∈ [0, 1] |
| `steer` | `float` | ∈ [‑1, 1]（负值 = 左转） |
| `brake` | `float` | ∈ [0, 1] |
| `hand_brake` | `bool` | |
| `reverse` | `bool` | |
| `vehicle_x/y/z` | `float` | 世界坐标（米，CARLA 坐标系） |
| `vehicle_yaw` | `float` | 偏航角（度，v2.0 run 中可用） |
| `speed_mps` | `float` | 速度（米/秒，v2.0 run 中可用） |

---

## 4. COCO 2D 检测

### 4.1 检测示例

以下是来自 proxy_1796 的真实 COCO 标注（帧 14253298——共 38 个检测框，分布在 7 个相机中）：

<div class="tat-img-grid split-64">
  <div class="tat-img-figure">
    <img src="/coco-detection-front.png" alt="COCO 2D bboxes on front camera" />
    <div class="caption"><strong>front</strong>——检测到 9 辆车</div>
  </div>
  <div class="tat-img-figure">
    <img src="/coco-detection-front_left.png" alt="COCO 2D bboxes on front_left camera" />
    <div class="caption"><strong>front_left</strong>——检测到 9 辆车</div>
  </div>
</div>

### 4.2 全局合并 COCO（`coco_annotations.json`）⭐

全部 61 次 run 合并为一个 COCO 格式文件——可直接输入任何检测框架：

| 统计项 | 数值 |
|-----------|-------|
| COCO 图像数 | 24,528 |
| COCO 标注数 | 27,763 |
| 类别 | 1（`vehicle`，id=1） |
| 平均标注数/图像 | 1.13 |
| 文件大小 | ~5 MB（压缩 JSON） |

从原始数据集自行生成：

```bat
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset
```

合并后的文件使用全局唯一的 `image_id` 和 `annotation_id` 值。图片路径相对于数据集根目录（`proxy_<id>/run_<ts>/images/<cam>/<frame>.png`）。

### COCO JSON 结构

```json
{
  "images": [
    {"id":1, "file_name":"proxy_1796/.../images/ego/14253298.png",
     "width":800, "height":600, "frame":14253298, "camera":"ego"}
  ],
  "annotations": [
    {"id":1, "image_id":1, "category_id":1,
     "bbox":[285, 220, 95, 80], "area":7600}
  ],
  "categories": [
    {"id":1, "name":"vehicle"}
  ]
}
```

---

## 5. `metadata.json`

Run 级元数据。关键字段：

| 字段 | 类型 | 说明 |
|-------|------|-------------|
| `xodr_path` | `str` | OpenDRIVE 地图路径（QingShiLing.xodr） |
| `cameras` | `list` | 相机安装位姿 + 成像参数 |
| `collect_frame_stride` | `int` | 每隔 N 个模拟器 tick 采样一次 |
| `fixed_delta_seconds` | `float` | 仿真时间步长（同步模式） |
| `sync_mode` | `bool` | 是否启用同步模式？ |
| `target_actor_id` | `int` | 被采集的代理车辆 actor ID |

---

## 6. 局限性

- ❌ 无每帧挂钟时间戳——仅提供 `world_frame` 计数器
- ❌ 无矩阵形式的相机内参/外参——改为提供位姿 + 视场角
- ❌ 仅相机数据——无激光雷达、雷达或深度传感器
- ✅ 全部 24,528 帧均提供完整的 2D 检测框 + 实例掩码

---

## 7. 验证

```bat
conda activate carla
cd /d E:\code\track_plot

# 逐 run 验证
python -m tp_tunnel_traffic.validate_dataset_run ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS

# COCO / 实例验证
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
```
