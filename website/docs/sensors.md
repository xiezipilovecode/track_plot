
# 传感器

<div class="tat-lead">
TAT 数据集每辆车配备 <strong>7 路同步 RGB 摄像头</strong>，提供隧道环境的全方位环绕覆盖。每帧摄像头图像都有对应的 <strong>像素级实例分割掩码</strong>——共计 24,528 张掩码，100% 配对。
</div>

---

## 多视角示例（7 路摄像头）

同一 `world_frame` 下的全部 7 路同步摄像头（第 14253598 帧，proxy_1796，run_20260506_163611）：

<div class="tat-img-full">
  <img src="/multi-view-grid.png" alt="同一帧下的全部 7 路摄像头" />
  <div class="caption">全部 7 路同步摄像头——ego、front、front_left、front_right、left、right、rear（第 14253598 帧）</div>
</div>

<div class="tat-narrative">
多视角配置提供了全面的空间感知能力：<strong>前向摄像头</strong>（front、front‑left、front‑right）捕捉前方道路及相邻车道；<strong>侧面摄像头</strong>（left、right）监控侧方交通；<strong>后向摄像头</strong>（rear）追踪后方车辆；而 <strong>ego（鸟瞰）摄像头</strong>则提供俯视视角，可用于定位与车道保持分析。
</div>

---

## RGB 与实例分割

每帧 RGB 图像都有对应的像素级实例掩码。以下是三个摄像头视角及其实例掩码——每种颜色代表一个不同的车辆实例。

<div class="tat-img-full">
  <img src="/instance-comparison.png" alt="RGB 与实例分割对比" />
  <div class="caption">上行：RGB 摄像头视角。下行：实例分割掩码（每种颜色 = 一个车辆实例）。全部来自第 14253598 帧。</div>
</div>

<div class="tat-highlight">
  <strong>100% 覆盖：</strong>全部 24,528 张 RGB 图像均有匹配的实例分割掩码——无缺失帧，无部分覆盖。这使得 TAT 数据集可直接用于监督式实例分割训练。
</div>

---

## 摄像头组

| 摄像头 | 目录 | 朝向 | 帧数（总计） | 主要用途 |
|--------|-----------|-------------|:---:|-------|
| Ego（俯视） | `ego` | 俯视 / 鸟瞰 | 3,504 | 定位、车道保持可视化 |
| Front | `front` | 前方 | 3,504 | 主驾驶视角 |
| Front‑Left | `front_left` | 前方左侧（约 45°） | 3,504 | 左侧车道与盲区覆盖 |
| Front‑Right | `front_right` | 前方右侧（约 45°） | 3,504 | 右侧车道与盲区覆盖 |
| Left | `left` | 左侧（约 90°） | 3,504 | 侧方感知 |
| Right | `right` | 右侧（约 90°） | 3,504 | 侧方感知 |
| Rear | `rear` | 后方 | 3,504 | 后方交通监控 |

**总计**：7 路摄像头 × 3,504 帧 = **24,528 张 RGB 图像 + 24,528 张实例掩码**（61 趟 × 65 帧/趟）。

---

## 摄像头属性（来自 `metadata.json`）

`metadata.json → cameras[]` 中的每个摄像头条目提供了挂载位姿与成像参数：

| 字段 | 类型 | 含义 |
|-------|------|---------|
| `name` | `str` | 摄像头名称（对应图像子目录） |
| `x`、`y`、`z` | `float` | 相对于车辆本体的挂载位置（米） |
| `pitch` | `float` | 俯仰角（度） |
| `yaw` | `float` | 偏航角（度） |
| `roll` | `float` | 翻滚角（度） |
| `width` | `int` | 图像宽度（px）——800 |
| `height` | `int` | 图像高度（px）——600 |
| `fov` | `float` | 水平视场角（度） |

> 对于需要矩阵形式内参的几何或传感器融合任务，可按以下公式近似 `K`（针孔模型）：

```
fx = width  / (2 * tan(fov / 2))
fy = height / (2 * tan(fov / 2))
cx = width  / 2
cy = height / 2
```

---

## 实例分割

每趟运行包含 7 个 `*_instance/` 目录——每个 RGB 摄像头对应一个——其中包含 PNG 掩码，每个像素值编码了一个唯一的车辆实例 ID。

| 属性 | 值 |
|-----------|-------|
| 格式 | PNG（800×600，24 位 RGB） |
| 编码方式 | 逐像素实例 ID（按 COCO 映射着色） |
| 覆盖范围 | 24,528 张掩码——覆盖 100% 的 RGB 帧 |
| 生成开关 | `TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1` |
| 后缀约定 | `TT_COLLECT_INSTANCE_SUFFIX=_instance` |

<div class="tat-info-box">
  <div class="box-title">🔍 用 Python 读取实例掩码</div>

```python
from PIL import Image
import numpy as np

mask = np.array(Image.open("images/front_instance/14253598.png"))
unique_ids = np.unique(mask)
print(f"Vehicles in frame: {len(unique_ids) - 1}")  # subtract background (0)
```

颜色到实例 ID 的映射定义在 COCO 标注中——`coco_instances.json` 中的每个 `annotation` 将实例与其边界框关联。
</div>
