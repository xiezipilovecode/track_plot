# 开发工具包

一套轻量级工具，用于读取、验证和可视化 TAT 数据集。

---

## 1. 离线验证

```bat
conda activate carla
cd /d E:\code\track_plot

# 单次采集验证（图像 ↔ 标注对齐）
python -m tp_tunnel_traffic.validate_dataset_run ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS

# COCO / 实例输出验证
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS

# 导出检查样本
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS ^
  --sample-cameras ego,front ^
  --sample-limit 3
```

---

## 2. 合并全局 COCO

将全部 61 次采集合并为一个 `coco_annotations.json`，可直接用于检测框架：

```bat
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset
```

输出：`dataset/coco_annotations.json` — 24,528 张图像，27,763 条标注。

---

## 3. 读取数据（Python）

### 3.1 加载全局 COCO

```python
import json

coco = json.load(open("dataset/coco_annotations.json"))
print(f"Images: {len(coco['images'])}, Annotations: {len(coco['annotations'])}")
# Images: 24528, Annotations: 27763
```

### 3.2 加载 `labels.jsonl`

```python
import json
from pathlib import Path

def load_labels(run_dir: Path) -> list[dict]:
    labels = []
    with open(run_dir / "labels.jsonl") as f:
        for line in f:
            if line.strip():
                labels.append(json.loads(line))
    return labels

run = Path("dataset/proxy_1796/run_20260506_163611")
labels = load_labels(run)
print(f"{len(labels)} samples")  # 65
```

### 3.3 对齐图像与标注

```python
def get_image_path(run_dir: Path, camera: str, world_frame: int) -> Path:
    return run_dir / "images" / camera / f"{int(world_frame)}.png"

path = get_image_path(run, "front", labels[0]["world_frame"])
print(path.exists())  # True
```

### 3.4 加载实例掩码

```python
from PIL import Image

mask_path = run / "images" / "front_instance" / f"{labels[0]['world_frame']}.png"
mask = Image.open(mask_path)
# 每个像素值 = 唯一的车辆实例 ID
```

---

## 4. 枚举全部采集

```python
def discover_runs(dataset_root: Path) -> list[Path]:
    runs = []
    for proxy_dir in sorted(dataset_root.glob("proxy_*")):
        for run_dir in sorted(proxy_dir.glob("run_*")):
            runs.append(run_dir)
    return runs

runs = discover_runs(Path("dataset"))
print(f"Found {len(runs)} runs")  # 61
```

---

## 5. 自动采集

v2.0 数据集分 6 批采集，覆盖不同的交通密度和速度。

| 环境变量 | 默认值 | 说明 |
|-------------|---------|-------------|
| `TT_AUTO_COLLECT_ENABLE` | `0` | 启用自动采集 |
| `TT_AUTO_COLLECT_SECONDS_PER_RUN` | `180` | 每辆车采集时长（秒） |
| `TT_AUTO_COLLECT_MAX_RUNS` | `61` | 最大采集次数 |
| `TT_AUTO_COLLECT_MODE` | `balanced` | balanced / cycle / random |
| `TT_AUTO_COLLECT_MIN_SPEED_MPS` | `5.0` | 最低速度阈值 |
| `TT_AUTO_COLLECT_COOLDOWN_S` | `5` | 采集间隔冷却 |

启动：

```bat
python -m tp_tunnel_traffic.tests.test_auto_collect
```

输出：`dataset/batch_summary.json`、`dataset/auto_collect_config.json`。

---

## 6. 环境变量参考

| 变量名 | 默认值 | 用途 |
|----------|---------|---------|
| `TT_COLLECT_ENABLE` | — | 启用数据集采集 |
| `TT_COLLECT_FRAME_STRIDE` | `10` | 每隔 N tick 写入一次 |
| `TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION` | — | 生成实例掩码 PNG |
| `TT_COLLECT_WRITE_COCO` | — | 生成 COCO 2D JSON |
| `TT_COLLECT_INSTANCE_SUFFIX` | `_instance` | 实例相机后缀 |
| `TT_COLLECT_COCO_MIN_AREA_PX2` | `200` | 最小边界框面积 |
| `TT_COLLECT_COCO_MAX_HEIGHT_RATIO` | `0.9` | 最大边界框高度比例 |
