# 视频采集模块设计文档

> 版本：Draft 0.1  
> 目标：在现有帧级数据集采集基础上，新增视频采集模式，面向驾驶人实时仿真与压力参数采集场景。

---

## 1. 背景与动机

当前 `tp_tunnel_traffic` 的采集模式为 **帧级数据集采集**（每 stride 写一帧），适合离线训练。但面向驾驶人实时仿真场景，需要：

- **连续帧记录**：不是每 N tick 采一帧，而是录制完整视频流（或高频连续帧序列）
- **驾驶人视角**：ego 相机作为"驾驶员看到的画面"，未来接入 VR 设备后驾驶人可实时操控车辆
- **压力参数通道**：预留与外部设备（VR 手环/心率带等）的数据接口，在采集视频的同时记录时间戳对齐的压力值

因此新增 **视频采集模式**，作为现有 `DatasetCollector` 的平行能力，互不干扰。

---

## 2. 核心设计原则

1. **不改现有代码**：`DatasetCollector`、`config.py` 中现有的 `TT_COLLECT_*` 变量保持不动
2. **新增独立模块**：`VideoCollector` 类 + 独立的配置项 + 独立输出目录
3. **GUI 共存**：在现有 GUI 中增加 `Record Video` 按钮，与 `Collect Selected` 可独立操作
4. **预留扩展性**：压力参数通过回调/队列注入，不耦合采集逻辑

---

## 3. 目录结构

### 3.1 新增/修改文件

```
tp_tunnel_traffic/
  config.py              ← 新增 TT_VIDEO_* 环境变量
  video_collector.py     ← 新增 VideoCollector 类
  gui.py                 ← 新增 Record Video 按钮
  main.py                ← 新增 VideoCollector 初始化和 tick 调用
  docs/
    VIDEO_COLLECTION_DESIGN.md  ← 本文档
```

### 3.2 输出目录结构

```
dataset_video/
  run_YYYYmmdd_HHMMSS/
    images/               ← 所有帧（PNG），与现有格式完全兼容
      ego/
      front/
      front_left/
      front_right/
      left/
      right/
      rear/
    timestamps.jsonl      ← 帧级时间戳（world_frame + timestamp）
    metadata.json         ← 录制元信息
    driver_stress.jsonl   ← 预留：驾驶人压力参数（可选）
```

> `dataset_video/` 与 `dataset/` 完全独立，互不干扰。

---

## 4. VideoCollector 类设计

### 4.1 核心职责

| 职责 | 说明 |
|---|---|
| 传感器管理 | 复用 `dataset_cameras.json` 中的相机布局，创建 7 个 RGB 相机传感器 |
| 帧缓冲与写入 | 每个传感器回调将帧放入队列，tick 时消费并写 PNG |
| 录制策略 | **连续录制**（不跳帧），每帧都写盘（可配置 stride 降采样） |
| 压力参数记录 | 预留 `stress_callback` 接口，每 tick 调用一次获取当前压力值 |
| 启停控制 | `start_record() / stop_record()`，每次 start 创建新 run 目录 |

### 4.2 伪代码骨架

```python
class VideoCollector:
    def __init__(self, carla, world, config, lane_points):
        self.root_dir = Path(config.video_output_dir)  # dataset_video
        self.sensors = []
        self.frame_queue = Queue(maxsize=3000)
        self.recording = False
        self.run_dir = None
        self._stress_callback = None  # callable or None

    def set_stress_callback(self, callback):
        """注入外部压力参数采集回调。"""
        self._stress_callback = callback

    def start_record(self, vehicle, lane_points):
        """创建 run 目录，生成传感器，开始记录。"""
        timestamp = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = self.root_dir / timestamp
        self.run_dir.mkdir(parents=True)
        self._spawn_sensors(vehicle)
        self._write_metadata(vehicle)
        self.recording = True

    def stop_record(self):
        """停止记录，销毁传感器，写最终统计。"""
        self.recording = False
        self._stop_sensors()

    def on_tick(self, world_frame: int):
        """每 tick 调用：消费队列中的帧并写盘。"""
        if not self.recording:
            return
        self._drain_queue()
        self._write_frame(world_frame)
        if self._stress_callback:
            stress = self._stress_callback()
            self._write_stress(world_frame, stress)
```

### 4.3 与 DatasetCollector 的区别

| 特性 | DatasetCollector | VideoCollector |
|---|---|---|
| 采样策略 | stride 间隔采样 | 连续录制（每帧） |
| 输出格式 | COCO / instance / labels.jsonl | 纯视频帧 + timestamps |
| 目标控制 | proxy-only（采集目标为某辆代理车） | 可选：代理车或将来接入的驾驶员控制车辆 |
| 压力数据 | 无 | 预留 stress_callback |
| 2D 框/分割 | 有（COCO + instance PNG） | 可选（性能权衡） |
| 输出目录 | `dataset/` | `dataset_video/` |

---

## 5. 配置项设计（config.py 新增）

```python
# --- 视频录制 ---
video_enable: bool = _env_bool("TT_VIDEO_ENABLE", False)
video_output_dir: str = _env_str("TT_VIDEO_OUTPUT_DIR", r"dataset_video")
video_frame_stride: int = _env_int("TT_VIDEO_FRAME_STRIDE", 1)     # 1=每帧都录, 2=每2帧
video_cameras_json: str = _env_str("TT_VIDEO_CAMERAS_JSON", r"tp_tunnel_traffic\dataset_cameras.json")
```

> `video_cameras_json` 默认复用现有相机配置，但允许独立指定。

---

## 6. GUI 改动设计（gui.py）

在现有 GUI 中，与 `Collect Selected` 并列增加 **Record Video** 按钮：

```
┌────────────────────────┐
│  Proxies               │
│  ┌──────────────────┐  │
│  │ #1234 @L1 v=42.3 │  │
│  │ #1235 @L2 v=38.1 │  │
│  └──────────────────┘  │
│                         │
│  [Collect Selected]     │  ← 现有
│  [Record Video]         │  ← 新增
│                         │
│  View: [Ego] [Overview] │
└────────────────────────┘
```

### 6.1 按钮行为

| 按钮 | 点击行为 | 状态标识 |
|---|---|---|
| `Record Video` | 切换 ON/OFF | ON 时绿色高亮 |
| `Collect Selected` | 切换 ON/OFF | ON 时绿色高亮 |

两者**互斥**：开启 Record Video 时自动关闭 Collect Selected，反之亦然。避免同一代理车同时被两个采集器写盘。

### 6.2 新增 GUI action

```python
# main.py 中新增 action 处理
elif gui_action == "toggle_video_record":
    video_recording = not video_recording
    if video_recording:
        # 关闭 DatasetCollector
        collector.set_enabled(False)
        collect_proxy_enabled = False
        # 开启 VideoCollector
        run_dir = video_output_path / datetime.now().strftime("run_%Y%m%d_%H%M%S")
        print(f"Record Video: ON, target_proxy={selected_proxy_actor_id}, output={run_dir}")
    else:
        video_collector.stop_record()
        print("Record Video: OFF")
```

---

## 7. 驾驶人压力参数接口设计（预留）

### 7.1 接口定义

```python
class DriverStressProvider(Protocol):
    """外部压力参数提供者协议。"""
    def get_stress(self) -> dict | None:
        """返回当前压力值，或 None（设备未就绪）。"""
        ...
```

### 7.2 数据格式（driver_stress.jsonl）

```json
{"world_frame": 12345, "timestamp": 12.345, "heart_rate": 72, "gsr": 0.45, "pupil_diameter_mm": 3.2}
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `world_frame` | int | CARLA 帧号 |
| `timestamp` | float | 距 run 开始的秒数 |
| `heart_rate` | float | 心率（bpm） |
| `gsr` | float | 皮肤电导（μS） |
| `pupil_diameter_mm` | float | 瞳孔直径（mm） |

> 当前为预留设计，`stress_callback` 接口已就绪，但实际硬件对接在后续阶段完成。

---

## 8. main.py 改动（集成 VideoCollector）

### 8.1 初始化

```python
# 在 main() 中，collector 初始化后新增：
video_collector = VideoCollector(carla, world, config, lane_points) if config.video_enable else None
```

### 8.2 tick 循环

```python
# 在 world.tick() 后新增：
if video_collector is not None:
    video_collector.on_tick(world_frame)

# 现有 collector 保持不变
if collector is not None:
    collector.on_tick(...)
```

### 8.3 cleanup

```python
# finally 块中新增：
if video_collector is not None:
    video_collector.destroy()
```

---

## 9. 用户使用流程

### 9.1 CLI 视频采集

```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla
cd /d E:\code\track_plot

set TT_GUI_ENABLE=1
set TT_PROXY_ENABLE=1
set TT_PROXY_TARGET_PER_LANE=9
set TT_VIDEO_ENABLE=1
set TT_VIDEO_OUTPUT_DIR=dataset_video

python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

然后：选定代理车 → 点击 **Record Video**（ON）→ 驾驶/观察 → 点击 OFF 结束。

### 9.2 将来 VR 模式（预留）

```bat
set TT_VIDEO_DRIVER_MODE=vr
set TT_VIDEO_STRESS_DEVICE_PORT=COM3
```

---

## 10. 实现优先级

| 阶段 | 内容 | 预计文件 |
|---|---|---|
| **Phase 1**（立即） | VideoCollector 核心逻辑、config、GUI 按钮 | `video_collector.py`、`config.py`、`gui.py`、`main.py` |
| Phase 2（后续） | 压力参数接口对接、driver_stress.jsonl | 同上 |
| Phase 3（后续） | VR 头显/手柄接入、驾驶员实时控车 | 新模块 |

---

## 11. 风险与注意事项

- **性能**：连续录制 7 路视频流对磁盘 IO 和 CPU 压力很大，建议使用 SSD 并适当调大 `TT_VIDEO_FRAME_STRIDE`
- **与 DatasetCollector 互斥**：两者不能同时采集同一目标（会创建重复传感器导致帧混乱），GUI 层面已做互斥
- **Disk 空间**：7 相机 × 800×600 × 30fps × 60s ≈ 每分钟约 4GB（PNG），建议提前估算
- **传感器冲突**：VideoCollector 和 DatasetCollector 不能同时 spawn 传感器（会导致同名传感器冲突），通过互斥控制解决
