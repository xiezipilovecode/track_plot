# HoloLens 集成到 tp_tunnel_traffic 设计方案

> 版本：Draft 0.1  
> 目标：在现有 GUI 中增加 HoloLens 推流按钮，实时将选中代理车的驾驶员视角画面推送到 HoloLens 2，并接收头部姿态控制相机旋转。

---

## 1. 核心问题

`holens_websocket/opencvdriver.py` 是**独立进程**，自己连 CARLA、自己 spawn 车和相机。集成时必须改为**共享同一个 CARLA 世界和代理车**，不能再单独 spawn。

---

## 2. 架构方案

```
tp_tunnel_traffic 主进程
│
├─ main.py 已有主循环
│   ├─ proxy_states (代理车列表)
│   ├─ capture_vehicle (GUI 选中的代理车)
│   ├─ collector (数据集采集器)
│   └─ video_collector (视频录制器)
│
├─ [新增] hololens_server.py
│   ├─ HoloLensServer 类
│   │   ├─ WebRTC 信令服务 (asyncio, 独立线程)
│   │   ├─ 视频源: 共享 CARLA 世界中的 ego 相机
│   │   └─ 头部旋转 → 相机 transform 更新
│   └─ 与 CARLA 世界复用: 不再独立 spawn
│
├─ gui.py [修改]
│   └─ 新增 "HoloLens Stream" 按钮
│
└─ config.py [修改]
    └─ 新增 TT_HOLOLENS_ENABLE / PORT / RESOLUTION
```

---

## 3. 数据流

```
CARLA 仿真 tick
  │
  ├─→ ego 相机传感器 (已在 collector 或 video_collector 中存在)
  │       │
  │       ├─→ DatasetCollector (帧级采集)
  │       ├─→ VideoCollector (视频录制)
  │       └─→ [新增] HoloLensServer (WebRTC 推流)
  │               │
  │               ├─→ WebRTC VideoTrack → HoloLens 2 显示
  │               └─→ WebRTC DataChannel ← 头部 yaw/pitch
  │                        │
  │                        └─→ 更新 ego 相机 transform
  │
  └─→ proxy 控制循环 (compute_control)
```

**关键：HoloLensServer 不自己 spawn 相机，而是复用已有的 ego 相机传感器。**  
当用户切换代理车时，GUI 调用 `hololens_server.set_vehicle(new_vehicle)`，服务器自动把相机 re-attach 到新目标。

---

## 4. 新增文件与改动

### 4.1 `tp_tunnel_traffic/hololens_server.py`（新文件）

```python
class HoloLensServer:
    """CARLA → HoloLens 2 WebRTC 推流服务器。

    复用主进程的 CARLA 世界和相机传感器，不独立 spawn。
    """

    def __init__(self, carla, world, config, host="0.0.0.0", port=8765):
        self.carla = carla
        self.world = world
        self.config = config
        self.port = port
        self._running = False
        self._vehicle = None
        self._camera_sensor = None      # 复用的 ego 相机
        self._latest_frame_data = None  # bytes
        self._latest_rotation = {"yaw": 0.0, "pitch": 0.0}
        self._track = None              # CarlaVideoTrack 实例
        self._thread = None             # asyncio 事件循环线程

    def start(self, vehicle):
        """启动推流服务。"""
        self._vehicle = vehicle
        self._running = True
        # 启动 asyncio 事件循环线程
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """停止推流。"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)

    def set_vehicle(self, vehicle):
        """切换目标车辆（GUI 选不同代理车时调用）。"""
        self._vehicle = vehicle
        # 相机自动从旧车 detach → attach 到新车

    def feed_frame(self, raw_data: bytes):
        """主循环每 tick 调用，将最新帧数据喂给 WebRTC track。"""
        self._latest_frame_data = raw_data

    def _run_loop(self):
        """后台线程：运行 asyncio 事件循环（WebSocket 信令 + WebRTC）。"""
        ...
```

### 4.2 `config.py`（新增 3 个环境变量）

```python
hololens_enable: bool = _env_bool("TT_HOLOLENS_ENABLE", False)
hololens_port: int = _env_int("TT_HOLOLENS_PORT", 8765)
hololens_resolution: str = _env_str("TT_HOLOLENS_RESOLUTION", "896x504")
```

### 4.3 `gui.py`（新增按钮）

在现有的 "Record Video" 按钮右边增加 **"HoloLens Stream"** 按钮：

```
┌──────────────────────────────────────────────────┐
│  [Ego View] [Overview] [Collect Selected]        │
│  [Record Video] [HoloLens Stream] [Reset Cam]    │
│  [Yaw -] [Reset] [Yaw +]                         │
└──────────────────────────────────────────────────┘
```

`action = "toggle_hololens"`, `group = "hololens"`。

### 4.4 `main.py`（集成调用）

```python
# 初始化
hololens_server = HoloLensServer(carla, world, config) if config.hololens_enable else None

# GUI action
elif gui_action == "toggle_hololens":
    if hololens_enabled:
        hololens_server.stop()
        hololens_enabled = False
    else:
        hololens_server.start(capture_vehicle)
        hololens_enabled = True

# 切换代理车时自动跟随
if hololens_enabled and capture_vehicle:
    hololens_server.set_vehicle(capture_vehicle)

# 主循环 tick：喂帧
if hololens_enabled:
    ego_image = get_ego_camera_frame()  # 从已有的 ego 传感器取
    hololens_server.feed_frame(ego_image.raw_data)
```

---

## 5. 视角一致性保证

| 场景 | 行为 |
|---|---|
| 点击代理车 | HoloLens 画面自动切换到该车驾驶员视角 |
| 点击 HoloLens Stream ON | 启动推流，推流当前选中车的视角 |
| 头部旋转 | HoloLens 发来的 yaw/pitch 控制 ego 相机旋转 |
| 点击 HoloLens Stream OFF | 停止推流，相机恢复默认角度 |

---

## 6. 与现有模块的关系

| 模块 | HoloLensServer 对其影响 |
|---|---|
| `DatasetCollector` | 无影响（不同采集开关） |
| `VideoCollector` | **共享 ego 相机**，如果两者都开，同一帧既录视频又推流 |
| GUI | 新增按钮，状态栏显示 `HoloLens: ON/OFF` |

---

## 7. GUI 运行命令

```bat
set TT_HOLOLENS_ENABLE=1
set TT_HOLOLENS_PORT=8765
set TT_VIDEO_ENABLE=1      ← 可选：同时录制视频
set TT_GUI_ENABLE=1
set TT_PROXY_ENABLE=1
python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

---

## 8. 实现要点

- **不创建新 CARLA 客户端**：HoloLensServer 完全复用 `main.py` 中已有的 `carla`、`world`、传感器
- **asyncio 线程隔离**：WebRTC 的 asyncio 事件循环跑在独立线程，不影响 CARLA 同步 tick
- **帧缓存无锁**：`feed_frame()` 和 `recv()` 之间用简单的 `latest_frame_data` 变量（单写单读，GIL 保护足够）
- **分辨率可配**：通过 `TT_HOLOLENS_RESOLUTION` 调整推流分辨率，平衡画质和延迟
