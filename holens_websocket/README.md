# holens_websocket — CARLA ↔ HoloLens 2 实时视频推流模块

基于 **WebRTC** 实现 CARLA 仿真画面到 HoloLens 2 的低延迟视频推流，同时接收 HoloLens 头部姿态（yaw/pitch）控制 CARLA 相机视角。

## 核心能力

- 将 CARLA 车内摄像头画面通过 WebRTC VideoTrack 实时推流到 HoloLens 2
- 接收 HoloLens 头部旋转并同步驱动 CARLA 相机视角
- 内置抗时钟漂移的 FPS 控制

## 文件说明

| 文件 | 角色 |
|---|---|
| `opencvdriver.py` | 独立服务端（已停用，被 `tp_tunnel_traffic/hololens_server.py` 取代） |
| `hololens_websocket_server.py` | 模拟客户端，本地测试 WebRTC 推流 + 头部旋转 |

## 架构

```
┌──────────────────────────┐       WebRTC (Video + DataChannel)       ┌────────────────┐
│  CARLA PC                 │ ←──────────────────────────────────────→ │  HoloLens 2     │
│  tp_tunnel_traffic        │   WebSocket (Signaling: SDP/ICE)         │                │
│  hololens_server.py       │  → VideoTrack (VP8/H264) 896×504 30fps  │  RTCPeerConn.   │
│                           │  ← DataChannel "controls" yaw/pitch      │  头部追踪       │
└──────────────────────────┘                                          └────────────────┘
```

## 运行

### 前提

```bat
pip install aiortc websockets opencv-python numpy
```

### 1) 启动服务端

```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla
cd /d E:\code\track_plot

set TP_PRESERVE_EXISTING_WORLD=1
set TT_SYNC_MODE=1
set TT_GUI_ENABLE=1
set TT_PROXY_ENABLE=1
set TT_HOLOLENS_ENABLE=1
python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

GUI 中选代理车 → 点 `HoloLens Stream` ON。

### 2) 本地测试客户端

```bat
conda activate carla
cd /d E:\code\track_plot
python -m tp_tunnel_traffic.tests.test_hololens_client
```

### 3) 连接 HoloLens 2

HoloLens C# 客户端连接 `ws://<PC_IP>:8765`，通过 WebRTC 接收视频并回传头部姿态。

## 数据格式

### 视频推流

| 参数 | 值 |
|---|---|
| 编码 | WebRTC VideoTrack (VP8/H264) |
| 分辨率 | 896 × 504 |
| 帧率 | ~30 FPS |
| 相机 | 驾驶员视角 (x=0.65, y=-0.18, z=1.22) |

### 头部姿态

WebRTC DataChannel `"controls"`，8 字节 LE binary：`[yaw: f32] [pitch: f32]`

### 信令

WebSocket JSON：`{"msg":"sdp","type":"offer","sdp":"..."}`

## 关键实现细节

### 全局帧缓冲

服务端相机回调直接写入模块级变量，`VideoStreamTrack.recv()` 读取。**闭包内必须显式声明 `global`**。

```python
_HOLO_FRAME = None  # 模块级

def _cb(image):
    global _HOLO_FRAME  # 必须！
    _HOLO_FRAME = image.raw_data
```

### 客户端创建 DataChannel

**必须在 `createOffer()` 之前创建 DataChannel**，否则 DTLS 握手差异导致 RTP 视频不通。

### FPS 防漂移

每 100 帧检查时间误差，超过 0.5 秒重置基准。

## 已知限制

- 进程内 SDP 回环测试不可用（ICE 需真实网络 socket）
- 后台线程 `ProactorEventLoop` 当前可正常工作
- 局域网 Host candidate，跨公网需 STUN/TURN
