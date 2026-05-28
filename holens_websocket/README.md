# holens_websocket — CARLA ↔ HoloLens 2 实时视频推流模块

基于 **WebRTC** 实现 CARLA 仿真画面到 HoloLens 2 的低延迟视频推流，同时接收 HoloLens 头部姿态（yaw/pitch）控制 CARLA 相机视角。

## 核心能力

- CARLA 驾驶员视角通过 WebRTC VideoTrack 实时推流到 HoloLens 2
- HoloLens 头部旋转（yaw/pitch）驱动 CARLA 相机，EMA 平滑
- 沉浸式驾驶效果：HUD 仪表盘、路面振动、隧道灯节律
- 抗时钟漂移 FPS 控制 + 防抖帧缓存

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
│  hololens_server.py       │  → VideoTrack (VP8/H264) 1280×720 30fps │  RTCPeerConn.   │
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
python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

GUI 中选代理车 → 点 `HoloLens Stream` ON。

### 2) 本地测试客户端

```bat
conda activate carla
cd /d E:\code\track_plot
python -m tp_tunnel_traffic.tests.test_hololens_client
```

### 3) HoloLens 2 连接

HoloLens C# 客户端连接 `ws://<PC_IP>:8765`，通过 WebRTC 接收视频并回传头部姿态。

## 数据格式

### 视频推流（1280×720 @ 30fps）

WebRTC VideoTrack，VP8/H264 编码，BGRA 原始帧 → 沉浸后处理 → RGB → VideoFrame。

### 头部姿态

WebRTC DataChannel `"controls"`，8 字节 LE binary：`[yaw: f32] [pitch: f32]`

### 信令

WebSocket JSON：`{"msg":"sdp","type":"offer","sdp":"..."}` / `{"msg":"ice",...}`

## 沉浸效果配置

| 环境变量 | 默认 | 说明 |
|---|---|---|
| `TT_HOLOLENS_IMMERSIVE` | 1 | 总开关 |
| `TT_HOLOLENS_IMM_HUD` | 1 | HUD 仪表盘 |
| `TT_HOLOLENS_IMM_VIBRATION` | 1 | 路面振动 |
| `TT_HOLOLENS_IMM_LIGHT` | 1 | 隧道灯节律 |

## 已知限制

- 进程内 SDP 回环测试不可用（ICE 需真实网络 socket）
- 后台线程 `ProactorEventLoop` 可正常工作
- 局域网 Host candidate，跨公网需 STUN/TURN
