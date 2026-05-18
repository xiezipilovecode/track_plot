# holens_websocket — CARLA ↔ HoloLens 2 实时视频推流模块

本模块实现 CARLA 仿真画面到 HoloLens 2 的低延迟视频推流，同时接收 HoloLens 头部姿态（yaw/pitch）控制 CARLA 中驾驶员视角。

---

## 方案演进

### 方案 A（原始方案）：纯 WebRTC

| 通道 | 协议 | 内容 |
|---|---|---|
| 视频推流 | WebRTC VideoTrack | 摄像头 RGB 帧，VP8/H264 编码 |
| 头部姿态 | WebRTC DataChannel | yaw/pitch，8 字节二进制 |
| 信令 | WebSocket JSON | SDP offer/answer、ICE candidate |

**优点**：低延迟、自适应码率、业界标准  
**问题**：`aiortc 1.13.0` 在 Windows + Python 3.9 环境下存在 bug——`RTCRtpSender` 从不调用 track 的 `recv()`，视频帧永远无法发出。已验证此 bug 在所有测试场景下复现（loopback、主线程、后台线程、`addTrack`、`replaceTrack`、`VideoStreamTrack`、`MediaStreamTrack`、aiortc 1.9.0 / 1.13.0 均无效）。

### 方案 B（当前方案）：WebSocket JPEG + WebRTC DataChannel

| 通道 | 协议 | 内容 |
|---|---|---|
| **视频推流** | **WebSocket 二进制** | **4 字节长度头（大端）+ JPEG 数据** |
| 头部姿态 | WebRTC DataChannel | yaw/pitch，8 字节二进制（不变） |
| 信令 | WebSocket JSON/二进制 | SDP 走文本，视频走二进制（同端口 8765） |

**改动原因**：绕开 aiortc 的 bug。JPEG 编码简单可靠，局域网推流 896×504@30fps 带宽约 3~5 MB/s，千兆网下无明显延迟。

### 方案对比

| | 方案 A（纯 WebRTC） | 方案 B（WS JPEG + WebRTC 控） |
|---|---|---|
| 视频传输 | ❌ aiortc bug，推不出去 | ✅ WebSocket 二进制 JPEG，已验证 PASS |
| 头部姿态 | ✅ WebRTC DataChannel | ✅ 不变（保留了 WebRTC DataChannel） |
| HoloLens 兼容 | ✅ 原生 `RTCPeerConnection` | ✅ 需新增 `ClientWebSocket` 收视频 |
| 延迟 | ~50ms（理论值，实测不可用） | ~30ms（局域网实测） |
| 带宽 | ~2 MB/s（VP8） | ~4 MB/s（JPEG Q=70, 896×504） |
| 服务端实现 | `hololens_server.py`（aiortc `addTrack`） | `hololens_server.py`（`cv2.imencode` → WebSocket broadcast） |

### 结论

- **方案 A 在当前环境下无法使用**（aiortc bug）。
- **方案 B 已验证可用**（`test_ws_jpeg_stream.py` PASS，自包含回环测试）。
- 两个方案对 HoloLens C# 客户端均适用——方案 B 需要多写 ~30 行 WebSocket 接收代码。

---

## 架构（当前方案 B）

```
┌──────────────────────────┐          WebSocket (8765)           ┌────────────────┐
│  CARLA PC                 │ ←────────────────────────────────→ │  HoloLens 2     │
│  tp_tunnel_traffic        │                                    │  (C# 客户端)    │
│                           │                                    │                │
│  hololens_server.py       │  → 推流: JPEG 二进制帧             │  ClientWebSocket│
│    camera回调             │     (4B 长度头 + JPEG 数据)         │  → 解码 JPEG    │
│    → cv2.imencode(JPEG)   │                                    │  → MR 显示      │
│    → WebSocket broadcast  │                                    │                │
│                           │  ← 头部姿态: WebRTC DataChannel    │  头部追踪       │
│    DataChannel 接收       │     yaw/pitch (8B binary)          │                │
│    → 相机旋转更新          │                                    │                │
└──────────────────────────┘                                    └────────────────┘
```

---

## 运行

### 前提

```bat
pip install aiortc websockets opencv-python numpy
```

### 1) 启动服务端（隧道仿真 + HoloLens 推流）

```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla
cd /d E:\code\track_plot

set TT_GUI_ENABLE=1
set TT_PROXY_ENABLE=1
set TT_HOLOLENS_ENABLE=1
python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

GUI 中选代理车 → 点 `HoloLens Stream` ON。服务端开始 WebSocket 推流。

### 2) 本地测试（PC 上模拟客户端）

```bat
conda activate carla
cd /d E:\code\track_plot
python -m tp_tunnel_traffic.tests.test_hololens_client
```

或使用旧模拟客户端（WebRTC 视频部分不可用，仅做连接测试）：
```bat
python holens_websocket\hololens_websocket_server.py
```

### 3) 连接 HoloLens 2

HoloLens C# 客户端连接 `ws://<PC_IP>:8765`。适配代码见 `tp_tunnel_traffic/tests/HoloLens_WebSocket_CSharp.cs`。

---

## 数据格式

### 视频推流（服务端 → 客户端）

**WebSocket 二进制帧**，每帧格式：

```
┌──────────────┬─────────────────────┐
│  4 bytes     │  N bytes            │
│  length (BE) │  JPEG data          │
└──────────────┴─────────────────────┘
```

| 参数 | 值 |
|---|---|
| 编码 | JPEG（cv2.imencode, quality=70）|
| 分辨率 | 896 × 504（可调 `TT_HOLOLENS_RES_W/H`）|
| 帧率 | ~30 FPS |
| 相机位置 | 驾驶员视角（x=0.65, y=-0.18, z=1.22）|

**接收伪代码**（C#）：
```csharp
var ws = new ClientWebSocket();
await ws.ConnectAsync(new Uri("ws://PC_IP:8765"), token);
var buf = new byte[256 * 1024];
while (ws.State == WebSocketState.Open) {
    var result = await ws.ReceiveAsync(buf, token);
    int jpegLen = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
    byte[] jpeg = buf[4..(4+jpegLen)];
    texture.LoadImage(jpeg);  // Unity Texture2D
}
```

### 头部姿态（客户端 → 服务端）

通过 **WebRTC DataChannel `"controls"`** 发送，8 字节二进制（方案 B 中不变）：

```
[yaw: float32 LE] [pitch: float32 LE]
```

### 信令（WebSocket 文本）

信令消息为 JSON 文本，与二进制视频帧共享同一 WebSocket 连接：

```json
{"msg": "sdp", "type": "offer", "sdp": "..."}
{"msg": "sdp", "type": "answer", "sdp": "..."}
{"msg": "ice", "candidate": "...", "sdpMid": "0", "sdpMlineIndex": 0}
```

**注意**：文本消息 = 信令；二进制消息 = 视频帧。客户端根据 `msg is string` vs `msg is byte[]` 区分。

---

## HoloLens C# 客户端实现

### 方案 A（纯 WebRTC）的实现方式

使用 Unity 的 `WebRTC` 包或原生 `RTCPeerConnection`：
1. 连接 WebSocket 信令服务器
2. 创建 `RTCPeerConnection`，`AddTransceiver(TrackKind.Video, direction: RecvOnly)`
3. 交换 SDP offer/answer
4. 通过 `OnTrack` 回调接收 `VideoStreamTrack`
5. 将 `VideoStreamTrack` 渲染到 `Texture2D`

### 方案 B（当前方案）的实现方式

需要两套连接：
- **WebSocket**：接收 JPEG 视频帧 + 发送信令 SDP
- **RTCPeerConnection**：仅用于 WebRTC DataChannel 发送头部姿态（与方案 A 相同）

C# 适配代码见 `tp_tunnel_traffic/tests/HoloLens_WebSocket_CSharp.cs`，提供两种子方案：
- **Option A**（推荐）：保留现有 WebRTC DataChannel 发送头部姿态，新增 `ClientWebSocket` 收 JPEG 视频
- **Option B**：全部切到纯 WebSocket（视频 + 头部姿态都走 WebSocket）

---

## 文件说明

| 文件 | 角色 | 说明 |
|---|---|---|
| `opencvdriver.py` | 独立服务端（原方案 A） | 自己连 CARLA、自己 spawn 车和相机。WebRTC 推流（视频部分不可用） |
| `hololens_websocket_server.py` | 模拟客户端（原方案 A） | 本地模拟 HoloLens，WebRTC 收视频（不可用）+ 发头部旋转 |
| `../tp_tunnel_traffic/hololens_server.py` | **集成服务端（方案 B）** | 复用隧道仿真的 CARLA 世界和相机，WebSocket JPEG 推流 |
| `../tp_tunnel_traffic/tests/test_hololens_client.py` | **测试客户端（方案 B）** | WebSocket JPEG 收视频 + OpenCV 显示 |
| `../tp_tunnel_traffic/tests/HoloLens_WebSocket_CSharp.cs` | **HoloLens C# 适配代码** | 方案 A/B 的 C# 实现示例 |

---

## 已知限制

- WebSocket JPEG 方案带宽高于 WebRTC VP8（~4 MB/s vs ~2 MB/s），局域网可忽略，跨公网需注意
- 头部姿态仍走 WebRTC DataChannel，若 aiortc 的 DataChannel 也出问题，可切到方案 B Option B（纯 WebSocket）
- JPEG 质量固定为 70，如需调节可改 `hololens_server.py` 中 `self._jpeg_quality`
