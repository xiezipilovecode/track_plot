## Why

当前隧道三车道交通流使用三段式距离阈值控制（25%/50%/100%跟车距离），产生"急刹→拉开→猛追→再急刹"的经典交通振荡，HoloLens 端观测和数据集采集效果均不理想。需要用学术界验证的 IDM（Intelligent Driver Model，智能驾驶员模型）替换现有控制逻辑，从根本上消除振荡、产生自然稳定的隧道车流。

## What Changes

- **BREAKING**：替换 `tp_tunnel_traffic/control.py` 中 `compute_control()` 的速度控制与跟车距离逻辑，从三段式阈值改为 IDM 连续函数
- 新增 `tp_tunnel_traffic/idm.py`：IDM 核心算法（加速度计算）
- `tp_tunnel_traffic/config.py`：新增 IDM 参数配置项（最大加速度、舒适减速度、最小车距、期望时距、加速度指数）
- `tp_tunnel_traffic/main.py`：调用侧传入 IDM 所需参数（前车速度差）
- `tp_tunnel_traffic/README.md`：更新控制算法说明与参数文档

## Capabilities

### New Capabilities
- `idm-car-following`: IDM 智能驾驶员模型跟车控制，用连续加速度函数替代三段式阈值，参数具有物理含义（物理上可解释的跟车行为）

### Modified Capabilities
<!-- No existing specs to modify -->

## Impact

- `tp_tunnel_traffic/control.py`：`compute_control()` 函数签名增加 `front_gap_m` 参数（已存在），增加 `front_speed_mps` 参数（新增）；函数内部速度控制逻辑全部替换
- `tp_tunnel_traffic/config.py`：新增 5 个 IDM 参数配置项
- `tp_tunnel_traffic/main.py`：调用 `compute_control()` 时传入前车速度
- `tp_tunnel_traffic/README.md`：更新控制算法文档
- 不改动：GUI、HoloLens 推流、数据集采集、摄像机控制
