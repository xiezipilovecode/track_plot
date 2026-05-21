## Context

当前 `compute_control()` 使用三段式距离阈值控制（25%/50%/100% 跟车距离），控制量在阈值处跳变，导致跟车振荡。IDM 是学术界最经典的微观交通流模型（Treiber et al., 2000），用连续加速度函数表达驾驶员行为，天然抑制振荡。

现有函数签名：
```python
def compute_control(carla, vehicle, target_wp, target_speed_mps,
                    follow_distance_m, front_gap_m=None, *, ...)
```

缺少前车速度信息，IDM 需要知道前车速度来计算公式中的 `Δv`（速度差）项。

## Goals / Non-Goals

**Goals:**
- 用 IDM 连续加速度函数替代三段式阈值控制
- 消除"急刹→猛追"振荡，产生稳定平滑车流
- 参数具有物理含义，调参后长期稳定不需再改
- 不改动 `compute_control()` 外部调用方的业务逻辑

**Non-Goals:**
- 不改动 GUI、HoloLens 推流、数据集采集、摄像机控制
- 不引入新 Python 依赖
- 不改变车辆蓝图选择或 spawn 策略

## Decisions

### Decision 1：IDM 算法实现为独立模块

新增 `tp_tunnel_traffic/idm.py`，函数签名：

```python
def idm_acceleration(
    v: float,           # 当前速度 (m/s)
    v0: float,          # 目标速度 (m/s)
    s: float,           # 实际车距 (m)
    dv: float,          # 速度差 (本车-前车, m/s)，正值=本车更快，负值=前车更快
    a: float = 2.0,     # 最大加速度 (m/s²)
    b: float = 1.5,     # 舒适减速度 (m/s²)
    s0: float = 2.0,    # 最小停车距离 (m)
    T: float = 1.5,     # 期望时距 (s)
    delta: float = 4.0, # 加速度指数
) -> float:              # 返回加速度 (m/s²)，正值=加速，负值=减速
```

公式：
```
s* = s0 + max(0, v*T + v*dv/(2*sqrt(a*b)))      # 期望车距
a_acc = a * (1 - (v/v0)^delta - (s*/s)^2)         # IDM 加速度
```

**Rationale**：独立模块便于单元测试，也便于后续在 `compute_control()` 或 Traffic Manager 中复用。严格按照 Treiber (2000) 原始论文的参数名和默认值。

### Decision 2：`compute_control()` 签名增加 `front_speed_mps`

```python
def compute_control(
    carla, vehicle, target_wp, target_speed_mps,
    follow_distance_m,
    front_gap_m: float | None = None,
    front_speed_mps: float | None = None,   # ← 新增
    *, ...
)
```

**Rationale**：IDM 需要前车速度信息。`front_gap_m` 已存在，`front_speed_mps` 与之对称。`main.py` 中计算前车速度只需从 `leader.get_velocity()` 提取——已有代码直接可用。

当 `front_gap_m` 为 None 或 `front_speed_mps` 为 None 时，退化为纯速度控制（`idm_acceleration` 中 `s` 不参与约束，相当于无前车场景）。

### Decision 3：IDM 参数通过 config.py 暴露

```python
idm_max_accel: float = 2.0      # TT_IDM_MAX_ACCEL
idm_comfort_decel: float = 1.5  # TT_IDM_COMFORT_DECEL
idm_min_gap: float = 2.0        # TT_IDM_MIN_GAP
idm_time_headway: float = 1.5   # TT_IDM_TIME_HEADWAY
idm_delta: float = 4.0          # TT_IDM_DELTA
```

**Rationale**：`follow_distance_m` 不再需要（IDM 用 `T` 时距替代固定距离）。保留 `follow_distance_m` 用于兼容但标记为 deprecated。

### Decision 4：前端调用改动在 `main.py` 的 `compute_control()` 调用点

当前：
```python
front_gap_m = p_loc.distance(leader.get_location()) if leader else None
```

增加：
```python
front_speed = leader_speed if leader else None
```

`leader_speed` 从 `leader.get_velocity()` 计算（已在 `main.py` 其他位置有类似逻辑）。

## Risks / Trade-offs

- **[Risk]** IDM 参数（`a`, `b`, `s0`, `T`, `delta`）默认值基于论文的高速公路场景，隧道环境可能需要微调
  → **Mitigation**：全部参数通过环境变量覆盖，用户可按需调整；默认值已是广泛验证的值

- **[Risk]** `delta=4.0` 在低速时可能加速度偏小
  → **Mitigation**：符合真实驾驶行为（低速时不需要大加速度）；可在实际使用中通过 `TT_IDM_DELTA` 调节

- **[Risk]** 前端车流密度可能因 IDM 更平滑的减速行为而降低（车辆保持更大间距）
  → **Mitigation**：通过降低 `T`（时距）来增加密度；数据采集质量优先于纯密度指标
