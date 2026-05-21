## ADDED Requirements

### Requirement: IDM 加速度计算

系统 SHALL 提供独立的 `idm_acceleration()` 函数，根据 IDM 模型计算车辆加速度。

函数签名：`idm_acceleration(v, v0, s, dv, a, b, s0, T, delta) -> float`

- `v`: 当前速度 (m/s)
- `v0`: 目标速度 (m/s)
- `s`: 实际车距 (m)，None 表示无前车
- `dv`: 速度差 本车-前车 (m/s)，None 等价于 0
- `a`, `b`, `s0`, `T`, `delta`: IDM 参数（均有默认值）

期望车距公式：`s* = max(0, s0 + v*T + v*dv/(2*sqrt(a*b)))`

加速度公式：`a_acc = a * (1 - (v/v0)^delta - (s*/s)^2)`

#### Scenario: 无前车（自由行驶）

- **WHEN** `s` 为 None 或极大值
- **THEN** 返回的加速度 SHALL 仅由速度误差项决定：`a * (1 - (v/v0)^delta)`
- **AND** `s` 为 None 时不计算 `s*/s` 项

#### Scenario: 接近前车（需减速）

- **WHEN** `dv > 0`（本车更快）且 `s < s*`
- **THEN** 返回负加速度（减速）
- **AND** 加速度大小 SHALL 不超过 `b` 参数值

#### Scenario: 前车更快（拉开距离）

- **WHEN** `dv < 0`（前车更快）
- **THEN** 返回的加速度 SHALL 允许本车加速追赶至不超过 `v0`

#### Scenario: 远距离跟车

- **WHEN** `s >> s*`（车距远大于期望车距）
- **THEN** `(s*/s)^2` 项趋近于 0，加速度趋近于自由行驶

### Requirement: compute_control 集成 IDM

`compute_control()` SHALL 接受 `front_speed_mps` 参数，当 `front_gap_m` 和 `front_speed_mps` 均非 None 时调用 `idm_acceleration()` 计算加速度，并将 IDM 加速度转换为 CARLA throttle/brake 控制量。

#### Scenario: 有前车时使用 IDM

- **WHEN** `front_gap_m` 和 `front_speed_mps` 均非 None
- **THEN** throttle/brake SHALL 由 `idm_acceleration()` 返回值决定
- **AND** 正加速度 → `throttle = clamp(accel / a_max, 0, 0.8)`
- **AND** 负加速度 → `brake = clamp(-accel / b, 0, 1.0)`

#### Scenario: 无前车时使用纯速度控制

- **WHEN** `front_gap_m` 或 `front_speed_mps` 为 None
- **THEN** throttle SHALL 由速度误差比例控制：`throttle = clamp((v0 - v) / v0, 0.0, 0.55)`

### Requirement: IDM 参数可配置

IDM 参数 SHALL 通过 `TunnelTrafficConfig` 环境变量配置，KEY 前缀为 `TT_IDM_`。

| 变量 | 默认值 | 物理含义 |
|---|---|---|
| `TT_IDM_MAX_ACCEL` | 2.0 | 最大加速度 (m/s²) |
| `TT_IDM_COMFORT_DECEL` | 1.5 | 舒适减速度 (m/s²) |
| `TT_IDM_MIN_GAP` | 2.0 | 最小停车距离 (m) |
| `TT_IDM_TIME_HEADWAY` | 1.5 | 期望时距 (s) |
| `TT_IDM_DELTA` | 4.0 | 加速度指数 |

#### Scenario: 默认参数可用

- **WHEN** 用户未设置任何 `TT_IDM_*` 环境变量
- **THEN** 系统 SHALL 使用 Treiber (2000) 论文推荐的默认值运行

#### Scenario: 自定义参数

- **WHEN** 用户设置 `TT_IDM_TIME_HEADWAY=2.0`
- **THEN** `TunnelTrafficConfig.idm_time_headway` SHALL 为 2.0
