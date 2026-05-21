## 1. 新增 IDM 核心模块

- [x] 1.1 创建 `tp_tunnel_traffic/idm.py`，实现 `idm_acceleration()` 函数
- [x] 1.2 实现期望车距计算 `s* = s0 + v*T + v*dv/(2*sqrt(a*b))`
- [x] 1.3 实现加速度公式 `a * (1 - (v/v0)^delta - (s*/s)^2)`，无前车时跳过 s 项

## 2. 集成到 compute_control

- [x] 2.1 `compute_control()` 新增 `front_speed_mps` 参数
- [x] 2.2 有前车时调用 `idm_acceleration()`，将加速度转换为 throttle/brake
- [x] 2.3 无前车时保持纯速度控制（兼容旧行为）

## 3. 配置与调用方改动

- [x] 3.1 `config.py` 新增 5 个 `TT_IDM_*` 环境变量
- [x] 3.2 `main.py` 调用 `compute_control()` 时传入前车速度
- [x] 3.3 移除 `config.py` 中不再需要的 `proxy_follow_distance_m` 相关注释

## 4. 编译与测试

- [x] 4.1 `compileall tp_tunnel_traffic` 通过
- [x] 4.2 实测隧道车流稳定无振荡：`python -m tp_tunnel_traffic.tests.test_tunnel_autodrive`

## 5. 转向优化（追加）

- [x] 5.1 `steer_lpf_alpha` 从 0.12 → 0.35：消除因相位滞后导致的车辆左右摆动
