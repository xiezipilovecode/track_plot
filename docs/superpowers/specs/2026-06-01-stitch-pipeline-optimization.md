# 轨迹拼接管线优化设计

**日期**: 2026-06-01
**状态**: 已审批

## 1. 背景

代码审查发现 17 个问题，其中 2 个 P0 级别缺陷导致大量有效轨迹被丢失。当前方案从 355K 匹配对中仅产出 27,919 条拼接轨迹（92% 丢失率），READEME 中记录的 70K 条全量结果也无法复现。

## 2. P0 — 关键缺陷修复

### 2.1 多起点轨迹组装（`stitcher.py:_assemble_trajectories`）

**当前逻辑**：固定从 `cameras[0]`（TV023）出发组装，丢弃所有从其他摄像头入口进入的车辆。

**新逻辑**：

```
for each camera in config.cameras:
    从该摄像头出发，按现有链追踪算法组装
    收集所有链

所有链去重：
    对每对链 (A, B):
        重叠度 = |A节点时间戳 ∩ B节点时间戳| / max(|A|, |B|)
        if 重叠度 > 0.5:
            保留 quality_score 更高的那条
```

**效果**：6 个摄像头入口全覆盖，不丢失从中间进入的车辆。

### 2.2 y 反向容忍度可配置（`stitcher.py:_check_y_monotonic`）

**当前逻辑**：`_assemble_trajectories` 中硬编码 `max_rev=0`（零容忍），函数签名中有 `max_rev=5` 的默认参数但从未被使用。

**新逻辑**：

```python
# config.py 新增
max_y_reversals: int = 3  # 允许的 y 反向次数

# stitcher.py 调用改为
_check_y_monotonic(stitched, max_rev=self.config.max_y_reversals)
```

**效果**：允许 3 次以内的 y 反向（容忍传感器噪声），大幅减少误杀。355K 匹配对中预期产出显著增加。

## 3. P1 — 重要修复

### 3.1 插值后重新校验单调性（`stitcher.py:_interpolate_gaps`）

**问题**：插值节点在 `next_node.y < curr.y` 时生成非单调 y 值，且从未被捕获。

**修复**：

```python
# _interpolate_gaps() 末尾新增
if not _check_y_monotonic(stitched, max_rev=self.config.max_y_reversals):
    logger.warning(f"Trajectory {stitched.trajectory_id} became non-monotonic after interpolation, discarding")
    return 0  # 丢弃该轨迹
```

### 3.2 质量评分去硬编码（`stitcher.py:_compute_quality`）

**当前**：`completeness = min(1.0, camera_count / 6.0)`、`camera_ratio = min(1.0, camera_count / 3.0)`

**修复**：使用 `len(self.config.cameras)` 替代硬编码的 `6`。`camera_ratio` 与 `completeness` 权重重复，合并为单一 `coverage_score`：

```python
coverage = min(1.0, camera_count / len(self.config.cameras))
quality_score = 0.5 * coverage + 0.3 * continuity + 0.2 * match_conf
```

## 4. P2 — 中优先级修复

### 4.1 CameraDataset.y_min / y_max 修复（`models.py`）

**当前**：只取每个轨迹的 `valid_nodes[0].y` 和 `valid_nodes[-1].y`，不是全局 min/max。

**修复**：遍历所有轨迹的所有 valid_nodes 取真正的最小/最大值。

### 4.2 is_static 修复（`models.py`）

**当前**：`np.std(y_vals)` 使用 `ddof=0`（总体标准差），对小样本轨迹估算偏低。

**修复**：改为 `np.std(y_vals, ddof=1)`。

## 5. 死代码清理

移除以下未使用字段和代码：
- `config.py`：`within_cam_min_overlap_nodes`、`interp_small_gap_ms`、`batch_size`、`output_unmatched`
- `stitcher.py`：`stats["unmatched_fragments"]` 初始化（从未更新）

## 6. 验证增强

### 6.1 `check_traj.py` 重写

**当前**：29 行，硬编码路径，仅查 5 条轨迹。

**重写后**：

| 功能 | 说明 |
|------|------|
| `--input` 参数 | 指定拼接 JSON 路径 |
| `--output` 参数 | 异常轨迹输出路径 |
| 全量扫描 | 遍历所有轨迹 |
| y 反向检测 | 统计 `neg_count`，输出前 N 个反向点 |
| 时间戳反向 | 检测 `ts[i+1] <= ts[i]` |
| 速度异常 | >200 km/h 标记 |
| 插值比过高 | >0.9 标记 |
| 摄像头顺序异常 | 检测非 TV023→TV028 方向的节点序列 |
| 异常轨迹导出 | 有问题的轨迹写 `anomalous_trajectories.json` |

## 7. 不变的部分

| 模块 | 是否改动 |
|------|---------|
| `parser.py` | 不改 |
| `matcher.py` | 不改 |
| `stitcher.py:_interpolate_gaps`（插值算法） | 不改（仅加单调性校验） |
| `stitcher.py:_compute_pair_cost` | 不改 |
| `stitcher.py:_build_chain` | 不改 |
| `models.py:TrajectoryNode` | 不改 |
| `config.py`（除死代码清理外） | 不改核心参数 |

## 8. 数据流

```
data_root/*.txt
    │
    ▼ parser.py（不改）
CameraDataset × 6
    │
    ▼ merge_within_camera（不改）
合并后轨迹 × 565K
    │
    ▼ match_all_pairs（不改）
匹配对 × 355K
    │
    ▼ _assemble_trajectories（改：多起点 + max_rev 可配置）
拼接轨迹 × 预期 > 70K
    │
    ▼ _interpolate_gaps（改：加 monotonic 校验）
带插值轨迹
    │
    ▼ _compute_quality（改：去硬编码）
带评分轨迹
    │
    ▼ export()（不改）
stitched_trajectories.json
    │
    ▼ check_traj.py（重写）
validation_report + anomalous_trajectories.json
```

## 9. 预期效果

| 指标 | 当前 | 优化后预期 |
|------|------|-----------|
| 拼接轨迹数 | 27,919 | > 70,000（恢复被 max_rev=0 误杀的） |
| 覆盖入口点 | TV023 独一 | 6 个摄像头全覆盖 |
| y 反向容忍 | 0（零容忍） | 可配置（默认 3） |
| 插值后校验 | 无 | 有（插值后重新校验单调性） |
| 轨迹质量验证 | 仅 5 条 | 全量扫描 + 自动报告 |

## 10. 已知不纳入

- 匈牙利匹配内存优化（`MAX_BATCH_SIZE=5000` 分配 200MB）— 当前数据集资源足够，后续按需处理
- 摄像头内合并改为匈牙利— 贪婪合并已足够，改动成本高收益低
- 插值节点 x 坐标计算— 当前 `x=0.0` 对回放无影响
