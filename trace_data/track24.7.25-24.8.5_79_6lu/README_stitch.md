# 轨迹拼接与时空图示例

本目录下的 `stitch_trajectories.py` 脚本演示如何将多个相机采集到的车辆轨迹数据进行轨迹拼接，并绘制拼接前后的时空图，同时将拼接后的数据保存到单独文件夹。

## 数据格式

每个 txt 文件中按空格分隔，每 6 个字段为一条记录：

1. 时间戳（毫秒）
2. x 坐标
3. y 坐标
4. 瞬时速度（可能为 `null`）
5. 加速度（可能为 `null`）
6. 类型（如 `car`, `truck`, `bus`, `tanker` 等）

文件中可能存在多条记录在同一行的情况，脚本会自动按 6 个字段一组进行解析；同时会过滤明显无效的点，例如 `(x, y) = (0, 0)`。

## 使用方法

1. 安装依赖（仅需一次）：

```bash
pip install pandas matplotlib
```

2. 在本目录下运行脚本（示例 1：只看 `car` 且使用 5 个文件）：

```bash
python stitch_trajectories.py --pattern "K615+703TV023_2024-07-25-*.txt" --max-files 5 --vehicle-type car --output-prefix demo
```

示例 2：使用所有匹配的文件，并拼接所有车型（默认）：

```bash
python stitch_trajectories.py --pattern "K615+703TV023_2024-07-25-*.txt" --output-prefix day1
```

运行完成后，将在本目录下的两个子目录中生成：

- `plots/`：
  - `demo_spacetime_before.png` / `day1_spacetime_before.png`：拼接前的时空图（按文件着色）
  - `demo_spacetime_after.png` / `day1_spacetime_after.png`：拼接后的时空图（按轨迹 ID 着色）
- `stitched_data/`：
  - `demo_stitched_all.csv` / `day1_stitched_all.csv`：包含所有拼接后轨迹点（含 `track_id`）的 CSV 文件

## 拼接思路（简要）

- 将多个 txt 文件读入并按时间排序，统一时间原点（减去最小时间并转换为秒）和空间原点（减去最小 y）。
- 基于时间和纵向位置 y 的连续性，使用**按时间戳分组的一对一多目标关联算法**：
  - 先按时间戳对所有观测排序并分组；
  - 在每个时间步内，对“当前活跃的轨迹”和“该时间步的观测点”构造候选关联，并计算预测纵向位置与观测位置之间的距离代价；
  - 以代价从小到大贪心选择匹配对，一个时间步内每条轨迹最多与一个观测匹配（避免一条轨迹接多个点而导致串车）；
  - 未被匹配到的观测各自新建一条轨迹；时间间隔超过阈值的轨迹会被自动“退休”。
- 得到 `track_id` 后，在时空图中按轨迹 ID 着色，可以看到单辆车在多个相机视野中的连续轨迹效果。

在确认该示例效果后，可以根据需要调节时间/空间阈值（在 `assign_track_ids` 的参数中），并将 `--max-files` 设为 0 或不指定，从而扩展到整天或更长时间范围的数据。