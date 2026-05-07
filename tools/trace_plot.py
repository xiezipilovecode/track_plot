"""轨迹绘图工具 - 使用matplotlib可视化轨迹数据"""

import csv
import os
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


# 配置 - 支持环境变量覆盖
DATA_FILE_PATH = os.getenv(
    "TP_DATA_FILE_PATH",
    r"E:\code\track_plot\trace_data\test_data\data_6lu1.txt"
)
trajectories = []

with open(DATA_FILE_PATH, 'r', encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        parts = line.split()
        trajectory = []

        for i in range(0, len(parts), 6):
            try:
                if i + 5 < len(parts):
                    timestamp = parts[i]
                    x = float(parts[i + 1])
                    y = float(parts[i + 2])
                    speed = float(parts[i + 3])
                    accel = parts[i + 4]
                    veh_type = parts[i + 5]
                    if x != 0.0 or y != 0.0:
                        trajectory.append({
                            'timestamp': timestamp,
                            'x': x,
                            'y': y,
                            'speed': speed,
                            'accel': accel,
                            'type': veh_type
                        })
            except (ValueError, IndexError):
                continue

        if trajectory:
            trajectories.append(trajectory)

print(f"读取到 {len(trajectories)} 条轨迹")

vehicle_type_count = {}
for trajectory in trajectories:
    if trajectory:
        veh_type = trajectory[0]['type']
        vehicle_type_count[veh_type] = vehicle_type_count.get(veh_type, 0) + 1

print("车辆类型统计:")
for veh_type, count in vehicle_type_count.items():
    print(f"  {veh_type}: {count} 辆")

plt.figure(figsize=(14, 12))
colors = list(mcolors.TABLEAU_COLORS.values())
type_legend_added = {}

for idx, trajectory in enumerate(trajectories):
    if len(trajectory) < 2:
        continue

    xs = [point['x'] for point in trajectory]
    ys = [point['y'] for point in trajectory]
    color = colors[idx % len(colors)]
    veh_type = trajectory[0]['type']

    if veh_type not in type_legend_added:
        label = f'{veh_type}: {vehicle_type_count[veh_type]} 辆'
        type_legend_added[veh_type] = True
    else:
        label = None

    plt.plot(xs, ys, '-', color=color, linewidth=1.5, alpha=0.6, label=label)

    for i, point in enumerate(trajectory):
        if i == 0:
            plt.plot(point['x'], point['y'], 'o', color=color, markersize=8,
                    markeredgecolor='green', markeredgewidth=2)
        elif i == len(trajectory) - 1:
            plt.plot(point['x'], point['y'], 's', color=color, markersize=8,
                    markeredgecolor='red', markeredgewidth=2)
        else:
            plt.plot(point['x'], point['y'], 'o', color=color, markersize=4)

        plt.annotate(str(i), (point['x'], point['y']),
                    textcoords="offset points",
                    xytext=(3, 3),
                    fontsize=7,
                    color=color,
                    alpha=0.7)

    print(f"轨迹 {idx + 1}: {len(trajectory)} 个点, 类型: {trajectory[0]['type']}")

plt.title("Vehicle Trajectories with Point Numbers", fontsize=14)
plt.xlabel("X Coordinate", fontsize=12)
plt.ylabel("Y Coordinate", fontsize=12)
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
plt.tight_layout()

clicked_points = []


def on_click(event):
    if event.inaxes is not None:
        x, y = event.xdata, event.ydata
        clicked_points.append((x, y))
        plt.plot(x, y, 'r*', markersize=15, markeredgecolor='black', markeredgewidth=1)
        plt.annotate(f'({x:.2f}, {y:.2f})',
                    (x, y),
                    textcoords="offset points",
                    xytext=(10, 10),
                    fontsize=9,
                    color='red',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
        plt.draw()
        print(f"点击点 {len(clicked_points)}: x={x:.2f}, y={y:.2f}")


fig = plt.gcf()
fig.canvas.mpl_connect('button_press_event', on_click)

print("\n提示: 点击图上任意位置获取坐标，坐标会显示在图上并打印到控制台")
print("关闭窗口结束程序\n")

plt.show()

print("\n所有点击的坐标:")
for i, (x, y) in enumerate(clicked_points, 1):
    print(f"点 {i}: ({x:.2f}, {y:.2f})")

if clicked_points:
    os.makedirs('plot_data', exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_DATA_FILE_PATH = f'plot_data/clicked_points_{timestamp}.csv'
    with open(csv_DATA_FILE_PATH, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['序号', 'X坐标', 'Y坐标'])
        for i, (x, y) in enumerate(clicked_points, 1):
            writer.writerow([i, f'{x:.2f}', f'{y:.2f}'])

    print(f"\n坐标已保存到文件: {csv_DATA_FILE_PATH}")
    print(f"共保存 {len(clicked_points)} 个点")
else:
    print("\n未点击任何坐标，不生成CSV文件")
