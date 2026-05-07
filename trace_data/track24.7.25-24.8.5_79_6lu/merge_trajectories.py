import os
import pandas as pd
import glob

# 设置工作目录
work_dir = r'f:\BaiduNetdiskDownload\datasets\track24.7.25-24.8.5\track24.7.25-24.8.5_79_6lu'
os.chdir(work_dir)

# 获取所有轨迹文件
files = glob.glob('*.txt')
files.sort()  # 按文件名排序，确保时间顺序

print(f'找到 {len(files)} 个轨迹文件')

# 存储所有车辆轨迹的数据结构
vehicle_trajectories = {}

# 处理每个文件
for file_idx, file_path in enumerate(files):
    print(f'处理文件 {file_idx+1}/{len(files)}: {file_path}')
    
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    for line in lines:
        line = line.strip()
        if not line:
            continue
        
        # 解析每行数据
        parts = line.split()
        if len(parts) < 2:
            continue
        
        # 第一部分是车辆ID
        vehicle_id = parts[0].strip('→')
        
        # 后续部分是轨迹点数据
        trajectory_data = parts[1:]
        
        # 每个轨迹点包含：时间戳、x坐标、y坐标、瞬时速度、加速度、类型
        # 每6个数据为一个轨迹点
        for i in range(0, len(trajectory_data), 6):
            if i + 5 < len(trajectory_data):
                timestamp = int(trajectory_data[i])
                x = float(trajectory_data[i+1])
                y = float(trajectory_data[i+2])
                speed = float(trajectory_data[i+3])
                acceleration = float(trajectory_data[i+4]) if trajectory_data[i+4] != 'null' else None
                vehicle_type = trajectory_data[i+5]
                
                # 检查数据有效性
                if x == 0 and y == 0 and speed == 0:
                    continue  # 跳过无效数据
                
                # 为车辆创建轨迹列表
                if vehicle_id not in vehicle_trajectories:
                    vehicle_trajectories[vehicle_id] = []
                
                # 添加轨迹点
                vehicle_trajectories[vehicle_id].append({
                    'timestamp': timestamp,
                    'x': x,
                    'y': y,
                    'speed': speed,
                    'acceleration': acceleration,
                    'type': vehicle_type,
                    'file': file_path
                })

print(f'共解析到 {len(vehicle_trajectories)} 辆车的轨迹数据')

# 对每辆车的轨迹按时间戳排序
for vehicle_id, trajectory in vehicle_trajectories.items():
    trajectory.sort(key=lambda x: x['timestamp'])

# 保存完整轨迹数据
output_dir = 'merged_trajectories'
os.makedirs(output_dir, exist_ok=True)

# 保存为CSV文件
all_trajectories = []
for vehicle_id, trajectory in vehicle_trajectories.items():
    for point in trajectory:
        all_trajectories.append({
            'vehicle_id': vehicle_id,
            'timestamp': point['timestamp'],
            'x': point['x'],
            'y': point['y'],
            'speed': point['speed'],
            'acceleration': point['acceleration'],
            'type': point['type'],
            'file': point['file']
        })

# 创建DataFrame并保存
df = pd.DataFrame(all_trajectories)
df = df.sort_values(['vehicle_id', 'timestamp'])
output_file = os.path.join(output_dir, 'complete_trajectories.csv')
df.to_csv(output_file, index=False, encoding='utf-8-sig')

print(f'完整轨迹数据已保存到: {output_file}')
print(f'总轨迹点数量: {len(df)}')
print(f'轨迹数据预览:')
print(df.head())

# 统计信息
print('\n轨迹统计信息:')
print(f'车辆数量: {df["vehicle_id"].nunique()}')
print(f'车辆类型分布:')
print(df["type"].value_counts())
