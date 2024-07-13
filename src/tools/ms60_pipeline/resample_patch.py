import numpy as np
import scipy.interpolate as interp
import matplotlib.pyplot as plt
import re
import os
import glob

# 创建resampled文件夹如果它不存在
if not os.path.exists('resampled'):
    os.makedirs('resampled')

# 定义一个函数来处理每个文件
def process_file(file_name):
    # 读取数据
    with open(file_name, 'r') as file:
        lines = file.readlines()

    # 解析数据
    timestamps = []
    positions = []
    for line in lines:
        parts = re.split('\s+', line.strip())
        timestamps.append(float(parts[0]))
        positions.append([float(parts[1]), float(parts[2]), float(parts[3])])

    # 数据转换为numpy数组
    timestamps = np.array(timestamps)
    positions = np.array(positions)

    # 准备重新采样的结果列表
    resampled_timestamps_list = []
    resampled_positions_list = []

    # 寻找大于2秒的间隔并分段处理
    start_idx = 0
    for i in range(1, len(timestamps)):
        if timestamps[i] - timestamps[i - 1] > 1:  # 时间间隔大于2秒
            # 处理当前段
            current_timestamps = timestamps[start_idx:i]
            current_positions = positions[start_idx:i]
            if len(current_timestamps) > 1:  # 需要至少两个点进行插值
                resampled_timestamps, resampled_positions = resample_segment(current_timestamps, current_positions)
                resampled_timestamps_list.append(resampled_timestamps)
                resampled_positions_list.append(resampled_positions)
            start_idx = i

    # 处理最后一段
    current_timestamps = timestamps[start_idx:]
    current_positions = positions[start_idx:]
    if len(current_timestamps) > 1:
        resampled_timestamps, resampled_positions = resample_segment(current_timestamps, current_positions)
        resampled_timestamps_list.append(resampled_timestamps)
        resampled_positions_list.append(resampled_positions)

    # 合并结果
    final_resampled_timestamps = np.concatenate(resampled_timestamps_list)
    final_resampled_positions = np.concatenate(resampled_positions_list, axis=0)

    # 保存重新采样的数据
    resampled_file_name = f'resampled/{os.path.splitext(file_name)[0]}_resampled.txt'
    with open(resampled_file_name, 'w') as file:
        for i, ts in enumerate(final_resampled_timestamps):
            line = f"{ts:.6f} {final_resampled_positions[i, 0]:.6f} {final_resampled_positions[i, 1]:.6f} {final_resampled_positions[i, 2]:.6f} 0 0 0 1\n"
            file.write(line)

    # 可视化并保存结果
    visualize_and_save(timestamps, positions, final_resampled_timestamps, final_resampled_positions, file_name)

# 定义一个函数来重新采样一个数据段
def resample_segment(timestamps, positions):
    # 3次多项式拟合
    fit_x = interp.interp1d(timestamps, positions[:, 0], kind='cubic')
    fit_y = interp.interp1d(timestamps, positions[:, 1], kind='cubic')
    fit_z = interp.interp1d(timestamps, positions[:, 2], kind='cubic')

    # 10Hz重新采样
    resample_rate = 20
    start_time = timestamps[0]
    end_time = timestamps[-1]
    # 确保结束时间符合采样间隔，避免超出原始数据范围
    adjusted_end_time = start_time + ((end_time - start_time) // (1/resample_rate)) * (1/resample_rate)
    resampled_timestamps = np.arange(start_time, adjusted_end_time, 1/resample_rate)

    # 确保所有重新采样的时间戳都在原始时间戳范围内
    resampled_timestamps = resampled_timestamps[resampled_timestamps <= timestamps[-1]]

    # 采样
    resampled_positions_x = fit_x(resampled_timestamps)
    resampled_positions_y = fit_y(resampled_timestamps)
    resampled_positions_z = fit_z(resampled_timestamps)

    resampled_positions = np.vstack((resampled_positions_x, resampled_positions_y, resampled_positions_z)).T

    return resampled_timestamps, resampled_positions

# 定义一个函数来可视化并保存结果
def visualize_and_save(timestamps, positions, resampled_timestamps, resampled_positions, file_name):
    plt.figure(figsize=(18, 5))

    # 分别提取x、y、z轴的重新采样位置数据
    resampled_positions_x = resampled_positions[:, 0]
    resampled_positions_y = resampled_positions[:, 1]
    resampled_positions_z = resampled_positions[:, 2]

    for i, axis in enumerate(['X Axis', 'Y Axis', 'Z Axis']):
        plt.subplot(1, 3, i + 1)
        # 绘制原始数据的连线和散点
        plt.plot(timestamps, positions[:, i], 'r-', alpha=0.5)
        plt.scatter(timestamps, positions[:, i], color='red', s=10)
        # 绘制重新采样数据的连线和散点
        plt.plot(resampled_timestamps, [resampled_positions_x, resampled_positions_y, resampled_positions_z][i], 'b-', alpha=0.5)
        plt.scatter(resampled_timestamps, [resampled_positions_x, resampled_positions_y, resampled_positions_z][i], color='blue', s=10)
        plt.title(axis)
        plt.xlabel('Timestamp')
        plt.ylabel(f'{axis.split()[0]} Position')
        plt.legend(['Original', 'Original Points', 'Resampled', 'Resampled Points'], loc='upper right')

    plt.tight_layout()
    plt.savefig(f'resampled/{os.path.splitext(file_name)[0]}.png')
    plt.close()
 

# 处理当前目录下的所有txt文件
for file_name in glob.glob('*.txt'):
    process_file(file_name)



