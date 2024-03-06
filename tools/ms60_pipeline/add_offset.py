import os

# 假设的offsets字典，键为原始文件名（不带_resampled），值为对应的时间偏移量
offsets = {
    'legged_room00': 3.01,
    'legged_grass00': 3.37,
    'legged_grass01': 3.49,
    'legged_transition00': 3.36,
    'legged_tunnel00': 3.31,
    'handheld_grass00': -0.23,
    'handheld_room00': -0.72,
    'handheld_room01': -0.73,
    'handheld_starbucks00': -0.64,
    'handheld_starbucks01': -0.67,
    'handheld_tunnel00': 3.2,
    'ugv_transition00': 0.02,
    'ugv_transition01': 0.06
}

# 确保correlated文件夹存在
if not os.path.exists('correlated'):
    os.makedirs('correlated')

# 遍历resampled文件夹下的所有_resampled.txt文件
for filename in os.listdir('resampled'):
    if filename.endswith('_resampled.txt'):
        original_filename = filename.replace('_resampled.txt', '')
        
        # 检查文件是否在offsets字典中
        if original_filename in offsets:
            # 读取原始文件内容
            with open(os.path.join('resampled', filename), 'r') as file:
                lines = file.readlines()

            # 构造新文件名，将后缀改为_correlated.txt
            new_filename = original_filename + '_correlated.txt'

            # 调整时间戳并写入新文件
            with open(os.path.join('correlated', new_filename), 'w') as file:
                for line in lines:
                    parts = line.split()
                    if len(parts) > 0:  # 确保行非空
                        # 调整时间戳
                        parts[0] = str(float(parts[0]) - offsets[original_filename])
                        # 写入新行
                        file.write(' '.join(parts) + '\n')
            print(f"Processed {filename} with offset {offsets[original_filename]}, saved as {new_filename}.")
        else:
            print(f"No offset found for {original_filename}, skipping.")
