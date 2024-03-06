import subprocess
import matplotlib.pyplot as plt
import re


##legged_room00  3.01
##legged_grass00 3.37
##legged_grass01 3.49
##legged_transition00 3.36
##legged_tunnel00 3.31
##handheld_grass00 -0.23
##handheld_room00 -0.72
##handheld_room01 -0.73
##handheld_starbucks00 -0.64
##handheld_starbucks01 -0.67
##handheld_tunnel00 3.2
##ugv_transition00 0.02
##ugv_transition01 0.06

# 初始化参数
t_offset_values = [i * 0.01 for i in range(-100, 0)]  # 从-0.5到0.5，步长为0.1
#t_offset_values = [i * 0.1 for i in range(-20, 40)]  # 从-0.5到0.5，步长为0.1
results = []

# 循环执行命令
for t_offset in t_offset_values:
    command = [
        'evo_ape', 'tum',
        './resampled/handheld_starbucks00_resampled.txt',
        '../../../../benchmark_private/fastlio2/laptop/traj/handheld_starbucks00_fastlio2.txt', '-a', '--t_max_diff', '0.1', '--t_offset', str(t_offset)
    ]
    #print(command)
    process = subprocess.run(command, capture_output=True, text=True)
    
    # 首先打印完整的命令输出
    output = process.stdout
    print(f"Command output at t_offset {t_offset}:\n{output}")
    
    # 然后解析输出以提取APE的RMSE值
    match = re.search(r"rmse\s+(\d+\.\d+)", output)  # 使用正则表达式匹配RMSE值
    if match:
        ape_rmse = float(match.group(1))
        results.append(ape_rmse)
        print(f"Extracted RMSE: {ape_rmse} at {t_offset}")
    else:
        results.append(None)  # 如果未找到匹配项，添加None
        print("RMSE value not found.")

# # 绘制结果
# plt.figure(figsize=(10, 6))
# plt.plot(t_offset_values, results, marker='o')
# plt.title('APE Mean vs. Time Offset')
# plt.xlabel('Time Offset (s)')
# plt.ylabel('APE Mean')
# plt.grid(True)
# plt.show()
        
# 找出 RMSE 最小值及其对应的 t_offset
min_rmse = min(results)  # 假设 results 中没有 None 值
min_rmse_index = results.index(min_rmse)
min_rmse_offset = t_offset_values[min_rmse_index]
print(f" The minimum offset is {min_rmse_offset}")

# 绘制 RMSE 曲线
plt.figure(figsize=(10, 6))
plt.plot(t_offset_values, results, marker='o', label='RMSE')

# 标记 RMSE 最小值的点
plt.scatter(min_rmse_offset, min_rmse, color='red', s=100, label='Min RMSE', zorder=5)

# 在标记点上添加注释显示对应的 t_offset 值
plt.annotate(f"t_offset={min_rmse_offset}\nRMSE={min_rmse:.3f}", 
             (min_rmse_offset, min_rmse),
             textcoords="offset points", 
             xytext=(-10,10), 
             ha='center',
             arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

plt.title('RMSE vs. Time Offset')
plt.xlabel('Time Offset (s)')
plt.ylabel('RMSE')
plt.legend()
plt.grid(True)
plt.show()
