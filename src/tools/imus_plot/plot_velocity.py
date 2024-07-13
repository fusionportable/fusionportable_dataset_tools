import os, sys
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

FOLDER_PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(FOLDER_PATH, '..'))
from acquire_color_list import color_list

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.weight'] = 'normal'
plt.rcParams['axes.labelweight'] = 'normal'
plt.rcParams['axes.titleweight'] = 'normal'

def read_pose_data(file_path):
    columns = ['time', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
    csv_data = pd.read_csv(file_path, sep=' ', names=columns)
    data = {}
    data['time'] = np.array([csv_data['time']]).T - csv_data['time'][0]
    data['translation'] = np.array([csv_data['tx'], csv_data['ty'], csv_data['tz']]).T
    data['quaternion'] = np.array([csv_data['qx'], csv_data['qy'], csv_data['qz'], csv_data['qw']]).T   

    vel = np.zeros((len(csv_data['time']), 1))
    for idx in range(1, vel.shape[0], 1):
        dt = csv_data['time'][idx] - csv_data['time'][idx - 1]
        dis = np.linalg.norm(data['translation'][idx, :] - data['translation'][idx - 1, :])
        # if dis < 0.1:
        #     dis = 0
        vel[idx] = dis / dt
    data['velocity'] = vel
    print(data['velocity'])

    print('Load data from: {}'.format(file_path))
    print('Shape of time: ', data['time'].shape)
    print('Shape of translation: ', data['translation'].shape)
    print('Shape of quaternion: ', data['quaternion'].shape)
    print('Shape of velocity: ', data['velocity'].shape)
    return data

def main():
    data_directory = os.path.join(FOLDER_PATH, 'example_data')
    print(f"Loading data from {data_directory}")
    platforms = ['handheld', 'legged', 'ugv', 'vehicle']
    seq_plot = ['handheld_room00_fastlio2', 'legged_grass00_fastlio2', 'ugv_parking00_fastlio2', 'vehicle_highway00_fastlio2']
    
    fig, axs = plt.subplots(1, 4, figsize=(22, 2.5))  # 1 row, 4 columns for each platform
    for idx, ax in enumerate(axs):
        for file_name in os.listdir(data_directory):
            if file_name.startswith(platforms[idx]) and file_name.endswith(".txt") and seq_plot[idx] in file_name:
                data = read_pose_data(os.path.join(data_directory, file_name))
        
                ax.plot(data['time'], data['velocity'],
                        color=color_list[2], markersize=5)
                ax.set_xlabel('Time [s]', fontsize=20)
                ax.set_ylabel('Linear Vel. [m/s]', fontsize=20)
                ax.tick_params(axis='both', which='major', labelsize=13)
                ax.grid(True, linestyle='--')
                if platforms[idx] == 'vehicle':
                    ax.set_ylim([-0.1, 33])                    
                else:
                    ax.set_ylim([-0.1, 3.5])
                current_xmin, current_xmax = ax.get_xlim()
                ax.set_xlim(left=-3, right=current_xmax-3)

    plt.tight_layout()
    plt.savefig(f'{data_directory}/../figure/velocity_anaylsis.png', bbox_inches='tight', format='png', dpi=300)
    # plt.show()

if __name__ == "__main__":
    main()

