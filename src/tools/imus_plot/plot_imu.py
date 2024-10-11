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

def read_imu_data(file_path):
    columns = ['time', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 
               'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z']
    csv_data = pd.read_csv(file_path, sep=' ', names=columns)
    data = {}
    data['time'] = np.array([csv_data['time']]).T - csv_data['time'][0]
    data['angular_velocity'] = np.array([csv_data['angular_velocity_x'], 
                                         csv_data['angular_velocity_y'],
                                         csv_data['angular_velocity_z']]).T
    data['linear_acceleration'] = np.array([csv_data['linear_acceleration_x'], 
                                            csv_data['linear_acceleration_y'],
                                            csv_data['linear_acceleration_z']]).T    
    print('Load data from: {}'.format(file_path))
    print('Shape of time: ', data['time'].shape)
    print('Shape of angular_velocity: ', data['angular_velocity'].shape)
    print('Shape of linear_acceleration: ', data['linear_acceleration'].shape)
    return data

def plot_time_data(ax, data, data_type='angular_velocity'):
    colors = color_list[:3]
    for idx, (axis, mark, color) in enumerate(zip(['X', 'Y', 'Z'], ['.', '.', '.'], colors)):
        ax.plot(data['time'], 
                data['{}'.format(data_type)][:, idx],
                label=axis, color=color, markersize=4)

    ax.set_xlabel('Time [s]', fontsize=20)
    ax.set_ylabel('Angular Vel. [rad/s]' if data_type == 'angular_velocity' else 
                  'Linear Acc. [m/s²]', 
                  fontsize=20)
    ax.grid(True, linestyle='--')

    current_xmin, current_xmax = ax.get_xlim()
    ax.set_xlim(left=-3, right=current_xmax-3)
    if data_type == 'angular_velocity':
        ax.set_ylim([-1.5, 3])  # Replace angular_velocity_range with your value
    elif data_type == 'linear_acceleration':
        ax.set_ylim([-12, 20])  # Replace linear_acceleration_range with your value
    ax.tick_params(axis='both', which='major', labelsize=13)

def plot_frequency_data(ax, data, data_type='angular_velocity'):
    colors = color_list[:3]

    sampling_interval = np.mean(np.diff(data['time'], axis=0))
    N = len(data['time'])
    for idx, (axis, mark, color) in enumerate(zip(['X', 'Y', 'Z'], ['.', '.', '.'], colors)):
        fft_vals = np.fft.rfft(data['{}'.format(data_type)][:, idx] - np.mean(data['{}'.format(data_type)][:, idx]))
        fft_freq = np.fft.rfftfreq(N, d=sampling_interval)
        ax.plot(fft_freq, 2.0 / N * np.abs(fft_vals), 
                label=axis, color=color, markersize=4)

    ax.set_xlabel('Frequency [Hz] (Angular Vel.)' if data_type == 'angular_velocity' else 
                  'Frequency [Hz] (Linear Acc.)', 
                  fontsize=20)
    ax.set_ylabel('Amplitude', fontsize=20)

    ax.grid(True, linestyle='--')

    current_xmin, current_xmax = ax.get_xlim()
    ax.set_xlim(left=-3, right=current_xmax-3)
    if data_type == 'linear_acceleration':
        ax.legend(loc='upper right', fontsize=20, markerscale=4)
    ax.tick_params(axis='both', which='major', labelsize=14)

def process_and_plot_for_platform(directory, platform, seq):
    fig, axs = plt.subplots(1, 4, figsize=(22, 2.8))  # 1 row, 4 columns for each platform
    platform_data = {}
    for file_name in os.listdir(directory):
        if 'stim300' in file_name and file_name.startswith(platform) and file_name.endswith(".txt") and seq in file_name:
            sequence = '_'.join(file_name.split('_')[1:-1])
            data = read_imu_data(os.path.join(directory, file_name))
            platform_data[sequence] = data
    
    if platform_data:  # Only plot if there's data for the platform
        for idx, (sequence, data) in enumerate(platform_data.items()):
            plot_time_data(axs[0], data, data_type='angular_velocity')
            plot_time_data(axs[1], data, data_type='linear_acceleration')
            plot_frequency_data(axs[2], data, data_type='angular_velocity')
            plot_frequency_data(axs[3], data, data_type='linear_acceleration')

    plt.tight_layout()
    plt.savefig(f'{directory}/../figure/{seq}_motion.png', bbox_inches='tight', format='png', dpi=300)
    # plt.show()

def main():
    data_directory = os.path.join(FOLDER_PATH, 'example_data')
    os.makedirs(f'{data_directory}/../figure', exist_ok=True)
    print(f"Loading data from {data_directory}")

    # platforms = ['handheld', 'legged', 'ugv', 'vehicle']
    # seq_plot = ['handheld_room00', 'legged_grass00', 'ugv_parking00', 'vehicle_highway00']
    platforms = ['handheld']
    seq_plot = ['handheld_room00']
    for platform, seq in zip(platforms, seq_plot):
        process_and_plot_for_platform(data_directory, platform, seq)    

if __name__ == "__main__":
    main()

