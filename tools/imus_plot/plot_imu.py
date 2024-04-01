import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from acquire_color_list import color_list

FOLDER_PATH = os.path.dirname(os.path.abspath(__file__))

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.weight'] = 'normal'
plt.rcParams['axes.labelweight'] = 'normal'
plt.rcParams['axes.titleweight'] = 'normal'

def read_imu_data(file_path):
    columns = ['time', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 
               'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z']
    data = pd.read_csv(file_path, sep=' ', names=columns)
    return data

def plot_time_data(ax, data, sequence, data_type='angular_velocity'):
    colors = color_list[:3]

    #marker_style = {'angular_velocity': 'o', 'linear_acceleration': 's'}
    for axis, color in zip(['x', 'y', 'z'], colors):
        ax.plot(data['time'], data[f'{data_type}_{axis}'], '.', label='{}{}'.format(axis.upper(), '-axis'), 
                color=color, markersize=3)

    formatted_data_type = data_type.replace('_', ' ').title()
    ax.set_xlabel(f'Time [s]', fontsize=20)
    ax.set_ylabel(f'Angular Vel. [rad/s]' if data_type == 'angular_velocity' else 'Linear Acc. [m/s²]', fontsize=20)
    # ax.legend()
    ax.grid(True, linestyle='--')
    # Manually set y-axis limits based on data type
    current_xmin, current_xmax = ax.get_xlim()
    ax.set_xlim(left=-3, right=current_xmax-3)
    if data_type == 'angular_velocity':
        ax.set_ylim([-1.5, 3])  # Replace angular_velocity_range with your value
    elif data_type == 'linear_acceleration':
        ax.set_ylim([-12, 20])  # Replace linear_acceleration_range with your value

def plot_frequency_data(ax, data, sequence, data_type='angular_velocity'):
    colors = color_list[:3]

    sampling_interval = np.mean(np.diff(data['time']))
    n = len(data['time'])

    for axis, color in zip(['x', 'y', 'z'], colors):
        fft_vals = np.fft.rfft(data[f'{data_type}_{axis}'] - np.mean(data[f'{data_type}_{axis}']))
        fft_freq = np.fft.rfftfreq(n, d=sampling_interval)
        ax.plot(fft_freq, 2.0/n * np.abs(fft_vals), '.', label=axis.upper(), 
                color=color, markersize=3)

    formatted_data_type = data_type.replace('_', ' ').title()
    if data_type == 'angular_velocity':
        ax.set_xlabel(f'Frequency [Hz] (Angular Vel.)', fontsize=20)
    else:
        ax.set_xlabel(f'Frequency [Hz] (Linear Acc.)', fontsize=20)
    ax.set_ylabel('Amplitude', fontsize=20)
    ax.grid(True, linestyle='--')
    ax.set_xlim([-3, 103])
    if data_type == 'linear_acceleration':
        ax.legend(loc='upper right', fontsize=15)

def process_and_plot_for_platform(directory, platform, seq):
    fig, axs = plt.subplots(1, 4, figsize=(22, 3))  # 1 row, 4 columns for each platform
    #fig.suptitle(f'{seq} - Motion Pattern Analysis', fontsize=20)

    platform_data = {}
    for file_name in os.listdir(directory):
        if 'stim300' in file_name and file_name.startswith(platform) and file_name.endswith(".txt") and seq in file_name:
            sequence = '_'.join(file_name.split('_')[1:-1])
            data = read_imu_data(os.path.join(directory, file_name))
            data['time'] -= data['time'].iloc[0]  # Adjust time so that each sequence starts at 0
            platform_data[sequence] = data
    
    if platform_data:  # Only plot if there's data for the platform
        for idx, (sequence, data) in enumerate(platform_data.items()):
            # Time Domain - Angular Velocity
            plot_time_data(axs[0], data, sequence, data_type='angular_velocity')
            # Time Domain - Linear Acceleration
            plot_time_data(axs[1], data, sequence, data_type='linear_acceleration')
            # Frequency Domain - Angular Velocity
            plot_frequency_data(axs[2], data, sequence, data_type='angular_velocity')
            # Frequency Domain - Linear Acceleration
            plot_frequency_data(axs[3], data, sequence, data_type='linear_acceleration')

        # Set common features and layout
        #for ax in axs.flat:
           # ax.legend(loc='upper right', fontsize='large')

    plt.tight_layout()
    plt.savefig(f'{directory}/../figure/modify/{seq}_motion.png', bbox_inches='tight', format='png', dpi=300)
    plt.show()

if __name__ == "__main__":
    data_directory = os.path.join(FOLDER_PATH, 'data')
    print(f"Loading data from {data_directory}")

    platforms = ['handheld', 'legged', 'ugv', 'vehicle']
    seq_plot = ['handheld_underground00', 'legged_grass00', 'ugv_parking03', 'vehicle_highway00']
    for platform, seq in zip(platforms, seq_plot):
        process_and_plot_for_platform(data_directory, platform, seq)

