import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'Times New Roman'
import pandas as pd
import numpy as np
import os

def read_imu_data(file_path):
    columns = ['time', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 
               'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z']
    data = pd.read_csv(file_path, sep=' ', names=columns)
    return data

def plot_time_data(ax, data, sequence, data_type='angular_velocity'):
    colors = [(217/255, 95/255, 2/255, 1.0),    # X axis color - Vermilion
            (27/255, 158/255, 119/255, 1.0),  # Y axis color - Bluish Green
            (0/255, 119/255, 182/255, 1.0)]   # Z axis color - Deep Sky Blue

    #marker_style = {'angular_velocity': 'o', 'linear_acceleration': 's'}
    for axis, color in zip(['x', 'y', 'z'], colors):
        ax.plot(data['time'], data[f'{data_type}_{axis}'], '.', label=axis.upper(), color=color, markersize=4)

    formatted_data_type = data_type.replace('_', ' ').title()
    ax.set_xlabel(f'{formatted_data_type} (Time Domain)', fontsize=20)
    ax.set_ylabel(f'(rad/s)' if data_type == 'angular_velocity' else '(m/s²)', fontsize=20)
    #ax.legend()
    ax.grid(True, linestyle='--')
        # Manually set y-axis limits based on data type
    if data_type == 'angular_velocity':
        ax.set_ylim([-2, 4])  # Replace angular_velocity_range with your value
    elif data_type == 'linear_acceleration':
        ax.set_ylim([-20, 30])  # Replace linear_acceleration_range with your value




def plot_frequency_data(ax, data, sequence, data_type='angular_velocity'):
    colors = [(217/255, 95/255, 2/255, 1.0),    # X axis color - Vermilion
            (27/255, 158/255, 119/255, 1.0),  # Y axis color - Bluish Green
            (0/255, 119/255, 182/255, 1.0)]   # Z axis color - Deep Sky Blue


    sampling_interval = np.mean(np.diff(data['time']))
    n = len(data['time'])

    for axis, color in zip(['x', 'y', 'z'], colors):
        fft_vals = np.fft.rfft(data[f'{data_type}_{axis}'] - np.mean(data[f'{data_type}_{axis}']))
        fft_freq = np.fft.rfftfreq(n, d=sampling_interval)
        ax.plot(fft_freq, 2.0/n * np.abs(fft_vals), '.', label=axis.upper(), color=color, markersize=4)

    formatted_data_type = data_type.replace('_', ' ').title()
    ax.set_xlabel(f'{formatted_data_type} (Frequency Domain)', fontsize=20)
    ax.set_ylabel('Amplitude', fontsize=20)
    #ax.legend()
    ax.grid(True, linestyle='--')



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
    plt.savefig(f'./imus_data/figure/modify/{seq}_motion.png', bbox_inches='tight')
    plt.show()

data_directory = './imus_data/data'
platforms = ['handheld', 'legged', 'ugv', 'vehicle']
seq_plot = ['handheld_room00', 'legged_grass00', 'ugv_campus00', 'vehicle_highway00']

# Process the data directory and plot data for specified platforms and sequences
for platform, seq in zip(platforms, seq_plot):
    process_and_plot_for_platform(data_directory, platform, seq)
