import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d  # Ensure 3D support is imported
from matplotlib.font_manager import FontProperties
def load_tum_format_trajectory(filename):
    data = pd.read_csv(filename, sep=' ', header=None, names=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    return data

def visualize_final_complete_trajectories_2D(datasets, ax, labels, font):
    markers = ['^', 'o', 's', 'p', '*']
    colors = ['b', 'g', 'r', 'c', 'm']
    linewidth = 1.8
    for i, data in enumerate(datasets):
        ax.plot(data['tx'], data['ty'], color=colors[i % len(colors)], linewidth=linewidth, label=labels[i])
        ax.scatter(data['tx'].iloc[0], data['ty'].iloc[0], c=colors[i % len(colors)], marker=markers[0], s=100)
        ax.scatter(data['tx'].iloc[-1], data['ty'].iloc[-1], c=colors[i % len(colors)], marker=markers[1], s=100)
    ax.grid(True, which='both', linestyle='-', linewidth=2, alpha=0.7)
    for spine in ax.spines.values():
        spine.set_color('black')
        spine.set_linewidth(1.5)
    ax.spines['right'].set_visible(True)
    ax.spines['top'].set_visible(True)
    ax.set_xlabel('X [m]', fontproperties=font)
    ax.set_ylabel('Y [m]', fontproperties=font)
   # ax.set_title('2D Trajectories', fontproperties=font)
    # ax.legend(handles=ax.lines + [plt.Line2D([0], [0], color=colors[0], marker=markers[0], linestyle='None', markersize=10, label='Start'),
    #                               plt.Line2D([0], [0], color=colors[0], marker=markers[1], linestyle='None', markersize=10, label='End')],
    #           prop=font, loc='upper left', bbox_to_anchor=(1.1,1), edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
    ax.axis('equal')

def visualize_final_complete_trajectories_3D(datasets, ax, labels, font):
    markers = ['^', 'o', 's', 'p', '*']
    colors = ['b', 'g', 'r', 'c', 'm']
    linewidth = 1.8
    for i, data in enumerate(datasets):
        ax.plot(data['tx'], data['ty'], data['tz'], color=colors[i % len(colors)], linewidth=linewidth, label=labels[i])
        ax.scatter(data['tx'].iloc[0], data['ty'].iloc[0], data['tz'].iloc[0], c=colors[i % len(colors)], marker=markers[0], s=100)
        ax.scatter(data['tx'].iloc[-1], data['ty'].iloc[-1], data['tz'].iloc[-1], c=colors[i % len(colors)], marker=markers[1], s=100)
    ax.grid(True, which='both', linestyle='-', linewidth=2, alpha=0.7)
    ax.w_xaxis.pane.fill = False
    ax.w_yaxis.pane.fill = False
    ax.w_zaxis.pane.fill = False
    ax.w_xaxis.pane.set_edgecolor('black')
    ax.w_yaxis.pane.set_edgecolor('black')
    ax.w_zaxis.pane.set_edgecolor('black')
    ax.w_xaxis.pane.set_linewidth(1.5)
    ax.w_yaxis.pane.set_linewidth(1.5)
    ax.w_zaxis.pane.set_linewidth(1.5)
    ax.w_xaxis.line.set_color("black")
    ax.w_xaxis.line.set_linewidth(1.5)
    ax.w_yaxis.line.set_color("black")
    ax.w_yaxis.line.set_linewidth(1.5)
    ax.w_zaxis.line.set_color("black")
    ax.w_zaxis.line.set_linewidth(1.5)
    ax.set_xlabel('X [m]', fontproperties=font,labelpad=25)
    ax.set_ylabel('Y [m]', fontproperties=font,labelpad=25)
    ax.set_zlabel('Z [m]', fontproperties=font,labelpad=25)
   # ax.set_title('3D Trajectories', fontproperties=font)
    # ax.legend(handles=ax.lines[:len(labels)] + [plt.Line2D([0], [0], color=colors[0], marker=markers[0], linestyle='None', markersize=10, label='Start'),
    #                                             plt.Line2D([0], [0], color=colors[0], marker=markers[1], linestyle='None', markersize=10, label='End')],
    #           prop=font, loc='upper left', bbox_to_anchor=(1.1, 1),  edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)

# Adjusted font properties
fonesize = 35
font = FontProperties()
font.set_family('serif')
font.set_name('Times New Roman')
font.set_size(fonesize)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
plt.rcParams['axes.labelweight'] = 'normal'
plt.rcParams['font.size'] = fonesize
plt.rcParams['xtick.labelsize'] = fonesize
plt.rcParams['ytick.labelsize'] = fonesize
plt.rcParams['legend.fontsize'] = fonesize

# Load the data from multiple files
#filenames = ["/home/cranesoar/DATASET/FusionPortable/ijrr/benchmark_private/vinsfusion_lc/laptop/traj/handheld_room00_vinfusion(lc).txt", "/home/cranesoar/DATASET/FusionPortable/ijrr/benchmark_private/fastlio2/laptop/traj/handheld_room00_fastlio2.txt", "/home/cranesoar/DATASET/FusionPortable/ijrr/benchmark_private/droid/handheld_room00_droid.txt", "/home/cranesoar/DATASET/FusionPortable/ijrr/benchmark_private/r3live_new/laptop/traj/handheld_room00_r3live.txt", "/home/cranesoar/DATASET/FusionPortable/ijrr/groundtruth/traj/ms/MS60_gt/correlated/handheld_room00_correlated.txt"]  # Add your file names here
# Reordered filenames for handheld_room00

# filenames = [
#     "./data/handheld_room00/handheld_room00_droid.txt",
#     "./data/handheld_room00/handheld_room00_vinfusion(lc).txt",
#     "./data/handheld_room00/handheld_room00_fastlio2.txt",
#     "./data/handheld_room00/handheld_room00_r3live.txt",
#     "./data/handheld_room00/handheld_room00_initialed.txt"
# ]

# Reordered filenames for legged_grass00
filenames = [
    # "./data/legged_grass00/legged_grass00_droid.txt",
    # "./data/legged_grass00/legged_grass00_vinfusion(lc).txt",
    "./data/legged_grass00/legged_grass00_fastlio2.txt",
    "./data/legged_grass00/legged_grass00_r3live.txt",
    "./data/legged_grass00/legged_grass00_initialed.txt"
]


datasets = [load_tum_format_trajectory(f) for f in filenames]

# Extract labels from filenames
labels = []
for f in filenames:
    filename = f.split('/')[-1]
    if "initialed" in filename:
        labels.append("Groundtruth")
    elif "droid" in filename:
        labels.append("DROID-SLAM")
    elif "fastlio2" in filename:
        labels.append("FAST-LIO2")
    elif "vinfusion(lc)" in filename:
        labels.append("Vins-Fusion(LC)")
    elif "r3live" in filename:
        labels.append("R3LIVE")
    else:
        labels.append("Unknown")  # Fallback label
# Now 'labels' contains the modified labels as per your requirement


# 过滤空的数据集并更新标签
datasets, labels = zip(*[(data, label) for data, label in zip(datasets, labels) if not data.empty])
# Plot final complete 2D trajectories
fig_2d, ax_2d = plt.subplots(figsize=(8, 6))
visualize_final_complete_trajectories_2D(datasets, ax_2d, labels, font)
plt.tight_layout()
# Plot final complete 3D trajectories
fig_3d = plt.figure(figsize=(8, 6))
ax_3d = fig_3d.add_subplot(111, projection='3d')
# visualize_final_complete_trajectories_3D(datasets, ax_3d, labels, font)
plt.tight_layout()
plt.show()
# Optionally, save the plots
# fig_3d.savefig("3D_trajectory_visualization.pdf", bbox_inches='tight', dpi=300)
# fig_2d.savefig("2D_trajectory_visualization.pdf", bbox_inches='tight', dpi=300)