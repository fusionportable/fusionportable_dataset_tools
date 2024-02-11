#!/usr/bin/python3
import os
import sys
import numpy as np

class FileLoader():
  def __init__(self):
    pass

  def load_timestamp(self, file_path):
    with open(file_path, 'r') as file:
      timestamps_str = [line for line in file]
    timestamps = np.array(timestamps_str, dtype=np.float64)
    return timestamps

  def load_odometry(self, file_path, traj_type='TUM'):
    from scipy.spatial.transform import Rotation as R

    timestamps, quaternions, translations = [], [], []
    with open(file_path, 'r') as file:
      for line in file:
        line_split = line.split()
        if traj_type == 'TUM':
          timestamp = np.array(line_split[0], dtype=np.float64)
          timestamps.append(timestamp)
          tx, ty, tz = float(line_split[1]), float(line_split[2]), float(line_split[3])
          translations.append((tx, ty, tz))
          qx, qy, qz, qw = float(line_split[4]), float(line_split[5]), float(line_split[6]), float(line_split[7])
          quaternions.append((qx, qy, qz, qw))
        elif traj_type == 'KITTI':
          timestamp = np.array(line_split[0], dtype=np.float64)
          timestamps.append(timestamp)
          r11, r12, r13, tx = float(line_split[1]), float(line_split[2]), float(line_split[3]), float(line_split[4])
          r21, r22, r23, ty = float(line_split[5]), float(line_split[6]), float(line_split[7]), float(line_split[8])
          r31, r32, r33, tz = float(line_split[9]), float(line_split[10]), float(line_split[11]), float(line_split[12])
          r41, r42, r43, r44 = float(line_split[13]), float(line_split[14]), float(line_split[15]), float(line_split[16])
          translations.append((tx, ty, tz))
          rotation_matrix = np.array([r11, r12, r13, r21, r22, r23, r31, r32, r33]).reshape(3, 3)
          rotation = R.from_matrix(rotation_matrix)
          quaternion = rotation.as_quat()
          quaternions.append((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    return timestamps, quaternions, translations

if __name__ == '__main__':
  file_loader = FileLoader()
  
  timestamps = file_loader.load_timestamp(file_path='/Rocket_ssd/dataset/FusionPortable_dataset_develop/sensor_data/data_refined/vehicle_highway00/raw_data/ouster00/points/timestamps.txt')

  timestamps, quaternions, translations = file_loader.load_odometry('/Rocket_ssd/dataset/FusionPortable_dataset_develop/sensor_data/data_refined/vehicle_highway00/algorithm_result/odometry/odometry.txt')
  print(timestamps[:2])
  print(quaternions[:2])
  print(translations[:2])