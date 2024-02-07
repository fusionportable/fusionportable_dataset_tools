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

if __name__ == '__main__':
  file_loader = FileLoader()
  timestamps = file_loader.load_timestamp(file_path='/Rocket_ssd/dataset/FusionPortable_dataset_develop/sensor_data/data_refined/vehicle_highway00/raw_data/ouster00/points/timestamps.txt')
