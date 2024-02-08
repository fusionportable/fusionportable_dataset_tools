#!/usr/bin/python3
import os
import sys
import numpy as np

class FileWriter():
  def __init__(self):
    pass

  def write_timestamp(self, timestamps, file_path):
    with open(file_path, 'w') as file:
      for time in timestamps:
        file.write('{:9f}\n'.format(time))

if __name__ == '__main__':
  file_writer = FileWriter()
  file_writer.write_timestamp([335456451.123156465487878, 121212121211.45454554556666], file_path='/Rocket_ssd/dataset/tmp/timestamps.txt')
