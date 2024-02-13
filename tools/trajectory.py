import numpy as np
import bisect
from scipy.spatial.transform import Rotation as R
import copy

class Trajectory():
  def __init__(self):
    self.tf_matrix = []
    self.timestamps = []

  def append_tf_matrix(self, timestamp, tf_matrix):
    self.timestamps.append(copy.deepcopy(timestamp))
    self.tf_matrix.append(copy.deepcopy(tf_matrix))

  def get_transform_matrix(self, query_timestamp):
    return self.interpolate_pose(query_timestamp)

  def interpolate_pose(self, timestamp):
    if len(self.timestamps) == 0:
      return None
    elif timestamp < self.timestamps[0]:
      return None
    elif timestamp > self.timestamps[-1]:
      return None

    upper_id = bisect.bisect(self.timestamps, timestamp)
    lower_id = upper_id - 1

    rotation1, translation1 = self.tf_matrix[lower_id][:3, :3], self.tf_matrix[lower_id][:3, 3]
    rotation2, translation2 = self.tf_matrix[upper_id][:3, :3], self.tf_matrix[upper_id][:3, 3]

    quat1 = R.from_matrix(rotation1).as_quat()
    quat2 = R.from_matrix(rotation2).as_quat()

    ratio = (timestamp - self.timestamps[lower_id]) / (self.timestamps[upper_id] - self.timestamps[lower_id])
    slerp = R.from_quat([quat1, quat2]).mean(weights=[1-ratio, ratio])
    interpolated_rotation = slerp.as_matrix()
    interpolated_translation = translation1 * (1 - ratio) + translation2 * ratio

    interpolated_matrix = np.eye(4)
    interpolated_matrix[:3, :3] = interpolated_rotation
    interpolated_matrix[:3,  3] = interpolated_translation
    return interpolated_matrix

if __name__ == "__main__":
  traj = Trajectory()
  tf = np.eye(4, 4)
  tf[:3, 3] = np.array([0, 0, 0.2])
  traj.append_tf_matrix(0, tf)

  tf[:3, 3] = np.array([0, 0, 0.4])
  traj.append_tf_matrix(1, tf)
  
  tf[:3, 3] = np.array([0, 0, 0.6])
  traj.append_tf_matrix(2, tf)
  
  tf[:3, 3] = np.array([0, 0, 0.8])
  traj.append_tf_matrix(3, tf)
  
  tf[:3, 3] = np.array([0, 0, 1.0])
  traj.append_tf_matrix(4, tf)
  
  print(traj.get_transform_matrix(-0.1))
  print(traj.get_transform_matrix(0))
  print(traj.get_transform_matrix(0.5))
  print(traj.get_transform_matrix(1.3))