#!/usr/bin/python3

algorithm_rostopic_msg_frameid_dict = {
  'ouster_points_undistorted': ['/r3live/cloud_nonregistered_raw', 'sensor_msgs/PointCloud2', 'r3live_world'],
  'odometry': ['/r3live/aft_mapped_to_init', 'nav_msgs/Odometry', 'r3live_world'],
  'path': ['/r3live/path', 'nav_msgs/Path', 'r3live_world']
}

# TEST
if __name__ == '__main__':
  for key, value in algorithm_rostopic_msg_frameid_dict.items():
    print('ROSTopic: {:<50}, Msg_type: {:<60}, Frame_id: {:<15}'.format(\
      key, value[0], value[1], value[2]))