#!/usr/bin/python3

algorithm_rostopic_msg_frameid_dict = {
  'ouster_points_undistorted': ['/cloud_registered_body', 'sensor_msgs/PointCloud2', 'body'],
  'odometry': ['/Odometry', 'nav_msgs/Odometry', 'camera_init'],
  'path': ['/path', 'nav_msgs/Path', 'camera_init']
}

# TEST
if __name__ == '__main__':
  for key, value in algorithm_rostopic_msg_frameid_dict.items():
    print('Sensor Involved: {:<30}, ROSTopic: {:<50}, Msg_type: {:<60}, Frame_id: {:<15}'.format(\
      key, value[0], value[1], value[2]))