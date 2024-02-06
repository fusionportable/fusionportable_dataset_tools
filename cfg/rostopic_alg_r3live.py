r3live_sensor_topic_msg_dict = {
  'ouster_points_undistorted': ['/r3live/cloud_nonregistered_raw', 'sensor_msgs/PointCloud2'],
  'odometry': ['/r3live/aft_mapped_to_init', 'nav_msgs/Odometry'],
  'path': ['/r3live/path', 'nav_msgs/Path']
}

if __name__ == '__main__':
  for key, value in r3live_sensor_topic_msg_dict.items():
    print('Sensor: {}, topic: {}, msg_type: {}'.format(key, value[0], value[1]))