#!/usr/bin/python3

dataset_sensor_frameid_dict = {
  '3dm_gnss_left': ['3dm_gnss00'],
  '3dm_gnss_right': ['3dm_gnss01'],
  '3dm_imu': ['3dm_imu'],
  '3dm_mag': ['3dm_imu'],
  '3dm_nav': ['3dm_nav'],
  'ouster': ['ouster00'],
  'ouster_imu': ['ouster00_imu'],
  'event_left_camera': ['event_cam00'],
  'event_left_imu': ['event_cam00_imu'],
  'event_right_camera': ['event_cam01'],
  'event_right_imu': ['event_cam01_imu'],
  'frame_left_camera': ['frame_cam00'],
  'frame_right_camera': ['frame_cam01'],
  'vehicle_frame_left_camera': ['vehicle_frame_cam00'],
  'vehicle_frame_right_camera': ['vehicle_frame_cam01'],
  'stim300_imu': ['body_imu']
}

dataset_rostopic_msg_frameid_dict = {
  '3dm_gnss_left_aiding_status': ['/3dm_ins/gnss_left/aiding_status', 'microstrain_inertial_msgs/GNSSAidingStatus', '3dm_gnss00'],
  '3dm_gnss_left_fix': ['/3dm_ins/gnss_left/fix', 'sensor_msgs/NavSatFix', '3dm_gnss00'],
  '3dm_gnss_left_fix_info': ['/3dm_ins/gnss_left/fix_info', 'microstrain_inertial_msgs/GNSSFixInfo', '3dm_gnss00'],
  '3dm_gnss_left_odom': ['/3dm_ins/gnss_left/odom', 'nav_msgs/Odometry', '3dm_gnss00'],
  # 
  '3dm_gnss_right_aiding_status': ['/3dm_ins/gnss_right/aiding_status', 'microstrain_inertial_msgs/GNSSAidingStatus', '3dm_gnss01'],
  '3dm_gnss_right_fix': ['/3dm_ins/gnss_right/fix', 'sensor_msgs/NavSatFix', '3dm_gnss01'],
  '3dm_gnss_right_fix_info': ['/3dm_ins/gnss_right/fix_info', 'microstrain_inertial_msgs/GNSSFixInfo', '3dm_gnss01'],
  '3dm_gnss_right_odom': ['/3dm_ins/gnss_right/odom', 'nav_msgs/Odometry', '3dm_gnss01'],
  # 
  '3dm_imu': ['/3dm_ins/imu/data_raw', 'sensor_msgs/Imu', '3dm_imu'],
  '3dm_mag': ['/3dm_ins/mag/data_raw', 'sensor_msgs/MagneticField', '3dm_imu'],
  '3dm_nav_aiding_summary': ['/3dm_ins/nav/aiding_summary', 'microstrain_inertial_msgs/FilterAidingMeasurementSummary', '3dm_nav'],
  '3dm_nav_dual_antenna_status': ['/3dm_ins/nav/dual_antenna_status', 'microstrain_inertial_msgs/GNSSDualAntennaStatus', '3dm_nav'],
  '3dm_nav_heading': ['/3dm_ins/nav/heading', 'microstrain_inertial_msgs/FilterHeading', '3dm_nav'],
  '3dm_nav_odom': ['/3dm_ins/nav/odom', 'nav_msgs/Odometry', '3dm_nav'],
  '3dm_nav_status': ['/3dm_ins/nav/status', 'microstrain_inertial_msgs/FilterStatus', '3dm_nav'],
  # 
  'ouster_imu': ['/os_cloud_node/imu/data_raw', 'sensor_msgs/Imu', 'ouster00_imu'],
  'ouster_points': ['/os_cloud_node/points', 'sensor_msgs/PointCloud2', 'ouster00'],
  'ouster_nearir_image': ['/os_image_node/nearir_image', 'sensor_msgs/Image', 'ouster00'],
  'ouster_range_image': ['/os_image_node/range_image', 'sensor_msgs/Image', 'ouster00'],
  'ouster_reflec_image': ['/os_image_node/reflec_image', 'sensor_msgs/Image', 'ouster00'],
  'ouster_signal_image': ['/os_image_node/signal_image', 'sensor_msgs/Image', 'ouster00'],
  # 
  'event_left_camera_info': ['/stereo/davis_left/camera_info', 'sensor_msgs/CameraInfo', 'event_cam00'],
  'event_left_events': ['/stereo/davis_left/events', 'dvs_msgs/EventArray', 'event_cam00'],
  'event_left_image_chunk_data': ['/stereo/davis_left/image_chunk_data', 'geometry_msgs/PointStamped', 'event_cam00'],
  'event_left_image': ['/stereo/davis_left/image_raw/compressed', 'sensor_msgs/CompressedImage', 'event_cam00'],
  'event_left_imu': ['/stereo/davis_left/imu/data_raw', 'sensor_msgs/Imu', 'event_cam00_imu', 'event_cam00_imu'],
  # 
  'event_right_camera_info': ['/stereo/davis_right/camera_info', 'sensor_msgs/CameraInfo', 'event_cam01'],
  'event_right_events': ['/stereo/davis_right/events', 'dvs_msgs/EventArray', 'event_cam01'],
  'event_right_image_chunk_data': ['/stereo/davis_right/image_chunk_data', 'geometry_msgs/PointStamped', 'event_cam01'],
  'event_right_image': ['/stereo/davis_right/image_raw/compressed', 'sensor_msgs/CompressedImage', 'event_cam01'],
  'event_right_imu': ['/stereo/davis_right/imu/data_raw', 'sensor_msgs/Imu', 'event_cam01_imu'],
  # 
  'vehicle_frame_left_camera_info': ['/stereo/vehicle_frame_left/camera_info', 'sensor_msgs/CameraInfo', 'frame_cam00'],
  'vehicle_frame_left_image_chunk_data': ['/stereo/vehicle_frame_left/image_chunk_data', 'geometry_msgs/PointStamped', 'frame_cam00'],
  'vehicle_frame_left_image': ['/stereo/vehicle_frame_left/image_raw/compressed', 'sensor_msgs/CompressedImage', 'frame_cam00'],
  # 
  'vehicle_frame_right_camera_info': ['/stereo/vehicle_frame_right/camera_info', 'sensor_msgs/CameraInfo', 'frame_cam01'],
  'vehicle_frame_right_image_chunk_data': ['/stereo/vehicle_frame_right/image_chunk_data', 'geometry_msgs/PointStamped', 'frame_cam01'],
  'vehicle_frame_right_image': ['/stereo/vehicle_frame_right/image_raw/compressed', 'sensor_msgs/CompressedImage', 'frame_cam01'],
  # 
  'stim300_imu': ['/stim300/imu/data_raw', 'sensor_msgs/Imu', 'body_imu'],
  'tf_static': ['/tf_static', 'tf2_msgs/TFMessage', 'none']
}

if __name__ == '__main__':
  for key, value in dataset_sensor_frameid_dict.items():
    print('Sensor: {:<20}, Frame_id: {:<15}'.format(\
      key, value[0]))

  for key, value in dataset_rostopic_msg_frameid_dict.items():
    print('ROSTopic: {:<50}, Msg_type: {:<60}, Frame_id: {:<15}'.format(\
      key, value[0], value[1], value[2]))