import rospy
import rosbag
import ros_numpy
import numpy as np

import os

class PointCloud():
    def __init__(self, sensor_type='vlp'):
        self.sensor_type = sensor_type
        
    def parse_message(self, msg):
        # Convert PointCloud2 to NumPy array
        cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        if len(cloud_arr) == 0:
            return None, None

        # NOTE(gogojjh): the bug of getting the RGB value needs to be fixed
        if self.sensor_type == 'rgbd_camera':
            return None, None
#             fields = ['x', 'y', 'z', 'rgb'] 
#             structured_arr = np.zeros(cloud_arr.shape[0], dtype={'names': fields, 'formats': ['f4', 'f4', 'f4', 'i8']})
#             pcd_header = f"""# .PCD v.7 - Point Cloud Data file format
# VERSION .7
# FIELDS { ' '.join(structured_arr.dtype.names) }
# SIZE 4 4 4 4
# TYPE F F F U
# COUNT 1 1 1 1
# WIDTH {len(structured_arr)}
# HEIGHT 1
# VIEWPOINT 0 0 0 1 0 0 0
# POINTS {len(structured_arr)}
# DATA ascii
# """
#             print(cloud_arr['rgb'])
#             for field in fields:
#                 structured_arr[field] = cloud_arr[field]
#             return pcd_header, structured_arr
        elif self.sensor_type == 'vlp':
            fields = ['x', 'y', 'z', 'intensity']  # Add 'time' if available in your PointCloud2 message
            structured_arr = np.zeros(cloud_arr.shape[0], dtype={'names': fields, 'formats': ['f4', 'f4', 'f4', 'f4']})
            pcd_header = f"""# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS { ' '.join(structured_arr.dtype.names) }
SIZE { ' '.join(['4'] * len(structured_arr.dtype.names)) }
TYPE { ' '.join(['F'] * len(structured_arr.dtype.names)) }
COUNT { ' '.join(['1'] * len(structured_arr.dtype.names)) }
WIDTH {len(structured_arr)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(structured_arr)}
DATA ascii
"""
            for field in fields:
                structured_arr[field] = cloud_arr[field]
            return pcd_header, structured_arr
        elif self.sensor_type == 'ouster':
            if cloud_arr.ndim == 1: # to address the point cloud with the size (N, )
                cloud_arr = np.expand_dims(cloud_arr, axis=1)
            point_arr = cloud_arr.flatten()
            fields = ['x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range']  
            structured_arr = np.zeros(point_arr.shape[0], dtype={'names': fields, 'formats': ['f4', 'f4', 'f4', 'f8', 'i4', 'i4', 'i4', 'i4', 'i4']})
            pcd_header = f"""# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z intensity t reflectivity ring ambient range
SIZE 4 4 4 4 4 2 1 2 4
TYPE F F F F U U U U U
COUNT 1 1 1 1 1 1 1 1 1
WIDTH {cloud_arr.shape[1]}
HEIGHT {cloud_arr.shape[0]}
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(structured_arr)}
DATA ascii
"""
            for field in fields:
                structured_arr[field] = point_arr[field]
            return pcd_header, structured_arr

    def load_messages_write_to_file(self, bag, output_path, topic):
        if not os.path.exists(os.path.join(output_path, 'data')):
            os.makedirs(os.path.join(output_path, 'data'))

        frame_cnt = 0
        timestamps = []
        for _, msg, t in bag.read_messages(topics=[topic]):
            pcd_header, structured_data = self.parse_message(msg)
            if pcd_header is None:
                continue
            self.write_to_file(pcd_header, structured_data, frame_cnt, output_path)
            sec = msg.header.stamp.secs
            nsec = msg.header.stamp.nsecs
            timestamps.append((sec, nsec))
            frame_cnt += 1
        timestamp_filename = os.path.join(output_path, 'timestamps.txt')
        with open(timestamp_filename, 'w') as file:
            for time in timestamps:
                file.write('{}.{:09d}\n'.format(time[0], time[1]))
        return len(timestamps)

    def write_to_file(self, pcd_header, data, frame_cnt, output_path):
        pcd_filename = os.path.join(output_path, 'data', '{:06d}.pcd'.format(frame_cnt))
        with open(pcd_filename, 'w') as f:
            f.write(pcd_header)
            for point in data:
                f.write(' '.join(str(val) for val in point) + '\n')

if __name__ == '__main__':
    print('[Test] Loading Ouster messages...')
    ouster = PointCloud(sensor_type='ouster')
    bag = rosbag.Bag('/Rocket_ssd/dataset/FusionPortable_dataset_develop/r3live_result/vehicle_highway00_r3live_test.bag')
    ouster.load_messages_write_to_file(bag=bag, \
                                       output_path='/Rocket_ssd/dataset/FusionPortable_dataset_develop/sensor_data/data_refined/vehicle_highway00/r3live_result/ouster00_undistorted/points', \
                                       topic='/r3live/cloud_nonregistered_raw')



