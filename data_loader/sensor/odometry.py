import os
import io

import rospy
import rosbag
import numpy as np
from nav_msgs.msg import Odometry

class Odometry():
    def __init__(self, traj_type='TUM', msg_type='nav_msgs/Odometry'):
        self.traj_type = traj_type
        self.msg_type = msg_type
        
    def parse_message(self, msg):
        quaternion = msg.pose.pose.orientation
        quaternion_tuple = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        
        translation = msg.pose.pose.position
        translation_tuple = (translation.x, translation.y, translation.z)
        return quaternion_tuple, translation_tuple

    def load_messages_write_to_file(self, bag, output_path, topic):
        if not os.path.exists(os.path.join(output_path)):
            os.makedirs(os.path.join(output_path))

        timestamps, orientations, translations = [], [], []
        for _, msg, _ in bag.read_messages(topics=[topic]):
            orientation, translation = self.parse_message(msg)
            orientations.append(orientation)
            translations.append(translation)
            sec = msg.header.stamp.secs
            nsec = msg.header.stamp.nsecs
            timestamps.append((sec, nsec))
        self.write_to_file(timestamps, orientations, translations, output_path)
        return len(timestamps)

    def write_to_file(self, timestamps, orientations, translations, output_path):
        odom_filename = os.path.join(output_path, 'odometry.txt')
        if self.traj_type == 'TUM':
            with open(odom_filename, 'w') as file:
                for time, orientation, translation in zip(timestamps, orientations, translations):
                    file.write('{}.{:09d} '.format(time[0], time[1]) + \
                            ''.join(['{:3f} '.format(data) for data in translation]) + \
                            ''.join(['{:3f} '.format(data) for data in orientation]) + '\n')
        elif self.traj_type == 'KITTI':
            pass

if __name__ == '__main__':
    print('[Test] Loading Odometry messages...')
    odometry = Odometry(msg_type='nav_msgs/Odometry')
    bag = rosbag.Bag('/Rocket_ssd/dataset/FusionPortable_dataset_develop/r3live_result/vehicle_highway00_r3live_test.bag')
    num_msg = odometry.load_messages_write_to_file(bag=bag, \
                                                   output_path='/Rocket_ssd/dataset/FusionPortable_dataset_develop/sensor_data/data_refined/vehicle_campus00/r3live_result/odometry', \
                                                   topic='/r3live/aft_mapped_to_init')
    print('Number of messages: {}'.format(num_msg))
