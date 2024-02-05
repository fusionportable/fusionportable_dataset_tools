import rosbag
import os

class Sensor:
    def __init__(self, topic):
        self.topic = topic

    def load_messages(self, rosbag_path):
        bag = rosbag.Bag(rosbag_path)
        for topic, msg, t in bag.read_messages(topics=[self.topic]):
            self.write_to_file(msg)
        bag.close()

    def write_to_file(self, msg):
        # Assuming msg has a data attribute that contains the point cloud data.
        # You might need to adjust this method depending on the actual structure of your ROS message.
        file_path = os.path.join('output', '{}.txt'.format(str(msg.header.stamp)))
        with open(file_path, 'w') as file:
            file.write(str(msg.data))
