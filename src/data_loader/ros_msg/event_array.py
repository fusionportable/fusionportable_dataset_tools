import os
import io

import rospy
import rosbag
import numpy as np

class EventArray():
    def __init__(self, sensor_type='event_cam', msg_type='dvs_msgs/EventArray'):
        self.sensor_type = sensor_type
        self.msg_type = msg_type

    def parse_message(self, msg):
        events = []
        for event in msg.events:
            event = [event.ts.secs, event.ts.nsecs, event.x, event.y, event.polarity]
            events.append(event)
        return events

    def load_messages_write_to_file(self, bag, output_path, topic):
        if not os.path.exists(os.path.join(output_path, 'data')):
            os.makedirs(os.path.join(output_path, 'data'))

        frame_cnt = 0
        timestamps = []
        for _, msg, _ in bag.read_messages(topics=[topic]):
            timestamps.append([msg.header.stamp.secs, msg.header.stamp.nsecs])
            event_list = self.parse_message(msg)
            self.write_event_to_file(event_list, frame_cnt, output_path)
            frame_cnt += 1

        self.write_timestamps_to_file(timestamps, output_path)
        return frame_cnt

    def write_event_to_file(self, event_list, frame_cnt, output_path):
        filename = os.path.join(output_path, 'data', '{:06d}.txt'.format(frame_cnt))
        np.savetxt(filename, np.array(event_list), fmt='%d.%09d %d %d %d')

    def write_timestamps_to_file(self, timestamps, output_path):
        timestamp_filename = os.path.join(output_path, 'timestamp.txt')
        np.savetxt(timestamp_filename, np.array(timestamps), fmt='%d.%09d')

if __name__ == '__main__':
    print('[Test] Loading EventArray messages...')

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_rosbag', type=str, help='path_to_rosbag.bag')
    parser.add_argument('--path_output', type=str, help='path_output')
    parser.add_argument('--topic_event_array', type=str, help='topic_event_array')
    args = parser.parse_args()
    print("Arguments:\n{}".format('\n'.join(['-{}: {}'.format(k, v) for k, v in args.__dict__.items()])))

    # Usage: python3 event_array.py --path_rosbag path_to_rosbag.bag --path_output path_output --topic_event_array topic_event_array
    event_array = EventArray(msg_type='dvs_msgs/EventArray')
    bag = rosbag.Bag(args.path_rosbag)
    num_events = event_array.load_messages_write_to_file(bag=bag, output_path=args.path_output, topic=args.topic_event_array)
    print('Number of events: {}'.format(num_events))