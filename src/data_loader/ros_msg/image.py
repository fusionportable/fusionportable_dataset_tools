import os
import io

import rospy
import rosbag
import numpy as np
from PIL import Image as PILImage

class Image():
	def __init__(self, sensor_type='frame_cam', msg_type='sensor_msgs/Image'):
		self.sensor_type = sensor_type
		self.msg_type = msg_type
		
	def parse_message(self, msg):
		if 'CompressedImage' in self.msg_type:
			# The compressed image data is in msg.data
			# Use io.BytesIO as a buffer for the compressed data
			image_io = io.BytesIO(msg.data)
			pil_image = PILImage.open(image_io)            
			return pil_image
		else:
			# Assuming the encoding is 'rgb8' or 'bgr8'
			if msg.encoding == 'rgb8' or msg.encoding == 'bgr8':
				height, width = msg.height, msg.width
				if msg.encoding == 'rgb8':
					color_order = 'RGB'
				else:
					color_order = 'BGR'
				image_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))               
				# If the encoding is 'bgr8', convert it to 'rgb8'
				if color_order == 'BGR':
					image_array = image_array[:, :, ::-1]
				pil_image = PILImage.fromarray(image_array, mode='RGB')
				return pil_image
			elif msg.encoding == 'mono16':
				height, width = msg.height, msg.width
				image_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
				pil_image = PILImage.fromarray(image_array)
				return pil_image

	def load_messages_write_to_file(self, bag, output_path, topic):
		if not os.path.exists(os.path.join(output_path, 'data')):
			os.makedirs(os.path.join(output_path, 'data'))

		frame_cnt = 0
		timestamps = []
		for _, msg, t in bag.read_messages(topics=[topic]):
			pil_img = self.parse_message(msg)
			self.write_to_file(pil_img, frame_cnt, output_path)
			sec = msg.header.stamp.secs
			nsec = msg.header.stamp.nsecs
			timestamps.append((sec, nsec))
			frame_cnt += 1

		timestamp_filename = os.path.join(output_path, 'timestamps.txt')
		with open(timestamp_filename, 'w') as file:
			for time in timestamps:
				file.write('{}.{:09d}\n'.format(time[0], time[1]))
		return len(timestamps)

	def write_to_file(self, pil_img, frame_cnt, output_path):
		filename = os.path.join(output_path, 'data', '{:06d}.png'.format(frame_cnt))
		pil_img.save(filename)

	def write_to_file_customize(self, pil_img, frame_cnt, output_path, suffix=''):
		filename = os.path.join(output_path, '{:06d}{}'.format(frame_cnt, suffix))
		pil_img.save(filename)

if __name__ == '__main__':
	print('[Test] Loading Outser messages...')
	data_path = '/Rocket_ssd/dataset/FusionPortable_dataset_develop/sensor_data/data_refined/vehicle_campus00'
	ouster_nearir_image = Image(sensor_type='ouster', msg_type='sensor_msgs/Image')
	bag = rosbag.Bag(os.paht.join(data_path, 'vehicle_campus00.bag'))
	ouster_nearir_image.load_messages_write_to_file( \
		bag=bag, \
		output_path=os.paht.join(data_path, 'raw_data/ouster/nearir_image'), \
		topic='/os_image_node/nearir_image')
