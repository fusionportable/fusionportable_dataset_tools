#!/usr/bin/python3

import os
import sys
import numpy as np
import yaml

sys.path.append('tools')
import eigen_conversion 
import tf_graph

sys.path.append('sensor')
from camera_pinhole import CameraPinhole
from lidar import Lidar

# sys.path.append(os.path.join(os.path.dirname(__file__), 'tools'))

class IntrinsicExtrinsicLoader():
	def __init__(self, is_print=False):
		self.is_print = is_print
		self.sensor_collection = {}
		self.tf_graph = tf_graph.TFGraph()

	def load_calibration(self, calib_path, sensor_frameid_dict):
		# Initialize sensor extrinsics object
		for _, value in sensor_frameid_dict.items():
			self.tf_graph.add_node(frame_id=value[0])

		# Initialize sensor collection object
		self.sensor_collection = {} # sensor_collection['ouster']
		for sensor, value in sensor_frameid_dict.items():
			frame_id = value[0]
			yaml_path = os.path.join(calib_path, frame_id + '.yaml')
			if 'ouster' in sensor and not 'imu' in sensor:
				print('Loading Intrinsic Extrinsics from {:<20} ...'.format(yaml_path))
				self.load_lidar(sensor, frame_id, yaml_path)
			elif 'frame' in sensor:
				print('Loading Intrinsic Extrinsics from {:<20} ...'.format(yaml_path))
				self.load_frame_camera(sensor, frame_id, yaml_path)
			elif 'event' in sensor and 'camera' in sensor:
				print('Loading Intrinsic Extrinsics from {:<20} ...'.format(yaml_path))
				self.load_event_camera(sensor, frame_id, yaml_path)
			else:
				if self.is_print:
					print('Unknown sensor: {:<20}'.format(sensor))
		
		if self.is_print:
			print('Sensors:')
			for sensor in self.sensor_collection.keys():
				print('Sensor: {}'.format(sensor))
				print(self.sensor_collection[sensor])

			print('Extrinsics:')
			for frame_id, links in self.extrinsics_collection.items():
				if len(links) > 0:
					print('Frame id: {}'.format(frame_id))
					for child_frame_id, tf_matrix in links.items():
						print('      {} - {}:'.format(frame_id, child_frame_id))
						print('{}'.format(tf_matrix))
			
	def load_lidar(self, sensor, frame_id, yaml_path):
		with open(yaml_path, 'r') as yaml_file:
			yaml_data = yaml.safe_load(yaml_file)

			lidar_name = yaml_data['lidar_name']

			if 'translation_sensor_frame_cam00' in yaml_data.keys():
				translation = np.array(yaml_data['translation_sensor_frame_cam00']['data'])
				quaternion = np.array(yaml_data['quaternion_sensor_frame_cam00']['data'])
				tf = eigen_conversion.convert_vec_to_matrix(translation, quaternion[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'frame_cam00', tf)

			if 'translation_sensor_vehicle_frame_cam00' in yaml_data:
				translation = np.array(yaml_data['translation_sensor_vehicle_frame_cam00']['data'])
				quaternion = np.array(yaml_data['quaternion_sensor_vehicle_frame_cam00']['data'])
				tf = eigen_conversion.convert_vec_to_matrix(translation, quaternion[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'vehicle_frame_cam00', tf)

			if 'translation_sensor_event_cam00' in yaml_data:
				translation = np.array(yaml_data['translation_sensor_event_cam00']['data'])
				quaternion = np.array(yaml_data['quaternion_sensor_event_cam00']['data'])
				tf = eigen_conversion.convert_vec_to_matrix(translation, quaternion[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'event_cam00', tf)
				
			lidar = Lidar(lidar_name)
			if self.is_print:
				print(lidar)
			self.sensor_collection[sensor] = lidar

	# NOTE(gogojjh): can only handle pinhole camera
	def load_frame_camera(self, sensor, frame_id, yaml_path):
		with open(yaml_path, 'r') as yaml_file:
			yaml_data = yaml.safe_load(yaml_file)

			camera_name = yaml_data['camera_name']
			distortion_model = yaml_data['distortion_model']
			width = yaml_data['image_width']
			height = yaml_data['image_height']
			K = np.array(yaml_data['camera_matrix']['data']).reshape(3, 3)
			D = np.array(yaml_data['distortion_coefficients']['data']).reshape(1, 5)
			Rect = np.array(yaml_data['rectification_matrix']['data']).reshape(3, 3)
			P = np.array(yaml_data['projection_matrix']['data']).reshape(3, 4)
			translation = np.array(yaml_data['translation_stereo']['data'])
			quaternion = np.array(yaml_data['quaternion_stereo']['data'])
			T_stereo = eigen_conversion.convert_vec_to_matrix(translation, quaternion[[1, 2, 3, 0]])

			if 'translation_sensor_body_imu' in yaml_data:
				translation = np.array(yaml_data['translation_sensor_body_imu']['data'])
				quaternion = np.array(yaml_data['quaternion_sensor_body_imu']['data'])
				tf = eigen_conversion.convert_vec_to_matrix(translation, quaternion[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'body_imu', tf)

			camera = CameraPinhole(width, height, camera_name, distortion_model, K, D, Rect, P, T_stereo)	
			if self.is_print:
				print(camera)
			self.sensor_collection[sensor] = camera

	# NOTE(gogojjh): can only handle pinhole camera
	def load_event_camera(self, sensor, frame_id, yaml_path):
		with open(yaml_path, 'r') as yaml_file:
			yaml_data = yaml.safe_load(yaml_file)

			camera_name = yaml_data['camera_name']
			distortion_model = yaml_data['distortion_model']
			width = yaml_data['image_width']
			height = yaml_data['image_height']
			K = np.array(yaml_data['camera_matrix']['data']).reshape(3, 3)
			D = np.array(yaml_data['distortion_coefficients']['data']).reshape(1, 5)
			Rect = np.array(yaml_data['rectification_matrix']['data']).reshape(3, 3)
			P = np.array(yaml_data['projection_matrix']['data']).reshape(3, 4)
			translation = np.array(yaml_data['translation_stereo']['data'])
			quaternion = np.array(yaml_data['quaternion_stereo']['data'])
			T_stereo = eigen_conversion.convert_vec_to_matrix(translation, quaternion[[1, 2, 3, 0]])

			if 'translation_sensor_body_imu' in yaml_data:
				translation = np.array(yaml_data['translation_sensor_body_imu']['data'])
				quaternion = np.array(yaml_data['quaternion_sensor_body_imu']['data'])
				tf = eigen_conversion.convert_vec_to_matrix(translation, quaternion[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'body_imu', tf)

			ei_frame_id = '{}_imu'.format(frame_id)
			if 'translation_sensor_{}'.format(ei_frame_id) in yaml_data:
				translation = np.array(yaml_data['translation_sensor_{}'.format(ei_frame_id)]['data'])
				quaternion = np.array(yaml_data['quaternion_sensor_{}'.format(ei_frame_id)]['data'])
				tf = eigen_conversion.convert_vec_to_matrix(translation, quaternion[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, ei_frame_id, tf)

			camera = CameraPinhole(width, height, camera_name, distortion_model, K, D, Rect, P, T_stereo)	
			if self.is_print:
				print(camera)
			self.sensor_collection[sensor] = camera

if __name__ == "__main__":
	sys.path.append('cfg')
	from dataset.cfg_vehicle import dataset_sensor_frameid_dict
	int_ext_loader = IntrinsicExtrinsicLoader(is_print=True)
	int_ext_loader.load_calibration(calib_path='/Titan/dataset/FusionPortable_dataset_develop/calibration_files/20230618_calib/calib', \
																	sensor_frameid_dict=dataset_sensor_frameid_dict)