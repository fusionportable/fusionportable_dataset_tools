#!/usr/bin/python3

import os
import sys
import numpy as np
import yaml

sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../sensor'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../tools'))
import eigen_conversion as eigc 
import tf_graph
from camera_pinhole import CameraPinhole
from lidar import Lidar

class IntrinsicExtrinsicLoader():
	def __init__(self, is_print=False):
		self.is_print = is_print
		self.sensor_collection = {}
		self.tf_graph = tf_graph.TFGraph(is_print)

	def load_calibration(self, calib_path, sensor_frameid_dict):
		# Initialize sensor extrinsics object
		self.tf_graph = tf_graph.TFGraph()
		for _, value in sensor_frameid_dict.items():
			self.tf_graph.add_node(frame_id=value[0])

		# Initialize sensor collection object
		self.sensor_collection = {} # sensor_collection['ouster']
		for sensor_name, value in sensor_frameid_dict.items():
			frame_id = value[0]
			yaml_path = os.path.join(calib_path, frame_id + '.yaml')
			# Ouster
			if 'ouster' in sensor_name and not 'imu' in sensor_name:
				self.load_lidar(sensor_name, frame_id, yaml_path)
				if self.is_print:
					print('Loading Int & Ext from {:<20} ...'.format(yaml_path))
			# Frame
			elif 'vehicle_frame' in sensor_name:
				stereo_base_frame_id = 'vehicle_frame_cam00'
				self.load_frame_camera(sensor_name, stereo_base_frame_id, frame_id, yaml_path)
				if self.is_print:
					print('Loading Int & Ext from {:<20} ...'.format(yaml_path))
			elif 'frame' in sensor_name:
				stereo_base_frame_id = 'frame_cam00'
				self.load_frame_camera(sensor_name, stereo_base_frame_id, frame_id, yaml_path)
				if self.is_print:
					print('Loading Int & Ext from {:<20} ...'.format(yaml_path))					
			# Event
			elif 'event' in sensor_name and 'camera' in sensor_name:
				stereo_base_frame_id = 'frame_cam00'
				self.load_event_camera(sensor_name, stereo_base_frame_id, frame_id, yaml_path)
				if self.is_print:
					print('Loading Int & Ext from {:<20} ...'.format(yaml_path))
			# Others
			else:
				if self.is_print:
					print('Unknown sensor: {:<20}'.format(sensor_name))
		
		if self.is_print:
			for sensor_name in self.sensor_collection.keys():
				print('Sensor: {}'.format(sensor_name))
				print(self.sensor_collection[sensor_name])
			
	def load_lidar(self, sensor, frame_id, yaml_path):
		with open(yaml_path, 'r') as yaml_file:
			yaml_data = yaml.safe_load(yaml_file)

			dataset_name = yaml_data['dataset_name']
			lidar_name = yaml_data['lidar_name']
			if 'translation_sensor_ouster00_imu' in yaml_data.keys():
				trans = np.array(yaml_data['translation_sensor_ouster00_imu']['data'])
				quat = np.array(yaml_data['quaternion_sensor_ouster00_imu']['data'])
				tf = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'ouster00_imu', tf)

			if 'translation_sensor_body_imu' in yaml_data.keys():
				trans = np.array(yaml_data['translation_sensor_body_imu']['data'])
				quat = np.array(yaml_data['quaternion_sensor_body_imu']['data'])
				tf = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'body_imu', tf)

			if 'translation_sensor_frame_cam00' in yaml_data.keys():
				trans = np.array(yaml_data['translation_sensor_frame_cam00']['data'])
				quat = np.array(yaml_data['quaternion_sensor_frame_cam00']['data'])
				tf = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'frame_cam00', tf)

			if 'translation_sensor_vehicle_frame_cam00' in yaml_data:
				trans = np.array(yaml_data['translation_sensor_vehicle_frame_cam00']['data'])
				quat = np.array(yaml_data['quaternion_sensor_vehicle_frame_cam00']['data'])
				tf = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'vehicle_frame_cam00', tf)

			if 'translation_sensor_event_cam00' in yaml_data:
				trans = np.array(yaml_data['translation_sensor_event_cam00']['data'])
				quat = np.array(yaml_data['quaternion_sensor_event_cam00']['data'])
				tf = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'event_cam00', tf)
				
			lidar = Lidar(frame_id, dataset_name, lidar_name)
			if self.is_print:
				print(lidar)
			self.sensor_collection[sensor] = lidar

	# NOTE(gogojjh): can only handle pinhole camera
	def load_frame_camera(self, sensor_name, stereo_base_frame_id, frame_id, yaml_path):
		with open(yaml_path, 'r') as yaml_file:
			yaml_data = yaml.safe_load(yaml_file)

			dataset_name = yaml_data['dataset_name']
			camera_name = yaml_data['camera_name']
			distortion_model = yaml_data['distortion_model']
			width = yaml_data['image_width']
			height = yaml_data['image_height']
			K = np.array(yaml_data['camera_matrix']['data']).reshape(3, 3)
			D = np.array(yaml_data['distortion_coefficients']['data']).reshape(1, 5)
			Rect = np.array(yaml_data['rectification_matrix']['data']).reshape(3, 3)
			P = np.array(yaml_data['projection_matrix']['data']).reshape(3, 4)
			trans = np.array(yaml_data['translation_stereo']['data'])
			quat = np.array(yaml_data['quaternion_stereo']['data'])
			T_stereo = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
			self.tf_graph.connect_nodes(frame_id, stereo_base_frame_id, T_stereo)

			if 'translation_sensor_body_imu' in yaml_data:
				trans = np.array(yaml_data['translation_sensor_body_imu']['data'])
				quat = np.array(yaml_data['quaternion_sensor_body_imu']['data'])
				tf = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'body_imu', tf)

			camera = CameraPinhole(frame_id, width, height, dataset_name, camera_name, distortion_model, K, D, Rect, P)	
			if self.is_print:
				print(camera)
			self.sensor_collection[sensor_name] = camera

	# NOTE(gogojjh): can only handle pinhole camera
	def load_event_camera(self, sensor_name, stereo_base_frame_id, frame_id, yaml_path):
		with open(yaml_path, 'r') as yaml_file:
			yaml_data = yaml.safe_load(yaml_file)

			dataset_name = yaml_data['dataset_name']
			camera_name = yaml_data['camera_name']
			distortion_model = yaml_data['distortion_model']
			width = yaml_data['image_width']
			height = yaml_data['image_height']
			K = np.array(yaml_data['camera_matrix']['data']).reshape(3, 3)
			D = np.array(yaml_data['distortion_coefficients']['data']).reshape(1, 5)
			Rect = np.array(yaml_data['rectification_matrix']['data']).reshape(3, 3)
			P = np.array(yaml_data['projection_matrix']['data']).reshape(3, 4)
			trans = np.array(yaml_data['translation_stereo']['data'])
			quat = np.array(yaml_data['quaternion_stereo']['data'])
			T_stereo = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
			self.tf_graph.connect_nodes(frame_id, stereo_base_frame_id, T_stereo)

			if 'translation_sensor_body_imu' in yaml_data:
				trans = np.array(yaml_data['translation_sensor_body_imu']['data'])
				quat = np.array(yaml_data['quaternion_sensor_body_imu']['data'])
				tf = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, 'body_imu', tf)

			ei_frame_id = '{}_imu'.format(frame_id)
			if 'translation_sensor_ecimu'.format(ei_frame_id) in yaml_data:
				trans = np.array(yaml_data['translation_sensor_{}'.format(ei_frame_id)]['data'])
				quat = np.array(yaml_data['quaternion_sensor_{}'.format(ei_frame_id)]['data'])
				tf = eigc.convert_vec_to_matrix(trans, quat[[1, 2, 3, 0]])
				self.tf_graph.connect_nodes(frame_id, ei_frame_id, tf)

			camera = CameraPinhole(frame_id, width, height, dataset_name, camera_name, distortion_model, K, D, Rect, P)	
			if self.is_print:
				print(camera)
			self.sensor_collection[sensor_name] = camera

if __name__ == "__main__":
	# sys.path.append('cfg')
	from cfg.dataset.cfg_vehicle import dataset_sensor_frameid_dict
	int_ext_loader = IntrinsicExtrinsicLoader(is_print=True)
	int_ext_loader.load_calibration(
		calib_path='/home/jjiao/robohike_ws/src/fusionportable_dataset_tools/calibration_files/20230618_calib/calib',
		sensor_frameid_dict=dataset_sensor_frameid_dict)