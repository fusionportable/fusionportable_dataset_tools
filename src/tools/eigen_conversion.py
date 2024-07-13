#! /usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

def convert_vec_to_matrix(vec_p, vec_q): # x, y, z, qx, qy, qz, qw
	tf = np.eye(4)
	tf[:3,:3] = Rotation.from_quat(vec_q).as_matrix()
	tf[:3, 3] = vec_p
	return tf

def convert_matrix_to_vec(tf_matrix):	
	vec_p = tf_matrix[:3, 3]
	vec_q = Rotation.from_matrix(tf_matrix[:3,:3]).as_quat()
	return vec_p, vec_q

if __name__ == '__main__':
	vec_p = np.array([0.1, 0.3, 0.6])
	vec_q = np.array([0.0, 0.0, 0.70710678, 0.70710678])
	tf = convert_vec_to_matrix(vec_p, vec_q)
	print(tf)

	vec_p, vec_q = convert_matrix_to_vec(tf)
	print(vec_p)
	print(vec_q)