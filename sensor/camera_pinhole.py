#!/usr/bin/python3
import os
import sys
import numpy as np

class CameraPinhole():
  def __init__(self, width, height, camera_name, distortion_model, K, D, Rect, P, T_stereo):
    self.width = width
    self.height = height
    self.camera_name = camera_name
    self.distortion_model = distortion_model
    self.K = K
    self.D = D
    self.Rect = Rect
    self.P = P
    self.T_stereo = T_stereo

  def __str__(self):
    return '{} Intrinsics:\nWidth:{}\nHeight:{}\nK:\n{}\nP:\n{}\nRect:\n{}\n'.format(self.camera_name, self.width, self.height, self.K, self.P, self.Rect)

  def vector_from_image_plane_coordinates(self, u_C):
    return np.linalg.inv(self.K) @ np.array([[u_C[0]], [u_C[1]], [1.0]])

  def get_view_corners(self, min_depth, max_depth):
    ray_0_C = self.vector_from_image_plane_coordinates([0.0, 0.0])
    ray_1_C = self.vector_from_image_plane_coordinates([self.width, 0.0])
    ray_2_C = self.vector_from_image_plane_coordinates([self.width, self.height])
    ray_3_C = self.vector_from_image_plane_coordinates([0.0, self.height])
    corners_C = np.concatenate([min_depth * ray_2_C, min_depth * ray_1_C, min_depth * ray_0_C, min_depth * ray_3_C, 
                                max_depth * ray_2_C, max_depth * ray_1_C, max_depth * ray_0_C, max_depth * ray_3_C], axis=1)
    return corners_C

  def project(self, p_C):
    u_C = np.zeros((2, 1))
    if p_C[2] <= 0.0:
      return (False, u_C)
    inv_z = 1.0 / p_C[2]
    u_C = inv_z * (self.K @ p_C)[:2]
    if (u_C[0] >= self.width or u_C[1] >= self.height or u_C[0] < 0 or u_C[1] < 0):
      return (False, u_C)
    else:
      return (True, u_C)

  def undistort(self, image):
    import cv2
    undistorted_image = cv2.undistort(image, self.K, self.D)
    return undistorted_image
  
if __name__ == '__main__':
  pass
  # calib_param = tsp.CalibParameter_20230906()
  # camera = Camera(width=calib_param.frame_cam00_width, 
  #                 height=calib_param.frame_cam00_height, 
  #                 camera_name='frame_cam00', 
  #                 distortion_model='plumb_bob', 
  #                 K=calib_param.frame_cam00_K, 
  #                 D=calib_param.frame_cam00_D, 
  #                 Rect=calib_param.frame_cam00_Rect, 
  #                 P=calib_param.frame_cam00_P, 
  #                 T_stereo=calib_param.T_stereo)
  # print(camera)
  # corners_C = camera.get_view_corners(min_depth=1.0, max_depth=10.0)
  # print(corners_C.shape)
  # print(corners_C[:, :4])
  # print(corners_C[:, 4:])

  # p_C = np.array([5.0, 3.0, 20.0]).reshape(3, 1)
  # print(camera.project(p_C))
  # print(camera.project(p_C)[0])

  # aabb_min = np.min(corners_C, axis=1) # get minimum given row
  # aabb_max = np.max(corners_C, axis=1)
  # print('aabb_min: {}'.format(aabb_min))
  # print('aabb_max: {}'.format(aabb_max))



  