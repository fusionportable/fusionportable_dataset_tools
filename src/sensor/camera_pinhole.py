#!/usr/bin/python3

import cv2
import numpy as np
from camera import Camera
import math

class CameraPinhole(Camera):
  def __init__(self, frame_id, width, height, dataset_name, camera_name, distortion_model, K, D, Rect, P):
    super().__init__(frame_id, width, height, dataset_name, camera_name, distortion_model, K, D, Rect, P)

  def undistort(self, image):
    undistorted_image = cv2.undistort(image, self.K, self.D)
    return undistorted_image
  
  def undistort_pixel(self, pixel):
      """
      Undistorts a pixel coordinate using camera intrinsics and distortion coefficients
      
      Args:
          pixel: Tuple/list of (u, v) coordinates
          K: Camera matrix (3x3 numpy array)
          D: Distortion coefficients (k1, k2, p1, p2[, k3[, k4, k5, k6]])
          
      Returns:
          Tuple of (u_undist, v_undist) corrected coordinates
      """
      # Convert input to OpenCV-friendly format
      points = np.array([[pixel]], dtype=np.float32)
      
      # Undistort the points
      undistorted_points = cv2.undistortPoints(
          points, 
          self.K, 
          self.D, 
          None,  # No rectification matrix
          self.K      # Use original camera matrix for output
      )
      
      # Extract and return the undistorted coordinates
      u, v = undistorted_points[0][0]
      return (math.floor(u), math.floor(v))

def test_main():
  """Main function to test the Camera class."""
  # Create a pinhole camera model
  D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
  K = np.array([205.46963709898583, 0.0, 320.5, 0.0, 205.46963709898583, 180.5, 0.0, 0.0, 1.0]).reshape(3, 3)
  Rect = np.array([[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]]).reshape(3, 3)
  P = np.array([205.46963709898583, 0.0, 320.5, -0.0, 0.0, 205.46963709898583, 180.5, 0.0, 0.0, 0.0, 1.0, 0.0]).reshape(3, 4)
  camera_pinhole = CameraPinhole(frame_id='frame_cam00', width=640, height=360, 
                                 camera_name='stereo_left_Flir_BFSU3', 
                                 distortion_model='plumb_bob', 
                                 K=K, D=D, Rect=Rect, P=P)
  print(camera_pinhole)
  
  # Compute the camera frustum
  corners_C = camera_pinhole.get_view_corners(min_depth=1.0, max_depth=10.0)
  print(corners_C[:, :4])
  print(corners_C[:, 4:])

  # Project a 3D point into the pixel plane
  p_C = np.array([5.0, 3.0, 20.0]).reshape(3, 1)
  _, tmp_p = camera_pinhole.project(p_C)
  u_C1 = tmp_p[:, 0]
  tmp_p, _ = cv2.projectPoints(p_C.reshape(1, 1, 3), np.zeros((3, 1)), np.zeros((3, 1)), K, D)
  u_C2 = tmp_p[0, 0, :2]
  print(np.array_equal(u_C1, u_C2))

  # Compute the AABB
  aabb_min = np.min(corners_C, axis=1)
  aabb_max = np.max(corners_C, axis=1)
  print('aabb_min: {}'.format(aabb_min))
  print('aabb_max: {}'.format(aabb_max))

if __name__ == '__main__':
  test_main()
