#!/usr/bin/python3
import numpy as np

class Camera:
  """Represents a camera with intrinsic and extrinsic parameters."""

  def __init__(self, frame_id, width, height, dataset_name, camera_name, distortion_model, K, D, Rect, P):
    """
    Initializes the Camera object.

    Args:
        frame_id: The name of the coordinate frame
        width (int): The width of the camera image.
        height (int): The height of the camera image.
        dataset_name (str): The name of the dataset, i.e., calib_20230618.
        camera_name (str): The name of the camera.
        distortion_model (str): The distortion model of the camera.
        K (np.ndarray): Intrinsic camera matrix.
        D (np.ndarray): Distortion coefficients.
        Rect (np.ndarray): Rectification matrix.
        P (np.ndarray): Projection matrix.
    """
    self.frame_id = frame_id
    self.width = width
    self.height = height
    self.dataset_name = dataset_name
    self.camera_name = camera_name
    self.distortion_model = distortion_model
    self.K = K
    self.D = D
    self.Rect = Rect
    self.P = P

  def __str__(self):
    """Returns a string representation of the Camera object."""
    return ('Camera ({}) with intrinsics at {}:\n'
            'Width: {}, '
            'Height: {}\n'
            'Distortion Model: {}\n'
            'K:\n{}\n'
            'P:\n{}\n'
            'Rect:\n{}\n').format(self.camera_name, self.frame_id, self.width, self.height, 
                                  self.distortion_model, self.K, self.P, self.Rect)

  def vector_from_image_plane_coordinates(self, u_C):
    """
    Computes the vector from image plane coordinates.

    Args:
        u_C (list): Image plane coordinates [u, v].

    Returns:
        np.ndarray: 3D vector from image plane coordinates.
    """
    return np.linalg.inv(self.K) @ np.array([[u_C[0]], [u_C[1]], [1.0]])

  def get_view_corners(self, min_depth, max_depth):
    """
    Computes the view corners at specified depths.

    Args:
        min_depth (float): Minimum depth.
        max_depth (float): Maximum depth.

    Returns:
        np.ndarray: 3D coordinates of view corners.
    """
    ray_0_C = self.vector_from_image_plane_coordinates([0.0, 0.0])
    ray_1_C = self.vector_from_image_plane_coordinates([self.width, 0.0])
    ray_2_C = self.vector_from_image_plane_coordinates([self.width, self.height])
    ray_3_C = self.vector_from_image_plane_coordinates([0.0, self.height])
    corners_C = np.concatenate(
        [min_depth * ray_2_C, min_depth * ray_1_C, min_depth * ray_0_C, min_depth * ray_3_C, 
          max_depth * ray_2_C, max_depth * ray_1_C, max_depth * ray_0_C, max_depth * ray_3_C], axis=1)
    return corners_C

  def project(self, p_C):
    """
    Projects 3D points to 2D image plane.

    Args:
        p_C (np.ndarray): 3D point in camera coordinates.

    Returns:
        tuple: (bool, np.ndarray) Projection success flag and 2D image coordinates.
    """
    u_C = np.zeros((2, 1))
    if p_C[2] <= 0.0:
        return (False, u_C)
    inv_z = 1.0 / p_C[2]
    u_C = inv_z * (self.K @ p_C)[:2]
    if (u_C[0] >= self.width or u_C[1] >= self.height or u_C[0] < 0 or u_C[1] < 0):
      return (False, u_C)
    else:
      return (True, u_C)
  
def main():
  """Main function to test the Camera class."""
  # Create a general camera model
  D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
  K = np.array([205.46963709898583, 0.0, 320.5, 0.0, 205.46963709898583, 180.5, 0.0, 0.0, 1.0]).reshape(3, 3)
  Rect = np.array([[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]]).reshape(3, 3)
  P = np.array([205.46963709898583, 0.0, 320.5, -0.0, 0.0, 205.46963709898583, 180.5, 0.0, 0.0, 0.0, 1.0, 0.0]).reshape(3, 4)
  camera = Camera(frame_id='frame_cam00', width=640, height=360, camera_name='stereo_left_Flir_BFSU3', 
                  distortion_model='general_pinhole', 
                  K=K, D=D, Rect=Rect, P=P)
  print(camera)
  
  # Compute the camera frustum
  corners_C = camera.get_view_corners(min_depth=1.0, max_depth=10.0)
  print(corners_C[:, :4])
  print(corners_C[:, 4:])

  # Project a 3D point into the pixel plane
  p_C = np.array([5.0, 3.0, 20.0]).reshape(3, 1)
  print(camera.project(p_C))
  print(camera.project(p_C)[0])

  # Compute the AABB
  aabb_min = np.min(corners_C, axis=1)
  aabb_max = np.max(corners_C, axis=1)
  print('aabb_min: {}'.format(aabb_min))
  print('aabb_max: {}'.format(aabb_max))

if __name__ == '__main__':
    main()
