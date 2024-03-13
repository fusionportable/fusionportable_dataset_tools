import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class Node:
    def __init__(self, frame_id):
      self.name = frame_id
      self.edges = []

    def __str__(self):
      return '{}'.format(self.name)

class Edge:
    def __init__(self, from_node, to_node, tf_matrix):
      self.from_node = from_node
      self.to_node = to_node
      self.tf_matrix = tf_matrix
    
    def __str__(self):
      return '{} - {}'.format(self.from_node, self.to_node)

class TFGraph:
    def __init__(self, is_print=False):
      self.nodes = {}
      self.is_print = is_print
    
    def add_node(self, frame_id):
      if frame_id not in self.nodes:
        self.nodes[frame_id] = Node(frame_id)
    
    def connect_nodes(self, from_node_name, to_node_name, tf_matrix):
      """
        from_node_name: parent name
        to_node_name: child name
        tf_matrix: 4x4 transformation matrix
      """
      if from_node_name in self.nodes and to_node_name in self.nodes:
        from_node = self.nodes[from_node_name]
        to_node = self.nodes[to_node_name]
        edge = Edge(from_node, to_node, tf_matrix)
        from_node.edges.append(edge)
        edge = Edge(to_node, from_node, np.linalg.inv(tf_matrix))
        to_node.edges.append(edge)
      else:
        print("One or both nodes do not exist.")
    
    def node_exists(self, frame_id):
      return (frame_id in self.nodes)
    
    def find_shortest_path(self, from_name, to_name):
      if from_name not in self.nodes or to_name not in self.nodes:
        return None, None

      visited = set()
      queue = deque([(from_name, ([np.eye(4, 4)], [from_name]))])
      while queue:
        current_node_name, path = queue.popleft()
        path_tf, path_node = path[0], path[1]
        if current_node_name == to_name:
          return path_tf, path_node
        if current_node_name not in visited:
          visited.add(current_node_name)
          for edge in self.nodes[current_node_name].edges:
            if edge.to_node.name not in visited:
              queue.append((edge.to_node.name, (path_tf + [edge.tf_matrix], path_node + [edge.to_node.name])))
      return None, None

    def get_relative_transform(self, frame_id, child_frame_id):
      path_tf, path_node = self.find_shortest_path(frame_id, child_frame_id)
      if path_tf is not None:
        if self.is_print:
          print(path_node)
        T = np.eye(4, 4)
        for tf in path_tf:
          T = T @ tf
        return T
      else:
        return None

    def visualize_graph(self):
      fig = plt.figure()
      ax = fig.add_subplot(111, projection='3d')
      
      base_origin = np.array([0, 0, 0])
      if 'body_imu' not in self.nodes:
        print('The base frame_id does not exist.')
        return

      x_axis = np.array([1, 0, 0])
      y_axis = np.array([0, 1, 0])
      z_axis = np.array([0, 0, 1])
      for frame_id, node in self.nodes.items():
        if frame_id == 'body_imu':
          # Frame
          ax.quiver(base_origin[0], base_origin[1], base_origin[2], x_axis[0], x_axis[1], x_axis[2], length=0.1, normalize=True, color=[1, 0, 0], arrow_length_ratio=0.3)
          ax.quiver(base_origin[0], base_origin[1], base_origin[2], y_axis[0], y_axis[1], y_axis[2], length=0.1, normalize=True, color=[0, 1, 0], arrow_length_ratio=0.3)
          ax.quiver(base_origin[0], base_origin[1], base_origin[2], z_axis[0], z_axis[1], z_axis[2], length=0.1, normalize=True, color=[0, 0, 1], arrow_length_ratio=0.3)
          ax.text(base_origin[0], base_origin[1], base_origin[2], frame_id, fontsize=10, color=[0, 0, 0])
        if frame_id != 'body_imu':
          tf_base_sensor = self.get_relative_transform('body_imu', frame_id)
          if tf_base_sensor is not None:
            origin = tf_base_sensor[:3, 3]
            rotation = tf_base_sensor[:3, :3]
            
            # Link
            ax.quiver(base_origin[0], base_origin[1], base_origin[2], origin[0], origin[1], origin[2], length=np.linalg.norm(origin), normalize=True, color=[0, 0, 0], arrow_length_ratio=0.05)
            ax.text(origin[0], origin[1], origin[2], frame_id, fontsize=10, color=[0, 0, 0])
            
            new_x_axis = rotation @ x_axis
            new_y_axis = rotation @ y_axis
            new_z_axis = rotation @ z_axis
            # Frame
            ax.quiver(origin[0], origin[1], origin[2], new_x_axis[0], new_x_axis[1], new_x_axis[2], length=0.1, normalize=True, color=[1, 0, 0], arrow_length_ratio=0.3)
            ax.quiver(origin[0], origin[1], origin[2], new_y_axis[0], new_y_axis[1], new_y_axis[2], length=0.1, normalize=True, color=[0, 1, 0], arrow_length_ratio=0.3)
            ax.quiver(origin[0], origin[1], origin[2], new_z_axis[0], new_z_axis[1], new_z_axis[2], length=0.1, normalize=True, color=[0, 0, 1], arrow_length_ratio=0.3)

      ax.set_xlim(-0.05, 0.35)
      ax.set_ylim(-0.5, 0.5)
      ax.set_zlim(-0.1, 0.1)
      ax.set_xlabel('X')
      ax.set_ylabel('Y')
      ax.set_zlabel('Z')
      ax.set_aspect('equal')
      ax.view_init(elev=45, azim=190, roll=0)
      ax.set_title('TF Graph')
      plt.show()

    def publish_graph(self):
      if 'body_imu' not in self.nodes:
        print('The base frame_id does not exist.')
        return

      import rospy
      import tf2_ros
      from geometry_msgs.msg import TransformStamped
      from eigen_conversion import convert_matrix_to_vec

      if not rospy.core.is_initialized():
        print('Initialize a ROS node to repeatedly publish TF messages.')
        rospy.init_node('visualize_tf_tree', anonymous=True)

      broadcaster = tf2_ros.StaticTransformBroadcaster()
      transform = TransformStamped()

      while True:
        if rospy.is_shutdown():
          break

        transform.header.stamp = rospy.Time.now()
        for frame_id, _ in self.nodes.items():
          if frame_id != 'body_imu':
            tf_base_sensor = self.get_relative_transform('body_imu', frame_id)
            if tf_base_sensor is not None:
              transform.header.frame_id = 'body_imu'
              transform.child_frame_id = frame_id
              
              vec_p, vec_q = convert_matrix_to_vec(tf_base_sensor)
              transform.transform.translation.x = vec_p[0]
              transform.transform.translation.y = vec_p[1]
              transform.transform.translation.z = vec_p[2]
              transform.transform.rotation.x = vec_q[0]
              transform.transform.rotation.y = vec_q[1]
              transform.transform.rotation.z = vec_q[2]
              transform.transform.rotation.w = vec_q[3]

              broadcaster.sendTransform(transform)
       

def TEST():
  import random
  graph = TFGraph()
  graph.add_node('body_imu')
  graph.add_node('ouster00')
  # graph.add_node('C')
  # graph.add_node('D')
  # graph.add_node('E')
  # graph.add_node('F')
  # graph.add_node('G')
  # graph.add_node('H')
  # graph.connect_nodes('A', 'B', np.eye(4, 4))
  # graph.connect_nodes('A', 'C', np.eye(4, 4))
  # graph.connect_nodes('A', 'F', np.eye(4, 4))
  # graph.connect_nodes('B', 'D', np.eye(4, 4))
  # graph.connect_nodes('B', 'E', np.eye(4, 4))
  # graph.connect_nodes('F', 'G', np.eye(4, 4))
  # graph.connect_nodes('B', 'H', np.eye(4, 4))
  # graph.connect_nodes('A', 'H', np.eye(4, 4))
  # print(graph.get_relative_transform('A', 'D'))
  # print(graph.get_relative_transform('A', 'G'))
  # print(graph.get_relative_transform('E', 'G'))
  # print(graph.get_relative_transform('E', 'C'))
  tf = np.eye(4, 4)
  tf[:3, 3] = np.array([0, 0, 0.2])
  graph.connect_nodes('body_imu', 'ouster00', tf)
  graph.visualize_graph()

if __name__ == "__main__":
  TEST()
  