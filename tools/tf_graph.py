import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from collections import deque

class Node:
    def __init__(self, frame_id):
      self.name = frame_id
      self.edges = []

    def __str__(self):
      return '{}'.format(self.name)

class Edge:
    def __init__(self, from_node, to_node, transformation_matrix):
      self.from_node = from_node
      self.to_node = to_node
      self.transformation_matrix = transformation_matrix
    
    def __str__(self):
      return '{} - {}'.format(self.from_node, self.to_node)

class TFGraph:
    def __init__(self):
      self.nodes = {}
      self.G = nx.Graph()  # Initialize a new Graph object
    
    def add_node(self, frame_id):
      if frame_id not in self.nodes:
        self.nodes[frame_id] = Node(frame_id)
        self.G.add_node(frame_id)  # Add node to the networkx graph
    
    def connect_nodes(self, from_node_name, to_node_name, transformation_matrix):
      """
        from_node_name: parent name
        to_node_name: child name
        transformation_matrix: 4x4 transformation matrix
      """
      if from_node_name in self.nodes and to_node_name in self.nodes:
        from_node = self.nodes[from_node_name]
        to_node = self.nodes[to_node_name]
        edge = Edge(from_node, to_node, transformation_matrix)
        from_node.edges.append(edge)
        edge = Edge(to_node, from_node, np.linalg.inv(transformation_matrix))
        to_node.edges.append(edge)
        self.G.add_edge(from_node_name, to_node_name, weight=1.0)
      else:
        print("One or both nodes do not exist.")
    
    def node_exists(self, frame_id):
      return (frame_id in self.nodes)
    
    def find_shortest_path(self, from_name, to_name):
      if from_name not in self.nodes or to_name not in self.nodes:
        return None
      visited = set()
      queue = deque([(from_name, [np.eye(4, 4)])])
      while queue:
        current_node_name, path_tf = queue.popleft()
        if current_node_name == to_name:
          return path_tf
        if current_node_name not in visited:
          # print('Visit: {}'.format(current_node_name))
          visited.add(current_node_name)
          for edge in self.nodes[current_node_name].edges:
            if edge.to_node.name not in visited:
              queue.append((edge.to_node.name, path_tf + [edge.transformation_matrix]))
      return None

    def get_relative_transform(self, frame_id, child_frame_id):
      path_tf = self.find_shortest_path(frame_id, child_frame_id)
      if path_tf is not None:
        T = np.eye(4, 4)
        for tf in path_tf:
          T = T @ tf
        return T
      else:
        return None

    def visualize_graph(self):
        pos = nx.spring_layout(self.G)  # Positions for all nodes
        nx.draw(self.G, pos, with_labels=True, node_color='skyblue', node_size=700, edge_color='k', linewidths=1, font_size=15, )
        edge_labels = dict([((u, v,), d['weight']) for u, v, d in self.G.edges(data=True)])
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels)
        plt.show()

if __name__ == "__main__":
  import random

  graph = TFGraph()
  graph.add_node('A')
  graph.add_node('B')
  graph.add_node('C')
  graph.add_node('D')
  graph.add_node('E')
  graph.add_node('F')
  graph.add_node('G')
  graph.add_node('H')
  graph.connect_nodes('A', 'B', np.eye(4, 4))
  graph.connect_nodes('A', 'C', np.eye(4, 4))
  graph.connect_nodes('A', 'F', np.eye(4, 4))
  graph.connect_nodes('B', 'D', np.eye(4, 4))
  graph.connect_nodes('B', 'E', np.eye(4, 4))
  graph.connect_nodes('F', 'G', np.eye(4, 4))
  graph.connect_nodes('B', 'H', np.eye(4, 4))
  graph.connect_nodes('A', 'H', np.eye(4, 4))
  print(graph.get_relative_transform('A', 'D'))
  print(graph.get_relative_transform('A', 'G'))
  print(graph.get_relative_transform('E', 'G'))
  print(graph.get_relative_transform('E', 'C'))  
  graph.visualize_graph()

