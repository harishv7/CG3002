from Node import Node
from GraphUtility import GraphUtility
import numpy as np

# This class represents a graph edge, its properties, and utility methods
class Edge:
    
    def __init__(self, node_one, node_two):
        self.node_one = node_one
        self.node_two = node_two
        self.manhattan_distance = node_one.calculate_manhattan_distance_to(node_two)
        self.euclidean_distance = node_one.calculate_euclidean_distance_to(node_two)
        
    def get_either(self):
        return self.node_one
    
    def get_other(self, node):
        if (node.get_id() == self.node_one.get_id()):
            return self.node_two
        else:
            return self.node_one
    
    def get_manhattan_distance(self):
        return self.manhattan_distance
    
    def get_euclidean_distance(self):
        return self.euclidean_distance
    
    # This method returns the normal length from a point to this edge in cartesian coordinate
    def get_normal_length_from_point(self, x, y):
        node_one = self.get_either()
        node_two = self.get_other(node_one)
        
        node_one_to_node_two_vector = np.array([
                node_two.get_x() - node_one.get_x(), 
                node_two.get_y() - node_one.get_y()
            ])
        node_one_to_point_vector = np.array([
                x - node_one.get_x(), 
                y - node_one.get_y()
            ])
        normal_length = (np.linalg.norm(np.cross(node_one_to_node_two_vector, node_one_to_point_vector)) / 
                         np.linalg.norm(node_one_to_node_two_vector))
        
        return normal_length
    
    def get_distance_from_first_node_to_normal_of_point(self, x, y):
        node_one = self.get_either()
        node_two = self.get_other(node_one)
        
        node_one_to_node_two_vector = np.array([
                node_two.get_x() - node_one.get_x(), 
                node_two.get_y() - node_one.get_y()
            ])
        node_one_to_point_vector = np.array([
                x - node_one.get_x(), 
                y - node_one.get_y()
            ])
        distance_from_first_node_to_normal = (np.dot(node_one_to_node_two_vector, node_one_to_point_vector) / 
                                              np.linalg.norm(node_one_to_node_two_vector))
        
        return distance_from_first_node_to_normal
    
    # This method returns the distance to the nearest node in this edge from a given point
    def get_distance_to_nearest_node_from_point(self, x, y):
        node_one = self.get_either()
        node_two = self.get_other(node_one)
        
        return min(
            GraphUtility.calculate_euclidean_distance(x, y, node_one.get_x(), node_one.get_y()), 
            GraphUtility.calculate_euclidean_distance(x, y, node_two.get_x(), node_two.get_y())
        )
    
    # This method returns the nearest node in this edge from a given point
    def get_nearest_node_from_point(self, x, y):
        node_one = self.get_either()
        node_two = self.get_other(node_one)
        
        dist_to_node_one = GraphUtility.calculate_euclidean_distance(x, y, node_one.get_x(), node_one.get_y())
        dist_to_node_two = GraphUtility.calculate_euclidean_distance(x, y, node_two.get_x(), node_two.get_y())
        
        if (dist_to_node_one < dist_to_node_two):
            return node_one
        else:
            return node_two
    
    def __repr__(self):
        return ("Edge consists of node " + str(self.node_one.get_id()) + " (" + self.node_one.get_name() + ")"
                " and node " + str(self.node_two.get_id()) + " (" + self.node_two.get_name() + ")"
                ", distance between the two nodes is " + str(self.euclidean_distance))
    
    def __str__(self):
        return self.__repr__()
