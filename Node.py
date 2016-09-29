import math

# This class represents a graph node, its properties, and utility methods
class Node:
    
    def __init__(self, node, x, y, name):
        self.id = node
        self.x = x
        self.y = y
        self.name = name
        self.adj = {}
    
    def calculate_vector_to(self, neighbour):
        return (neighbour.get_x() - self.x, neighbour.get_y() - self.y)
    
    def calculate_manhattan_distance_to(self, neighbour):
        return GraphUtility.calculate_manhattan_distance(self.x, self.y, neighbour.get_x(), neighbour.get_y())
    
    def calculate_euclidean_distance_to(self, neighbour):
        return GraphUtility.calculate_euclidean_distance(self.x, self.y, neighbour.get_x(), neighbour.get_y())
    
    def calculate_euclidean_distance_from_point(self, x, y):
        return GraphUtility.calculate_euclidean_distance(x, y, self.x, self.y)
    
    def calculate_euclidean_distance_from_node(self, source_node):
        return self.calculate_euclidean_distance_from_point(source_node.get_x(), source_node.get_y())
    
    def add_neighbour(self, neighbour):
        self.adj[neighbour] = self.calculate_vector_to(neighbour)
    
    def get_id(self):
        return self.id
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_name(self):
        return self.name
    
    def get_degree(self):
        return len(self.adj)
    
    def get_neighbours(self):
        return self.adj
    
    # This method returns the rotation difference from a given point with a given heading angle (in degrees relative to North)
    def get_rotation_difference_from_point(self, x, y, heading_angle, north_angle):
        target_rotation_relative_to_east = math.atan2(self.y - y, self.x - x) * -Constant.DEGREE_TO_RADIAN_RATIO
        target_rotation_relative_to_north = (target_rotation_relative_to_east + Constant.EAST_TO_NORTH_ANGLE - 
                                             heading_angle - north_angle)
        # Normalize the target rotation angle to (-180, 180]
        if (target_rotation_relative_to_north <= -180):
            target_rotation_relative_to_north += 360
        if (target_rotation_relative_to_north > 180):
            target_rotation_relative_to_north -= 360
            
        return target_rotation_relative_to_north
    
    # This method returns the rotation difference from a given node with a given heading angle (in degrees relative to North)
    def get_rotation_difference_from_node(self, source_node, heading_angle, north_angle):
        return self.get_rotation_difference_from_point(source_node.get_x(), source_node.get_y(), 
                                                       heading_angle, north_angle)
    
    def __repr__(self):
        return ("Node id: " + str(self.id) + ", "
                "Node name: " + self.name + ", "
                "Adjacent to: " + str([x.id for x in self.adj]))
    
    def __str__(self):
        return self.__repr__()