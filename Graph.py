import math
import numpy as np

# This class represents an undirected (bidirectional) graph with adjacency list approach in storing the neighbouring nodes
class Graph:
    
    def __init__(self, json = None):
        self.node_map = {}
        self.num_nodes = 0
        self.edges = []
        self.north_degree = None
        
        if (json is not None):
            self.north_degree = json["info"]["northAt"]
            for node in json["map"]:
                node_id = int(node["nodeId"])
                x = int(node["x"])
                y = int(node["y"])
                node_name = node["nodeName"]
                link_to = node["linkTo"].split(", ")

                self.add_node(node_id, x, y, node_name)

                for i in range(len(link_to)):
                    self.add_edge(node_id, int(link_to[i]))
                    
            print("Graph is created successfully from the given JSON\n")
            print(self)
    
    def add_node(self, node_id, x, y, node_name):
        self.num_nodes = self.num_nodes + 1
        new_node = Node(node_id, x, y, node_name)
        self.node_map[node_id] = new_node
        return new_node
    
    def is_valid_node_id(self, node_id):
        if (node_id not in self.node_map):
            return False
        return True
    
    def add_edge(self, node_one_id, node_two_id):
        if (self.is_valid_node_id(node_one_id) and self.is_valid_node_id(node_two_id)):
            self.node_map[node_one_id].add_neighbour(self.node_map[node_two_id])
            self.node_map[node_two_id].add_neighbour(self.node_map[node_one_id])
            # Store the newly created edge
            new_edge = Edge(
                self.node_map[node_one_id], 
                self.node_map[node_two_id]
            )
            self.edges.append(new_edge)
    
    def get_node(self, node):
        return self.node_map[node]
    
    def get_degree(self, node):
        return self.node_map[node].get_degree()
    
    def get_num_nodes(self):
        return self.num_nodes
    
    def get_nodes(self):
        return self.node_map.values()
    
    # This method returns the nearest edge from a point in cartesian coordinate
    def get_nearest_edge_from_point(self, x, y):
        nearest_edge = None
        shortest_normal_length = math.inf
        shortest_distance_to_nearest_node_in_edge = math.inf
        
        for edge in self.edges:
            normal_length = edge.get_normal_length_from_point(x, y)
            # Case 1: If normal length is longer than the shortest so far, ignore this edge
            if (normal_length > shortest_normal_length):
                continue
            # Case 2: If normal length is shortest so far, check further
            distance_from_first_node_to_normal_of_point = edge.get_distance_from_first_node_to_normal_of_point(x, y)
            # Case 1.1: If projection point lies inside the edge, update nearest edge
            if (distance_from_first_node_to_normal_of_point >= 0 and 
                distance_from_first_node_to_normal_of_point <= edge.get_euclidean_distance()):
                # Update nearest edge
                nearest_edge = edge
                shortest_normal_length = normal_length
                shortest_distance_to_nearest_node_in_edge = edge.get_distance_to_nearest_node_from_point(x, y)
            # Case 1.2: If projection point lies outside the edge, check further
            else:
                distance_to_nearest_node_in_edge = edge.get_distance_to_nearest_node_from_point(x, y)
                # Case 1.2.1: If distance to nearest node is smaller, update nearest edge
                if (distance_to_nearest_node_in_edge < shortest_distance_to_nearest_node_in_edge):
                    # Update nearest edge
                    nearest_edge = edge
                    shortest_distance_to_nearest_node_in_edge = distance_to_nearest_node_in_edge
                    
        return nearest_edge
    
    def __repr__(self):
        result = "Nodes in this graph:\n"
        for node in self.node_map.values():
            result += str(node) + "\n"
        result += "North is at " + self.north_degree + " degrees"
        return result
    
    def __str__(self):
        return self.__repr__()