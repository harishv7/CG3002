import math

class Node:
    def __init__(self, node, x, y, node_name):
        self.id = node
        self.x = x
        self.y = y
        self.node_name = node_name
        self.adj = {}
    
    def calculate_dist_to(self, neighbour):
        return (neighbour.get_x() - self.x, neighbour.get_y() - self.y)
    
    def add_neighbour(self, neighbour):
        self.adj[neighbour] = self.calculate_dist_to(neighbour)
    
    def get_id(self):
        return self.id
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_degree(self):
        return len(self.adj)
    
    def get_neighbours(self):
        return self.adj
    
    def __repr__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adj]) + '\n'
    
    def __str__(self):
        return self.__repr__()
    