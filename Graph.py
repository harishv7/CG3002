# This class implements an undirected graph
class Graph:
    def __init__(self):
        self.node_map = {}
        self.num_nodes = 0
    
    def add_node(self, node_id, x, y, node_name):
        self.num_nodes = self.num_nodes + 1
        new_node = Node(node_id, x, y, node_name)
        self.node_map[node_id] = new_node
        return new_node
    
    def is_valid_node_id(self, node_id):
        if (node_id not in self.node_map):
            return False
        return True
    
    def add_edge(self, node_one, node_two):
        if (self.is_valid_node_id(node_one) and self.is_valid_node_id(node_two)):
            self.node_map[node_one].add_neighbour(self.node_map[node_two])
            self.node_map[node_two].add_neighbour(self.node_map[node_one])
    
    def get_node(self, node):
        return self.node_map[node]
    
    def get_degree(self, node):
        return self.node_map[node].get_degree()
    
    def get_num_nodes(self):
        return self.num_nodes
    
    def get_nodes(self):
        return self.node_map.values()
    
    def __repr__(self):
        result = ''
        for node in self.node_map.values():
            result += str(node)
        return result