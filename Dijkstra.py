import math
from Constant import Constant
import heapq

# This class implements the Dijkstra algorithm and various utility methods related to it
class Dijkstra:
    
    def __init__(self, graph, source):
        num_nodes = graph.get_num_nodes()
        self.source_id = source
        self.dist_to = {}  # distance of shortest s->v path
        self.prev = {}  # previous node on shortest s->v path
        self.pq = [] # priority queue of vertices
        
        nodes = graph.get_nodes()
        
        for node in nodes:
            node_id = node.get_id()
            self.dist_to[node_id] = Constant.INF
            self.prev[node_id] = None
        self.dist_to[source] = 0.0
        
        # relax vertices in order of distance from s
        heapq.heappush(self.pq, (source, self.dist_to[source]))
        while(len(self.pq)):
            v = heapq.heappop(self.pq)[0]
            for neighbour in graph.get_node(v).get_neighbours():
                weight = graph.get_node(v).calculate_manhattan_distance_to(neighbour)
                self.__relax(v, neighbour.get_id(), weight)
                
    def __relax(self, v_id, w_id, weight):            
        # get the index, since we start from 1, we need to minus one
        if (self.dist_to[w_id] > self.dist_to[v_id] + weight):
            self.dist_to[w_id] = self.dist_to[v_id] + weight
            self.prev[w_id] = v_id
            heapq.heappush(self.pq, (w_id, self.dist_to[w_id]))
                
    def dist_to_node(self, v):
        return self.dist_to[v]
    
    def get_path(self, target_id):
        path = []
        current_node_id = target_id
        while (current_node_id != self.source_id):
            path.append(current_node_id)
            current_node_id = self.prev[current_node_id]
        path.append(current_node_id)
        return path[::-1]
