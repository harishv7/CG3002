import heapq

class Dijkstra:
    
    def __init__(self, graph, source):
        num_nodes = graph.get_num_nodes()
        self.dist_to = []  # distance of shortest s->v path
        self.edge_to = []  # last edge on shortest s->v path
        self.pq = [] # priority queue of vertices
        
        print("num of nodes in graph: " + str(num_nodes))
        
        for i in range(0, num_nodes):
            self.dist_to.append(math.inf)
            self.edge_to.append(None)
        print(self.dist_to)
        print(self.edge_to)
        self.dist_to[source] = 0.0
        
        # relax vertices in order of distance from s
        heapq.heappush(self.pq, (source, self.dist_to[source]))
        while(len(self.pq)):
            v = heapq.heappop(self.pq)[0]
            for neighbour in graph.get_node(v).get_neighbours():
                print(neighbour)
                self.__relax(graph.get_node(v), neighbour)
                
    def __relax(self, v, w):
            weight = v.calculate_dist_to(w)
            weight = weight[0] + weight[1]
            v_id = v.get_id()
            w_id = w.get_id()
            if (self.dist_to[w_id] > self.dist_to[v_id] + weight):
                self.dist_to[w_id] = self.dist_to[v_id] + weight
                self.edge_to[w_id] = (v, w)
            else:
                heapq.heappush(w, self.dist_to[w_id])
                
    def dist_to_node(self, v):
        print(self.dist_to[v])
        return self.dist_to[v]
                 