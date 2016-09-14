class Dijkstra:
    
    def __init__(self, graph, source):
        self.dist_to = [] # distance of shortest s->v path
        self.edge_to = [] # last edge on shortest s->v path
        self.pq = [] # priority queue of vertices