import requests
import json

r = requests.get("http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=COM1&Level=1")

body = r.json()

g = Graph()

for node in body["map"]:
    node_id = int(node['nodeId'])
    x = int(node['x'])
    y = int(node['y'])
    node_name = node['nodeName']
    link_to = node['linkTo'].split(', ')
    
    g.add_node(node_id, x, y, node_name)
    
    for i in range(len(link_to)):
        g.add_edge(node_id, int(link_to[i]))

print(g)
shortest_path = Dijkstra(g, 1)
shortest_path.dist_to_node(2)
