import requests
import json

r = requests.get("http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=COM1&Level=2")

g = Graph(r.json())

source = int(input("Enter source node id: "))
shortest_path = Dijkstra(g, source)
print()

destination = int(input("Enter destination node id: "))
print()

distance = shortest_path.dist_to_node(destination)

print("Shortest path from " + str(source) + " to " + str(destination) + ": ")
print(shortest_path.get_path(destination))
print()

print("Shortest distance is " + str(distance / 100) + "m")
print()

while (True):
    x = int(input("Enter current x-coordinate: "))
    y = int(input("Enter current y-coordinate: "))
    heading_angle = int(input("Enter angular difference from the North direction (in degrees): "))
    print()

    nearest_edge = g.get_nearest_edge_from_point(x, y)
    nearest_node = nearest_edge.get_nearest_node_from_point(x, y)
    rotate_direction = nearest_node.get_rotation_difference_from_point(x, y, heading_angle, g.get_north_angle())
    walk_distance = nearest_node.calculate_euclidean_distance_from_point(x, y)

    print("The nearest edge is: ")
    print(nearest_edge)
    print()

    print("The nearest node is: ")
    print(nearest_node)
    print()

    print("To get to the nearest node, you have to: ")
    print("Rotate " + str(rotate_direction) + " degrees and walk " + str(walk_distance / 100) + " meters")