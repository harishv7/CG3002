import requests
import json

jsonRequest = requests.get("http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=COM1&Level=2")

graph = Graph(jsonRequest.json())

source = int(input("Enter source node id: "))
shortest_path = Dijkstra(graph, source)
print()

destination = int(input("Enter destination node id: "))
print()

distance = shortest_path.dist_to_node(destination)
path = shortest_path.get_path(destination)

print("Shortest path from " + str(source) + " to " + str(destination) + ": ")
print(path)
print()

print("Shortest distance is " + str(distance / 100) + "m")
print()

isReady = str(input("Do you want to start your journey? (y / n): "))
if (isReady == "y"):
    heading_angle = float(input("Enter angular difference from the North direction (in degrees): "))
    
    current_node_id_in_path = 0
    current_node = graph.get_node(path[current_node_id_in_path])
    
    next_node_id_in_path = 1
    next_node = graph.get_node(path[next_node_id_in_path])
    
    rotate_direction = next_node.get_rotation_difference_from_node(current_node, heading_angle, graph.get_north_angle())
    walk_distance = next_node.calculate_euclidean_distance_from_node(current_node)
    
    print()
    print("To get to the next node (" + str(next_node) + "), you have to: ")
    print("Rotate " + str(rotate_direction) + " degrees and walk " + str(walk_distance / 100) + " meters")
    
    while (True):
        has_reached_next_node = str(input("Has player reached next node? (y / n): "))
        if (has_reached_next_node == "y"):
            heading_angle += rotate_direction
            
            if (path[next_node_id_in_path] == destination):
                break
            elif (is_rerouting):
                is_rerouting = False
                current_node = next_node
                # Recalculate shortest path
                shortest_path = Dijkstra(graph, current_node.get_id())
                distance = shortest_path.dist_to_node(destination)
                path = shortest_path.get_path(destination)
                
                print("Shortest path from " + str(current_node) + " to " + str(destination) + ": ")
                print(path)
                print()

                print("Shortest distance is " + str(distance / 100) + "m")
                print()
                
                current_node_id_in_path = 0
                current_node = graph.get_node(path[current_node_id_in_path])

                next_node_id_in_path = 1
                next_node = graph.get_node(path[next_node_id_in_path])
                
                rotate_direction = next_node.get_rotation_difference_from_node(current_node, heading_angle, graph.get_north_angle())
                walk_distance = next_node.calculate_euclidean_distance_from_node(current_node)
            else:
                current_node_id_in_path = next_node_id_in_path
                current_node = next_node
                
                next_node_id_in_path += 1
                next_node = graph.get_node(path[next_node_id_in_path])
                
                rotate_direction = next_node.get_rotation_difference_from_node(current_node, heading_angle, graph.get_north_angle())
                walk_distance = next_node.calculate_euclidean_distance_from_node(current_node)
        else:
            x = int(input("Enter current x-coordinate: "))
            y = int(input("Enter current y-coordinate: "))
            heading_angle = int(input("Enter angular difference from the North direction (in degrees): "))
            
            nearest_edge = graph.get_nearest_edge_from_point(x, y)
            nearest_node = nearest_edge.get_nearest_node_from_point(x, y)
            # If still within the designated path
            if ((nearest_edge.get_either() == current_node and nearest_edge.get_other(current_node) == next_node) or 
                (nearest_edge.get_either() == next_node and nearest_edge.get_other(next_node) == current_node)):
                rotate_direction = next_node.get_rotation_difference_from_point(x, y, heading_angle, graph.get_north_angle())
                walk_distance = next_node.calculate_euclidean_distance_from_point(x, y)
            else:
                rotate_direction = nearest_node.get_rotation_difference_from_point(x, y, heading_angle, graph.get_north_angle())
                walk_distance = nearest_node.calculate_euclidean_distance_from_point(x, y)
                # Flag for rerouting
                is_rerouting = True
                next_node = nearest_node
        
        print()
        print("To get to the next node (" + str(next_node) + "), you have to: ")
        print("Rotate " + str(rotate_direction) + " degrees and walk " + str(walk_distance / 100) + " meters")
    
    print()
    print("Congratulations, you have reached your destination!")