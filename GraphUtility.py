import math

# This class contains utility static methods related to 2D graphs in general
class GraphUtility:
    
    @staticmethod
    def calculate_manhattan_distance(x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)
    
    @staticmethod
    def calculate_squared_distance(x1, y1, x2, y2):
        return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
    
    @staticmethod
    def calculate_euclidean_distance(x1, y1, x2, y2):
        return math.sqrt(GraphUtility.calculate_squared_distance(x1, y1, x2, y2))

    @staticmethod
    def calculate_map_aligned_angle(north_aligned_angle, north_angle):
        if (north_angle > 180):
            north_angle -= 360
        map_aligned_angle = north_aligned_angle + north_angle
        if (map_aligned_angle < -180):
            map_aligned_angle += 360
        if (map_aligned_angle > 180):
            map_aligned_angle -= 360
        return map_aligned_angle
