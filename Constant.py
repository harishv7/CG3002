import math

# This class contains all the constants used in this project, ranging from mathematical to predefined constants
class Constant:
    
    EPS = 0.001
    PI = math.acos(-1)
    EAST_TO_NORTH_ANGLE = 90
    DEGREE_TO_RADIAN_RATIO = 180 / PI
    RADIAN_TO_DEGREE_RATIO = PI / 180
    INF = 1000000007
    AVERAGE_STEP_DISTANCE = 50
    PROMPT_DELAY = 4
    DISTANCE_FROM_NODE_THRESHOLD = 50
    DISTANCE_FROM_EDGE_THRESHOLD = 200
