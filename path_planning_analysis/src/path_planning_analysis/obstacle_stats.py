from path_planning_analysis.math_util import *

def collisions(path):
    """METRIC"""
    return path.collisions

def distance_to_obstacle(path):
    """METRICS"""
    return {'minimum_distance_to_obstacle': min(path.object_distances),
            'average_distance_to_obstacle': average(path.object_distances)}

def front_distance(path):
    """METRIC"""
    return inv_average(path.front_distances)

def left_distance(path):
    """METRIC"""
    return inv_average(path.left_distances)

def right_distance(path):
    """METRIC"""
    return inv_average(path.right_distances)

