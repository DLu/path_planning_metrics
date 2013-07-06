from path_planning_analysis.math_util import *

def collisions(path):
    """METRICS"""
    return path.collisions

def minimum_distance_to_obstacle(path):
    """METRIC"""
    return min(path.object_distances)

def average_distance_to_obstacle(path):
    """METRIC"""
    return average(path.object_distances)

def front_distance(path):
    """METRIC"""
    return inv_average(path.front_distances)

def left_distance(path):
    """METRIC"""
    return inv_average(path.left_distances)

def right_distance(path):
    """METRIC"""
    return inv_average(path.right_distances)

