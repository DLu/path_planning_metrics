from path_planning_analysis.math_util import *

def distance_to_people(path):
    """METRICS"""
    return {'minimum_distance_to_person': min(path.people_distances),
            'average_distance_to_person': average(path.people_distances)}


