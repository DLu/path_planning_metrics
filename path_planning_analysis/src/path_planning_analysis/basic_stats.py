from path_planning_analysis.math_util import *

def time(path):
    """METRIC"""
    return path.t[-1]-path.t[0]


def completed(path):
    """METRIC"""
    dist = path.get_distance_to_goal()
    angle = path.get_angle_to_goal()
    return 1.0 if dist < 0.2 and angle < .2 else 0.0

def translate_efficiency(path):
    """METRIC"""
    D = sum(map(abs, path.displacement))
    D0 = dist(path.poses[0], path.poses[-1])
    return inverse_scale(D-D0)
    
def rotate_efficiency(path):
    """METRIC"""
    p0 = None
    A = 0.0
    for pose in path.poses:
        if not p0:
            p0 = pose
        A += abs(a_dist(p0, pose))
        p0 = pose
    A0 = path.get_angle_to_goal()
    return inverse_scale(A-A0)

def face_direction_of_travel(path, mag_limit=0.1):
    """METRIC"""
    angles = [pose.theta for pose in path.poses]
    products = []
    for angle1, angle2, mag in zip(angles, path.headings, path.speeds):
        if mag > mag_limit:
            products.append( a_dist_helper(angle1, angle2) )
    m = average(products)
    return inverse_scale(m)

def curvature(path):
    """METRIC"""
    return average(path.curvatures)



