


def distances_to_goal(path):
    """ARRAY_MET"""
    a = []
    for i in range(path.get_num_poses()):
        a.append(path.get_distance_to_goal(i))
    return a
    
def angles_to_goal(path):
    """ARRAY_MET"""
    a = []
    for i in range(path.get_num_poses()):
        a.append(path.get_angle_to_goal(i))
    return a
