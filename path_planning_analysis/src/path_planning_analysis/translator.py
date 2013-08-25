from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def get_pose_from_scenario(name, properties):
    xyz = properties.get('xyz', [0,0,0])
    rpy = properties.get('rpy', [0,0,0])
    p = Pose()
    p.position.x = xyz[0]
    p.position.y = xyz[1]
    p.position.z = xyz[2]
    q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p, False
    
def get_pose_from_state(name, state):
    i = state.name.index(name)
    return state.pose[i]

def get_simulated_pose(name, state, scenario_objects):
    properties = scenario_objects[name]
    if 'movement' in properties:
        return get_pose_from_scenario(name, properties)
    else:
        return get_pose_from_state(name, state)

