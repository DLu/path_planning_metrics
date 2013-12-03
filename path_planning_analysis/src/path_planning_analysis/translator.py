from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def get_pose_from_scenario(name, obj):
    p = Pose()
    p.position.x = obj.xyz[0]
    p.position.y = obj.xyz[1]
    p.position.z = obj.xyz[2]
    q = quaternion_from_euler(obj.rpy[0], obj.rpy[1], obj.rpy[2])
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p
    
def get_pose_from_state(name, state):
    i = state.name.index(name)
    return state.pose[i]

