import roslib; roslib.load_manifest('path_planning_simulation')
import rospy
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

SET_STATE_NAME = '/gazebo/set_model_state'
SPAWN_NAME = '/gazebo/spawn_urdf_model'

def get_pose(x, y, theta):
    p = Pose()
    p.position.x = x
    p.position.y = y
    q = quaternion_from_euler(0, 0, theta)
    p.orientation.w = q[3]
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    return p
    

class GazeboHelper:
    def __init__(self):
        rospy.wait_for_service(SET_STATE_NAME)
        self.state_proxy = rospy.ServiceProxy(SET_STATE_NAME, SetModelState)
        rospy.wait_for_service(SPAWN_NAME)
        self.spawn_proxy = rospy.ServiceProxy(SPAWN_NAME, SpawnModel)

    def set_state(self, name, pose, twist=None, frame='/map'):
        state = ModelState()
        state.model_name = name
        state.reference_frame = frame
        state.pose = pose
        if twist is not None:
            state.twist = twist
        response = self.state_proxy(state)
        rospy.loginfo("SetState: %s | %s", "Success" if response.success else "Failure", response.status_message)

    def spawn_model(self, name, xml, pose, frame='/map', namespace=""):
        response = self.spawn_proxy(name, xml, namespace, pose, frame)
        rospy.loginfo("SpawnModel: %s | %s", "Success" if response.success else "Failure", response.status_message)



if __name__=='__main__':
    rospy.init_node('gazebo_helper')
    g = GazeboHelper()
    g.set_state('pr2', get_pose(0,0,0))
    #p.position.z = 2
    #g.spawn_model('boxx', BOX, p)
