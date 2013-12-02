import roslib; roslib.load_manifest('path_planning_simulation')
import rospy
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel, GetWorldProperties
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D
import rosnode

SET_STATE_NAME = '/gazebo/set_model_state'
SPAWN_NAME = '/gazebo/spawn_gazebo_model'
DELETE_NAME = '/gazebo/delete_model'
WORLD_NAME = '/gazebo/get_world_properties'

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
    
def get_pose2d(scenario, key):
    pose = Pose2D()
    if key in scenario:
        value = scenario[key]
        if type(value)==type([]):
            pose.x = value[0]
            pose.y = value[1]
            pose.theta = value[2]
        else:
            print "Unknown type"
    return pose
    
def pose2d_to_pose(pose):
    return get_pose(pose.x, pose.y, pose.theta)
    
def service(name, s_type):
    rospy.wait_for_service(name)
    return rospy.ServiceProxy(name, s_type)    

class GazeboHelper:
    def __init__(self, quiet=False):
        self.state_proxy = service(SET_STATE_NAME, SetModelState)
        self.spawn_proxy = service(SPAWN_NAME, SpawnModel)
        self.delete_proxy = service(DELETE_NAME, DeleteModel)
        self.world_proxy = service(WORLD_NAME, GetWorldProperties)
        self.quiet = quiet

    def set_state(self, name, pose, twist=None, frame='/map'):
        state = ModelState()
        state.model_name = name
        state.reference_frame = frame
        state.pose = pose
        if twist is not None:
            state.twist = twist
        response = self.state_proxy(state)
        if not self.quiet:
            rospy.loginfo("SetState: %s | %s", "Success" if response.success else "Failure", response.status_message)
            
    def spawn_robot(self, name):
        description = rospy.get_param('/robot_description')
        self.spawn_model(name, description, get_pose(0,0,0))

    def spawn_model(self, name, xml, pose, frame='/map', namespace=""):
        response = self.spawn_proxy(name, xml, namespace, pose, frame)
        if not self.quiet:
            rospy.loginfo("SpawnModel: %s | %s", "Success" if response.success else "Failure", response.status_message)

    def delete_model(self, name):
        response = self.delete_proxy(name)
        if not self.quiet:
            rospy.loginfo("DeleteModel: %s | %s", "Success" if response.success else "Failure", response.status_message)
            
    def get_models(self):
        response = self.world_proxy()
        return response.model_names
        
    def spawn_robot_maybe(self, name='pr2'):
        models = self.get_models()
        if name in models:
            return
        else:
            self.spawn_robot(name)
            s, f = rosnode.kill_nodes(['default_controllers_spawner'])

