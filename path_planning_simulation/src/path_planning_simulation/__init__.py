import roslib; roslib.load_manifest('path_planning_simulation')
import rospy
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

SET_STATE_NAME = '/gazebo/set_model_state'
SPAWN_NAME = '/gazebo/spawn_urdf_model'

BOX = """
<robot name="simple_box">
  <link name="my_box">
    <inertial>
      <origin xyz="2 0 0" /> 
      <mass value="1.0" />
      <inertia  ixx="1.0" ixy="0.0"  ixz="0.0"  iyy="100.0"  iyz="0.0"  izz="1.0" />
    </inertial>
    <visual>
      <origin xyz="2 0 1"/>
      <geometry>
        <box size="1 1 2" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="2 0 1"/>
      <geometry>
        <box size="1 1 2" />
      </geometry>
    </collision>
  </link>
  <gazebo reference="my_box">
    <material>Gazebo/Blue</material>
  </gazebo>
</robot>
"""

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
        rospy.loginfo("S: %s | %s", "Success" if response.success else "Failure", response.status_message)



if __name__=='__main__':
    rospy.init_node('gazebo_helper')
    g = GazeboHelper()
    p = Pose()
    p.position.x = 5
    p.position.y = 2
    p.position.z = 0
    p.orientation.w = 1
    g.set_state('pr2', p)
    p.position.z = 2
    g.spawn_model('boxx', BOX, p)
