import rospy
import rosbag
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D

def finished(state):
    return state==GoalStatus.SUCCEEDED or state==GoalStatus.ABORTED or state==GoalStatus.PREEMPTED

class MoveBaseClient:
    def __init__(self, record_rate=5):
        self.tf = tf.TransformListener()
        self.ac = SimpleActionClient('move_base', MoveBaseAction)
        self.record_rate = record_rate
        self.base_frame = '/map'
        self.target_frame = '/base_footprint'

        self.recording = False
        self.other_data = []

        while not self.ac.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown():
            rospy.loginfo('Waiting for server')
        if rospy.is_shutdown():
            exit(1)

    def addSubscription(self, topic, msg_type):
        sub = rospy.Subscriber(topic, msg_type, self.cb, topic)
        
    def cb(self, msg, topic):
        if not self.recording:
            return

        self.other_data.append( (rospy.Time.now(), topic, msg) )

    def getTransform(self):
        try:
            pos, q = self.tf.lookupTransform(self.base_frame, self.target_frame, rospy.Time(0))
            rpy = euler_from_quaternion(q)
        except:
            return None
        
        return Pose2D(pos[0], pos[1], rpy[2])

    def goto(self, loc):
        q = quaternion_from_euler(0, 0, loc[2])
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.base_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = loc[0]
        goal.target_pose.pose.position.y = loc[1]
        goal.target_pose.pose.orientation.w = q[3]
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]

        rate = rospy.Rate(self.record_rate)
        data = []
        self.other_data = []
        self.recording = True
        self.ac.send_goal(goal)

        while not finished(self.ac.get_state()):
            tf = self.getTransform()
            if tf is not None:
                t = rospy.Time.now()
                data.append((t,"/robot_pose", tf))
            rate.sleep()

        self.recording = False
        rospy.sleep(1)
        print len(self.other_data)
        return data + self.other_data

def bag(filename, data):
    b = rosbag.Bag(filename, 'w')
    for time, topic, msg in sorted(data):
        b.write(topic, msg, t=time)
    b.close()
