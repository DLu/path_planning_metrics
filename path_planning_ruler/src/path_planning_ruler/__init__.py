import rospy
import rosbag
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Path
from path_planning_simulation import *
from gazebo_msgs.msg import ModelStates
import std_msgs.msg

def finished(state):
    return state==GoalStatus.SUCCEEDED or state==GoalStatus.ABORTED or state==GoalStatus.PREEMPTED

def get_time_and_pose(tf, f1, f2):
    try:
        t = tf.getLatestCommonTime(f1, f2)
        pos, q = tf.lookupTransform(f1, f2, t)
    except:
        return None, None

    rpy = euler_from_quaternion(q)
    return t, Pose2D(pos[0], pos[1], rpy[2])

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

    def goto(self, loc, debug=True):
        self.goal = loc
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
        self.data = []
        self.other_data = []
        self.recording = True
        rospy.sleep(0.5)
        self.ac.send_goal(goal)

        if debug:
            timer = rospy.Timer(rospy.Duration(5), self.print_distance)

        while not finished(self.ac.get_state()):
            t, pose = get_time_and_pose(self.tf, self.base_frame, self.target_frame)
            if pose is not None:
                self.data.append((t,"/robot_pose", pose))
            else:
                print 'TF Error'
            rate.sleep()

        if debug:
            timer.shutdown()

        self.recording = False
        rospy.sleep(1)
        return self.data + self.other_data

    def print_distance(self, event=None):
        if len(self.data)==0:
            return
        pose = self.data[-1][2]
        dx = abs(self.goal[0]-pose.x)
        dy = abs(self.goal[1]-pose.y)
        dt = abs(self.goal[2]-pose.theta)
        rospy.loginfo( "dx: %.2f dy: %.2f dt: %d"%(dx, dy, int(dt*180/3.141)) )

def bag(filename, data):
    b = rosbag.Bag(filename, 'w')
    for time, topic, msg in sorted(data):
        b.write(topic, msg, t=time)
    b.close()

def run_empty_room_test(filename, start=(0,0,0), end=(0,0,0)):
    g = GazeboHelper()    
    g.set_state('pr2', get_pose(start[0], start[1], start[2]))
    rospy.sleep(1.0)

    t = rospy.Time.now()

    mb = MoveBaseClient()
    mb.addSubscription('/move_base_node/NavfnROS/plan', Path)
    mb.addSubscription('/move_base_node/DWAPlannerROS/local_plan', Path)
    mb.addSubscription('/collisions', std_msgs.msg.String)
    data = mb.goto(end)
    data.append( (t, '/start', Pose2D(start[0], start[1], start[2])) )
    data.append( (t, '/goal' , Pose2D(end[0],   end[1],   end[2])) )
    bag(filename, data)

def run_scenario(scenario, filename):
    rospy.set_param('/nav_experiments/scenario', scenario.scenario)
    g = GazeboHelper()
    scenario.spawn(g)
    scenario.reset(g)
    t = rospy.Time.now()
    endpoints = []
    endpoints.append( (t, '/start', scenario.start) )
    endpoints.append( (t, '/goal' , scenario.goal ) )

    mb = MoveBaseClient()

    #TODO: Load classes dynamically
    topics = rospy.get_param('/nav_experiments/topics', [])
    for topic in topics:
        if 'plan' in topic:
            mb.addSubscription(topic, Path)
        elif 'command' in topic:
            mb.addSubscription(topic, Twist)
        else:
            rospy.logerror("unknown type for", topic)
    mb.addSubscription('/collisions', std_msgs.msg.String)
    mb.addSubscription('/simulation_state', ModelStates)
    goal = (scenario.goal.x, scenario.goal.y, scenario.goal.theta)

    scenario.start_update_loop()
    data = mb.goto(goal)
    scenario.stop_update_loop()
    scenario.unspawn(g)
    bag(filename, endpoints + data)
