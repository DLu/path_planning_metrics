#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_teleop')
import rospy
from path_planning_ruler import *
from path_planning_simulation import *
import actionlib
from move_base_msgs import *
from math import pi, sin, cos
from geometry_msgs.msg import Twist, PolygonStamped, Point32, Pose2D
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import tf
PIx2 = pi * 2

FOOTPRINT_RADIUS = .46
TRANSLATION_SLIP = .1
ROTATION_SLIP = .1

def a_dist(p1, p2):
    diff = p1 - p2
    mod_diff = abs(diff % PIx2)
    return min(mod_diff, PIx2-mod_diff)

def close(pose, x, y, theta):
    return abs(x-pose.x)<TRANSLATION_SLIP and abs(y-pose.y)<TRANSLATION_SLIP and a_dist(theta, pose.theta)


class TeleopNavClient:
    def __init__(self):
        rospy.set_param('/nav_experiments/topics', ['/base_controller/command'])
        self.tf = tf.TransformListener()
        self.target_frame = '/base_footprint'
        self.pub = rospy.Publisher('/goal_state', PolygonStamped, latch=True)
        self._as = actionlib.SimpleActionServer('/move_base', MoveBaseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # Create polygon
        p = PolygonStamped()
        p.header.frame_id = goal.target_pose.header.frame_id
        pose = goal.target_pose.pose
        gx = pose.position.x
        gy = pose.position.y
        rpy = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        gt = rpy[2]

        for i in range(4):
            angle = i * pi / 2 + gt + pi / 4
            x = gx + FOOTPRINT_RADIUS * cos(angle)
            y = gy + FOOTPRINT_RADIUS * sin(angle)
            p.polygon.points.append(Point32(x,y,0))

        # front point
        x = gx + FOOTPRINT_RADIUS * cos(gt)
        y = gy + FOOTPRINT_RADIUS * sin(gt)
        p.polygon.points.append(Point32(x,y,0))

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                return

            self.pub.publish(p)

            t, pose = get_time_and_pose(self.tf, p.header.frame_id, self.target_frame)
            if pose is not None:
                if close(pose, gx, gy, gt):
                    break
            rate.sleep()

        self._as.set_succeeded()

        p.polygon.points = []
        N = 10
        for i in range(N):
            angle = i * PIx2 / N
            x = gx + FOOTPRINT_RADIUS * cos(angle)
            y = gy + FOOTPRINT_RADIUS * sin(angle)
            p.polygon.points.append(Point32(x,y,0))

        for i in range(5):
            self.pub.publish(p)
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('teleop_nav_client')
    tnc = TeleopNavClient()
    rospy.spin()
