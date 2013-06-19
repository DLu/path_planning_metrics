#!/usr/bin/python

import roslib; roslib.load_manifest('dumb_planner')
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from math import pi, sin, cos
from geometry_msgs.msg import Twist, PolygonStamped, Point32, Pose2D
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import copysign, sqrt

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

def orientation_to_euler(orientation):
    return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

class DumbNavClient:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/base_controller/command', Twist)
        self.tlim = rospy.get_param('translation_speed_limit', 1.2)
        self.rlim = rospy.get_param('rotation_speed_limit', 1.4)
        rospy.set_param('/nav_experiments/topics', ['/base_controller/command'])
        self.tf = tf.TransformListener()
        self.target_frame = '/base_footprint'
        self._as = actionlib.SimpleActionServer('/move_base', MoveBaseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            goal.target_pose.header.stamp = rospy.Time(0)
            relative = self.tf.transformPose(self.target_frame, goal.target_pose)
            dx = relative.pose.position.x
            dy = relative.pose.position.y
            rot = orientation_to_euler(relative.pose.orientation)
            dz = rot[2]

            d = sqrt(dx*dx+dy*dy) 
            if d<TRANSLATION_SLIP/2:
                break

            self.publish_command( dx, dy, dz )
            r.sleep()
        self._as.set_succeeded()

    def publish_command(self, dx, dy, dz):
        cmd = Twist()
        cmd.linear.x = dx
        cmd.linear.y = dy
        cmd.angular.z = dz

        if abs(cmd.linear.x) > self.tlim:
            cmd.linear.x = copysign(self.tlim, cmd.linear.x)
        if abs(cmd.linear.y) > self.tlim:
            cmd.linear.y = copysign(self.tlim, cmd.linear.y)
        if abs(cmd.angular.z) > self.rlim:
            cmd.angular.z = copysign(self.rlim, cmd.angular.z)
        self.cmd_pub.publish(cmd)


if __name__=='__main__':
    rospy.init_node('dumb_nav_client')
    tnc = DumbNavClient()
    rospy.spin()
