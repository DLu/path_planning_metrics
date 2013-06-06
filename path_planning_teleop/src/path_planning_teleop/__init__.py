#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_teleop')
import rospy
import rosbag
from geometry_msgs.msg import Twist, PolygonStamped, Point32, Pose2D
from math import pi, sin, cos
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

class TeleopPathRecorder:
    def __init__(self, record_rate=5):
        self.tf = tf.TransformListener()
        self.record_rate = record_rate
        self.base_frame = '/map'
        self.target_frame = '/base_footprint'
        self.recording = False
        self.cmd_data = []
        sub = rospy.Subscriber('/base_controller/command', Twist, self.command_cb)

        self.pub = rospy.Publisher('/goal_state', PolygonStamped, latch=True)

    def command_cb(self, msg):
        if not self.recording:
            if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
                return
            else:
                self.recording = True
        self.cmd_data.append( (rospy.Time.now(), '/base_controller/command', msg ))

    def goto(self, gx, gy, gt):

        # Create polygon
        p = PolygonStamped()
        p.header.frame_id = '/map'

        for i in range(4):
            angle = i * pi / 2 + gt + pi / 4
            x = gx + FOOTPRINT_RADIUS * cos(angle)
            y = gy + FOOTPRINT_RADIUS * sin(angle)
            p.polygon.points.append(Point32(x,y,0))

        # front point
        x = gx + FOOTPRINT_RADIUS * cos(gt)
        y = gy + FOOTPRINT_RADIUS * sin(gt)
        p.polygon.points.append(Point32(x,y,0))

        # Wait for robot to move (started in callback)
        rate = rospy.Rate(self.record_rate)
        while not self.recording and not rospy.is_shutdown():
            self.pub.publish(p)
            rate.sleep()

        data = []

        # While not there



        while not rospy.is_shutdown():
            self.pub.publish(p)

            t, pose = get_time_and_pose(self.tf, self.base_frame, self.target_frame)
            if pose is not None:
                self.data.append((t,"/robot_pose", pose))

                if close(pose, gx, gy, gt):
                    break
            rate.sleep()

        return data + self.cmd_data

