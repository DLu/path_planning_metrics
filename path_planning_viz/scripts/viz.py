#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_viz')
import rosbag
import rospy
import sys
#from easy_markers import Generator
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def getPose(p):
    pose = PoseStamped()
    if p:
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/map'
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        quat = quaternion_from_euler(0,0,p.theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

    return pose


if len(sys.argv)>1:
    fn = sys.argv[1]
else:
    fn = '/home/dlu/Desktop/path_data/core/09_personobstacle-hydro_collect-000.bag'
bag = rosbag.Bag(fn)

#costmaps
#global path
#local path

start = None
goal = None
poses = []

for topic, msg, t in bag.read_messages():
    if topic == '/start':
        start = msg
    elif topic == '/goal':
        goal = msg
    elif topic == '/robot_pose':
        poses.append(msg)

rospy.init_node('s')
spub = rospy.Publisher('/start', PoseStamped)
gpub = rospy.Publisher('/goal', PoseStamped)
rpub = rospy.Publisher('/robot', PoseStamped)

t = 0

while t < len(poses):
    print "%d/%d"%(t,len(poses))
    spub.publish( getPose(start) )
    gpub.publish( getPose(goal) )

    rpub.publish( getPose( poses[t] ) )
    t+=1

    x = raw_input("?")
    if 'q' in x:
        break

    
