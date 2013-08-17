#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_viz')
import rospy
import sys
from path_planning_viz import *

if len(sys.argv)>1 and sys.argv[1][0]!='-':
    fn = sys.argv[1]
else:
    fn = '/home/dlu/Desktop/path_data/core/09_personobstacle-hydro_collect-000.bag'

bag = CondensedBag(fn, '-g' in sys.argv, '-l' in sys.argv)
rospy.init_node('viz')
pub = Republisher(bag.paths.keys())
pub.jump = 4

obstacle = PolygonStamped()
obstacle.header.frame_id = '/map'
obstacle.polygon.points.append(Point32(1.75, .25, .1))
obstacle.polygon.points.append(Point32(1.75, -.25, .1))
obstacle.polygon.points.append(Point32(2.25, -.25, .1))
obstacle.polygon.points.append(Point32(2.25, .25, .1))
bag.obstacle = obstacle

footprint = PolygonStamped()
footprint.header.frame_id = '/odom_combined'
footprint.polygon.points.append(Point32(-0.325, -0.325, .1))
footprint.polygon.points.append(Point32(-0.325, 0.325, .1))
footprint.polygon.points.append(Point32(0.325, 0.325, .1))
footprint.polygon.points.append(Point32(0.46, 0.0, .1))
footprint.polygon.points.append(Point32(0.325, -0.325, .1))
bag.footprint = footprint

t = 0
play = False

while t < len(bag.poses) and not rospy.is_shutdown():
    pub.publish(bag, t)
    t+=1

    if not play:
        x = raw_input("?")
    else:
        rospy.sleep(.1)

    if 'x' in x:
        break
    elif 'p' in x:
        play = True

    
