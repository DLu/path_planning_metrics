#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_viz')
import rospy
import sys
from path_planning_viz import *

if len(sys.argv)>1 and sys.argv[1][0]!='-':
    fn = sys.argv[1]
else:
    fn = '/home/dlu/Desktop/path_data/core/09_personobstacle-hydroX-000.bag'

bag = CondensedBag(fn, '-g' in sys.argv, '-l' in sys.argv)
rospy.init_node('viz')
pub = Republisher(bag.paths.keys(), '-o' in sys.argv)
pub.jump = 4

if '-o' in sys.argv:
    rospy.set_param('/lps_tester/local_costmap/plugins', [])
    rospy.set_param('/lps_tester/local_costmap/footprint', '[[-0.335,-0.335],[-0.335,0.335],[0.335,0.335],[0.47,0],[0.335,-0.335]]')
    rospy.set_param('/lps_tester/local_costmap/global_frame', 'odom_combined')
    rospy.set_param('/lps_tester/local_costmap/robot_base_frame', 'base_footprint')
    rospy.set_param('/lps_tester/local_costmap/rolling_window', True)
    rospy.set_param('/lps_tester/base_local_planner', 'dwa_local_planner/DWAPlannerROS')
    rospy.set_param('/lps_tester/DWAPlannerROS/publish_cost_grid_pc', True)




obstacle = PolygonStamped()
obstacle.header.frame_id = '/map'
obstacle.polygon.points.append(Point32(1.75, .25, .1))
obstacle.polygon.points.append(Point32(1.75, -.25, .1))
obstacle.polygon.points.append(Point32(2.25, -.25, .1))
obstacle.polygon.points.append(Point32(2.25, .25, .1))
bag.obstacle = obstacle

footprint = PolygonStamped()
footprint.header.frame_id = '/base_footprint'
footprint.polygon.points.append(Point32(-0.325, -0.325, .1))
footprint.polygon.points.append(Point32(-0.325, 0.325, .1))
footprint.polygon.points.append(Point32(0.325, 0.325, .1))
footprint.polygon.points.append(Point32(0.46, 0.0, .1))
footprint.polygon.points.append(Point32(0.325, -0.325, .1))
bag.footprint = footprint

#pub.transforms['to_link'] = ('/odom_combined', '/base_footprint', (0,0,0), (0,0,0))
#pub.transforms['to_linxk'] = ('/odom_combined', '/odom', (0,0,0), (0,0,0))

pub.transforms['to_link'] = ('/map', '/odom_combined', (0,0,0), (0,0,0))
pub.transforms['to_linxk'] = ('/odom_combined', '/odom', (0,0,0), (0,0,0))


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

    
