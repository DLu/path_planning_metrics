#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from nav_msgs.msg import Path
from path_planning_ruler import *

if __name__=='__main__':
    rospy.init_node('test_script')
    mb = MoveBaseClient()
    mb.addSubscription('/move_base_node/NavfnROS/plan', Path)
    mb.addSubscription('/move_base_node/DWAPlannerROS/local_plan', Path)
    data = mb.goto([3,0,0])
    bag("path.bag", data)


