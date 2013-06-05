#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from nav_msgs.msg import Path
from path_planning_ruler import *
import sys
from path_planning_simulation import *

if __name__=='__main__':
    rospy.init_node('test_script')
    g = GazeboHelper()    
    g.set_state('pr2', get_pose(0,0,0))
    rospy.sleep(1.0)

    mb = MoveBaseClient()
    mb.addSubscription('/move_base_node/NavfnROS/plan', Path)
    mb.addSubscription('/move_base_node/DWAPlannerROS/local_plan', Path)
    if len(sys.argv)<=1:
        print "Error no bag specified"
        exit(1)
    filename = sys.argv[1]
    if '.bag' not in filename:
        print "Check args:", filename
        exit(1)
    x = 0
    y = 0
    theta = 0
    if len(sys.argv)>2:
        x = float(sys.argv[2])
        if len(sys.argv)>3:
            y = float(sys.argv[3])
            if len(sys.argv)>4:
                theta = float(sys.argv[4])
    data = mb.goto([x,y,theta])
    bag(filename, data)


