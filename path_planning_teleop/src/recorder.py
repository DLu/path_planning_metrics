#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_teleop')
import rospy
from path_planning_teleop import *
from path_planning_ruler import *
from path_planning_simulation import *

if __name__=='__main__':
    rospy.init_node('teleop_path_recorder')
    g = GazeboHelper()    
    g.set_state('pr2', get_pose(0,0,0))

    tpr = TeleopPathRecorder()
    data = tpr.goto(1, 0, 0)
    bag('/home/dlu/Desktop/teleop.bag', data)

