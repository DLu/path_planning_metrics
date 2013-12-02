#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_simulation')
import rospy
from path_planning_simulation import GazeboHelper

if __name__=='__main__':
    rospy.init_node('spawn_again')
    g = GazeboHelper()
    g.spawn_robot_maybe()
